#include "MovementSimulator.hpp"
#include "AGVKinematics.hpp"
#include "BezierFollower.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace
{
    constexpr float NODE_RELEASE_OFFSET = 1.1f;
    constexpr float DESTINATION_STOP_OFFSET = 1.2f;
    constexpr float EXECUTION_BLOCK_TIMEOUT = 1.0f;
    constexpr uint32_t EXECUTION_BLOCK_ATTEMPTS = 12;

    float GetLinkDistance(const MovementCachedLink& cache)
    {
        if (cache.link.m_Type == 1 && cache.link.m_Dist > 0.0f)
            return cache.link.m_Dist;

        const float dx = cache.toNode.m_PosX - cache.fromNode.m_PosX;
        const float dz = cache.toNode.m_PosZ - cache.fromNode.m_PosZ;
        float dist = std::sqrt(dx * dx + dz * dz);
        return (dist > 0.0f) ? dist : 1.0f;
    }

    float ClampRatio(float value, float minValue, float maxValue)
    {
        return std::max(minValue, std::min(value, maxValue));
    }

    void StepHeadingTowards(float& heading, float& angularVelocity, float targetHeading, float dt)
    {
        const float headingError = AGVKinematics::NormalizeAngle(targetHeading - heading);
        const float targetAngularVelocity = std::clamp(
            headingError * AGVKinematics::HEADING_KP,
            -AGVKinematics::MAX_ANGULAR_SPEED,
            AGVKinematics::MAX_ANGULAR_SPEED
        );

        angularVelocity = AGVKinematics::MoveAngularVelocityTowards(angularVelocity, targetAngularVelocity, dt);

        const float step = angularVelocity * dt;
        if (std::abs(step) >= std::abs(headingError))
        {
            heading = targetHeading;
            angularVelocity = 0.0f;
            return;
        }

        heading = AGVKinematics::NormalizeAngle(heading + step);
    }
}

MovementSimulator::MovementSimulator(uint32_t agvID, float x, float z, float heading)
{
    m_CurrentRoute.agvID = agvID;
    m_X = x;
    m_Z = z;
    m_Heading = heading;
}

void MovementSimulator::LoadRoute(const RoutePacket& routePacket)
{
    while (!m_EventQueue.empty()) { m_EventQueue.pop(); }

    m_CurrentRoute = routePacket;
    m_CurrentLinkIndex = 0;
    m_LinkProgress = 0.0f;
    m_DistanceOnLink = 0.0f;
    m_CurrentVelocity = 0.0f;
    m_CurrentAngularVelocity = 0.0f;
    m_CachedLinks.clear();

    m_OverTime = 0.0f;
    m_IsMovingLink = false;
    m_ActualStartTime = 0.0f;
    m_HasReleasedFromNode = false;
    m_ExecutionWaitTime = 0.0f;
    m_ExecutionWaitAttempts = 0;
    m_BlockEventSent = false;

    if (routePacket.nodes.empty()) return;

    MapManager& map = MapManager::GetInstance();
    const MapNode startNode = map.GetMapNode(routePacket.nodes.front().nodeID);
    m_X = startNode.m_PosX;
    m_Z = startNode.m_PosZ;

    if (routePacket.nodes.size() < 2) return;

    const size_t linkCount = routePacket.nodes.size() - 1;
    m_CachedLinks.reserve(linkCount);
    for (size_t i = 0; i < linkCount; i++)
    {
        uint32_t fromID = routePacket.nodes[i].nodeID;
        uint32_t toID = routePacket.nodes[i + 1].nodeID;
        float depTime = routePacket.nodes[i].departureTime;
        float arrTime = routePacket.nodes[i + 1].arrivalTime;

        MapLink link = map.FindLink(fromID, toID);
        MapNode from = map.GetMapNode(fromID);
        MapNode to = map.GetMapNode(toID);

        m_CachedLinks.push_back({ link, from, to, depTime, arrTime });
    }
}

void MovementSimulator::CancelRoute()
{
    m_CachedLinks.clear();
    m_IsMovingLink = false;
    m_LinkProgress = 0.0f;
    m_DistanceOnLink = 0.0f;
    m_CurrentVelocity = 0.0f;
    m_CurrentAngularVelocity = 0.0f;
    m_OverTime = 0.0f;
    m_ExecutionWaitTime = 0.0f;
    m_ExecutionWaitAttempts = 0;
    m_BlockEventSent = false;
    while (!m_EventQueue.empty()) { m_EventQueue.pop(); }
}

StatusPacket MovementSimulator::GetStatus() const
{
    StatusPacket p;
    p.agvID = m_CurrentRoute.agvID;
    if (m_CurrentLinkIndex < m_CachedLinks.size()) p.currentLinkID = m_CachedLinks[m_CurrentLinkIndex].link.m_Id;
    else p.currentLinkID = 0;

    p.progress = m_LinkProgress;
    p.x = m_X;
    p.z = m_Z;
    p.heading = m_Heading;
    p.velocity = m_CurrentVelocity;
    p.battery = 100.0f;
    return p;
}

bool MovementSimulator::HasEvent() const
{
    return !m_EventQueue.empty();
}

MovementEvent MovementSimulator::PopEvent()
{
    if (m_EventQueue.empty()) return MovementEvent{ MovementEventType::NONE, 0, 0 };
    MovementEvent event = m_EventQueue.front();
    m_EventQueue.pop();
    return event;
}

void MovementSimulator::Update(float dt, float serverTime, const MovementCallbacks& callbacks)
{
    if (m_CachedLinks.empty() || m_CurrentLinkIndex >= m_CachedLinks.size()) return;

    const MovementCachedLink& cache = m_CachedLinks[m_CurrentLinkIndex];
    const float linkDist = GetLinkDistance(cache);
    const float travelDuration = AGVKinematics::EstimateStopToStopTravelTime(linkDist);

    if (!m_IsMovingLink)
    {
        if (serverTime < cache.departureTime) return;

        const float startTargetHeading = BezierFollower::Heading(cache.fromNode, cache.toNode, cache.link, 0.0f);
        const float startHeadingError = std::abs(AGVKinematics::NormalizeAngle(startTargetHeading - m_Heading));
        const float turnDuration = AGVKinematics::EstimateTurnInPlaceTime(startHeadingError);
        float dynamicExpectedArrival = serverTime + std::max(travelDuration + turnDuration, 0.01f);

        if (callbacks.tryOccupyEdge && !callbacks.tryOccupyEdge(cache.fromNode.m_Id, cache.toNode.m_Id, serverTime, dynamicExpectedArrival))
        {
            m_ExecutionWaitTime += dt;
            ++m_ExecutionWaitAttempts;
            if (!m_BlockEventSent &&
                (m_ExecutionWaitTime >= EXECUTION_BLOCK_TIMEOUT || m_ExecutionWaitAttempts >= EXECUTION_BLOCK_ATTEMPTS))
            {
                m_EventQueue.push({ MovementEventType::EXECUTION_BLOCKED, cache.fromNode.m_Id, cache.toNode.m_Id });
                m_BlockEventSent = true;
                std::cout << "[BLOCKED EVENT] AGV " << m_CurrentRoute.agvID
                          << " stuck before " << cache.fromNode.m_Id << "->" << cache.toNode.m_Id << "\n";
            }
            return;
        }

        m_IsMovingLink = true;
        m_HasReleasedFromNode = false;
        m_ExecutionWaitTime = 0.0f;
        m_ExecutionWaitAttempts = 0;
        m_BlockEventSent = false;

        m_ActualStartTime = serverTime;
        m_DistanceOnLink = 0.0f;
        m_LinkProgress = 0.0f;
        m_CurrentVelocity = 0.0f;
        m_CurrentAngularVelocity = 0.0f;
        m_OverTime = 0.0f;
    }

    const float destinationHoldProgress = ClampRatio(1.0f - (DESTINATION_STOP_OFFSET / linkDist), 0.25f, 0.90f);
    const float destinationHoldDistance = destinationHoldProgress * linkDist;
    const bool shouldHoldBeforeDestination = callbacks.isNodeFree && !callbacks.isNodeFree(cache.toNode.m_Id);
    const float currentTargetHeading = BezierFollower::Heading(cache.fromNode, cache.toNode, cache.link, m_LinkProgress);
    const float headingError = std::abs(AGVKinematics::NormalizeAngle(currentTargetHeading - m_Heading));
    const float headingSpeedScale = AGVKinematics::HeadingSpeedScale(headingError);

    float targetDistance = shouldHoldBeforeDestination ? destinationHoldDistance : linkDist;
    if (targetDistance < m_DistanceOnLink)
        targetDistance = m_DistanceOnLink;

    const float remainingDistance = std::max(0.0f, targetDistance - m_DistanceOnLink);
    const float stopLimitedSpeed = AGVKinematics::StopLimitedSpeed(remainingDistance);
    const float targetVelocity = std::min(AGVKinematics::MAX_SPEED, stopLimitedSpeed) * headingSpeedScale;

    const float prevVelocity = m_CurrentVelocity;
    m_CurrentVelocity = AGVKinematics::MoveVelocityTowards(m_CurrentVelocity, targetVelocity, dt);

    float moveDistance = (prevVelocity + m_CurrentVelocity) * 0.5f * dt;
    if (moveDistance >= remainingDistance)
    {
        moveDistance = remainingDistance;
        m_CurrentVelocity = 0.0f;
    }

    m_DistanceOnLink += moveDistance;
    m_LinkProgress = ClampRatio(m_DistanceOnLink / linkDist, 0.0f, 1.0f);

    if (m_IsMovingLink && !m_HasReleasedFromNode)
    {
        float dynamicReleaseRatio = ClampRatio(NODE_RELEASE_OFFSET / linkDist, 0.20f, 0.70f);

        if (m_LinkProgress >= dynamicReleaseRatio)
        {
            if (callbacks.onNodeLeave) callbacks.onNodeLeave(cache.fromNode.m_Id);
            m_HasReleasedFromNode = true;
        }
    }

    Vector2 pos = BezierFollower::Evaluate(cache.fromNode, cache.toNode, cache.link, m_LinkProgress);
    m_X = pos.x;
    m_Z = pos.z;

    const float targetHeading = BezierFollower::Heading(cache.fromNode, cache.toNode, cache.link, m_LinkProgress);
    StepHeadingTowards(m_Heading, m_CurrentAngularVelocity, targetHeading, dt);

    if (shouldHoldBeforeDestination && m_DistanceOnLink >= destinationHoldDistance - 0.001f)
    {
        m_LinkProgress = destinationHoldProgress;
        m_DistanceOnLink = destinationHoldDistance;
        m_CurrentVelocity = 0.0f;
        Vector2 holdPos = BezierFollower::Evaluate(cache.fromNode, cache.toNode, cache.link, m_LinkProgress);
        m_X = holdPos.x;
        m_Z = holdPos.z;
        const float holdHeading = BezierFollower::Heading(cache.fromNode, cache.toNode, cache.link, m_LinkProgress);
        StepHeadingTowards(m_Heading, m_CurrentAngularVelocity, holdHeading, dt);
        return;
    }

    if (m_LinkProgress >= 1.0f)
    {
        if (callbacks.canEnterNode && !callbacks.canEnterNode(cache.toNode.m_Id))
        {
            m_LinkProgress = destinationHoldProgress;
            m_DistanceOnLink = destinationHoldDistance;
            m_CurrentVelocity = 0.0f;
            Vector2 holdPos = BezierFollower::Evaluate(cache.fromNode, cache.toNode, cache.link, destinationHoldProgress);
            m_X = holdPos.x;
            m_Z = holdPos.z;
            const float holdHeading = BezierFollower::Heading(cache.fromNode, cache.toNode, cache.link, destinationHoldProgress);
            StepHeadingTowards(m_Heading, m_CurrentAngularVelocity, holdHeading, dt);
            return;
        }

        m_X = cache.toNode.m_PosX;
        m_Z = cache.toNode.m_PosZ;
        m_CurrentVelocity = 0.0f;
        m_CurrentAngularVelocity = 0.0f;
        m_DistanceOnLink = linkDist;
        m_OverTime = 0.0f;

        m_EventQueue.push({ MovementEventType::ARRIVED, cache.toNode.m_Id, 0 });

        m_CurrentLinkIndex++;
        m_IsMovingLink = false;
        m_DistanceOnLink = 0.0f;
        m_LinkProgress = 0.0f;

        if (m_CurrentLinkIndex >= m_CachedLinks.size())
        {
            m_LinkProgress = 0.0f;
            m_OverTime = 0.0f;
        }
        return;
    }
}
