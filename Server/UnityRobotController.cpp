#include "UnityRobotController.hpp"
#include "Map.hpp"
#include "BezierFollower.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>

const float NODE_RELEASE_OFFSET = 1.1f;
const float DESTINATION_STOP_OFFSET = 1.2f;
const float EXECUTION_BLOCK_TIMEOUT = 1.0f;
const uint32_t EXECUTION_BLOCK_ATTEMPTS = 12;

static float GetLinkDistance(const CachedLink& cache)
{
    if (cache.link.m_Type == 1 && cache.link.m_Dist > 0.0f)
        return cache.link.m_Dist;

    const float dx = cache.toNode.m_PosX - cache.fromNode.m_PosX;
    const float dz = cache.toNode.m_PosZ - cache.fromNode.m_PosZ;
    float dist = std::sqrt(dx * dx + dz * dz);
    return (dist > 0.0f) ? dist : 1.0f;
}

static float ClampRatio(float value, float minValue, float maxValue)
{
    return std::max(minValue, std::min(value, maxValue));
}

UnityRobotController::UnityRobotController()
    : m_CurrentLinkIndex(0), m_LinkProgress(0.0f), m_IsMovingLink(false)
    , m_ActualStartTime(0.0f), m_OverTime(0.0f), m_HasReleasedFromNode(false)
    , m_X(0.0f), m_Z(0.0f), m_Heading(0.0f) {}

UnityRobotController::UnityRobotController(uint32_t agvID, float x, float z, float heading)
    : UnityRobotController()
{
    m_CurrentRoute.agvID = agvID;
    m_X = x;
    m_Z = z;
    m_Heading = heading;
}

UnityRobotController::~UnityRobotController() {}

void UnityRobotController::FollowRoute(const RoutePacket& _routePacket)
{   
    while(!m_EventQueue.empty()) { m_EventQueue.pop(); } 

    m_CurrentRoute = _routePacket;
    m_CurrentLinkIndex = 0;
    m_LinkProgress = 0.0f;
    m_CachedLinks.clear();
    
    m_OverTime = 0.0f;
    m_IsMovingLink = false;
    m_ActualStartTime = 0.0f;
    m_HasReleasedFromNode = false; 
    m_ExecutionWaitTime = 0.0f;
    m_ExecutionWaitAttempts = 0;
    m_BlockEventSent = false;

    if (_routePacket.nodes.empty()) return;

    MapManager& map = MapManager::GetInstance();
    const MapNode startNode = map.GetMapNode(_routePacket.nodes.front().nodeID);
    m_X = startNode.m_PosX;
    m_Z = startNode.m_PosZ;

    if (_routePacket.nodes.size() < 2) return;

    const size_t linkCount = _routePacket.nodes.size() - 1;
    m_CachedLinks.reserve(linkCount);
    for (size_t i = 0; i < linkCount; i++)
    {
        uint32_t fromID = _routePacket.nodes[i].nodeID;
        uint32_t toID   = _routePacket.nodes[i+1].nodeID;
        float depTime   = _routePacket.nodes[i].departureTime;
        float arrTime   = _routePacket.nodes[i+1].arrivalTime;

        MapLink link = map.FindLink(fromID, toID);
        MapNode from = map.GetMapNode(fromID);
        MapNode to   = map.GetMapNode(toID);

        m_CachedLinks.push_back({ link, from, to, depTime, arrTime });
    }
}

void UnityRobotController::CancelRoute()
{
    m_CachedLinks.clear();
    m_IsMovingLink = false;
    m_LinkProgress = 0.0f;
    m_OverTime = 0.0f;
    m_ExecutionWaitTime = 0.0f;
    m_ExecutionWaitAttempts = 0;
    m_BlockEventSent = false;
    while(!m_EventQueue.empty()) { m_EventQueue.pop(); }
}

StatusPacket UnityRobotController::GetStatus()
{
    StatusPacket p;
    p.agvID = m_CurrentRoute.agvID;
    if(m_CurrentLinkIndex < m_CachedLinks.size()) p.currentLinkID = m_CachedLinks[m_CurrentLinkIndex].link.m_Id;
    else p.currentLinkID = 0; 
    
    p.progress = m_LinkProgress;
    p.x = m_X; p.z = m_Z; p.heading = m_Heading; 
    p.velocity = 4.0f; p.battery = 100.0f; 
    return p;
}

bool UnityRobotController::HasEvent() const { return !m_EventQueue.empty(); }

ControllerEvent UnityRobotController::PopEvent()
{
    if(m_EventQueue.empty()) return ControllerEvent{ControllerEventType::NONE, 0};
    ControllerEvent event = m_EventQueue.front();
    m_EventQueue.pop();
    return event;
}

void UnityRobotController::Update(float dt, float serverTime)
{   
    if (m_CachedLinks.empty() || m_CurrentLinkIndex >= m_CachedLinks.size()) return; 

    const CachedLink& cache = m_CachedLinks[m_CurrentLinkIndex];
    float plannedDuration = cache.arrivalTime - cache.departureTime;
    if (plannedDuration <= 0.001f) plannedDuration = 1.0f;
    
    // ==========================================
    // 1. 대기 및 출발 결심 
    // ==========================================
    if (!m_IsMovingLink)
    {
        if (serverTime < cache.departureTime) return; 

        // 동적 남은 시간 및 도착 예상 시각 산출
        float remainingDuration = plannedDuration - m_OverTime;
        if (remainingDuration <= 0.01f) remainingDuration = 0.01f;
        float dynamicExpectedArrival = serverTime + remainingDuration;

        if (m_TryOccupyEdgeCallback && !m_TryOccupyEdgeCallback(cache.fromNode.m_Id, cache.toNode.m_Id, serverTime, dynamicExpectedArrival))
        {
            m_ExecutionWaitTime += dt;
            ++m_ExecutionWaitAttempts;
            if (!m_BlockEventSent && 
                (m_ExecutionWaitTime >= EXECUTION_BLOCK_TIMEOUT || m_ExecutionWaitAttempts >= EXECUTION_BLOCK_ATTEMPTS))
            {
                m_EventQueue.push({ ControllerEventType::EXECUTION_BLOCKED, cache.fromNode.m_Id, cache.toNode.m_Id });
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

        float compensatedTime = serverTime - m_OverTime;
        m_ActualStartTime = std::max(compensatedTime, cache.departureTime);
        m_OverTime = 0.0f; 
    }
    
    m_LinkProgress = (serverTime - m_ActualStartTime) / plannedDuration;

    // ==========================================
    // 2. [수정 완료] 차체 물리 기반 동적 꼬리 탈출 연산
    // ==========================================
    const float linkDist = GetLinkDistance(cache);
    if (m_IsMovingLink && !m_HasReleasedFromNode)
    {
        float dynamicReleaseRatio = ClampRatio(NODE_RELEASE_OFFSET / linkDist, 0.20f, 0.70f);

        if (m_LinkProgress >= dynamicReleaseRatio)
        {
            if (m_NodeLeaveCallback) m_NodeLeaveCallback(cache.fromNode.m_Id);
            m_HasReleasedFromNode = true;
        }
    }

    const float destinationHoldProgress = ClampRatio(1.0f - (DESTINATION_STOP_OFFSET / linkDist), 0.25f, 0.90f);
    if (m_IsMovingLink && m_IsNodeFreeCallback && m_LinkProgress >= destinationHoldProgress && !m_IsNodeFreeCallback(cache.toNode.m_Id))
    {
        m_LinkProgress = destinationHoldProgress;
        m_ActualStartTime = serverTime - (plannedDuration * destinationHoldProgress);
        Vector2 holdPos = BezierFollower::Evaluate(cache.fromNode, cache.toNode, cache.link, destinationHoldProgress);
        m_X = holdPos.x;
        m_Z = holdPos.z;
        m_Heading = BezierFollower::Heading(cache.fromNode, cache.toNode, cache.link, destinationHoldProgress);
        return;
    }

    // ==========================================
    // 3. 1프레임 도착 판정 (1프레임 지연 부여)
    // ==========================================
    if (m_LinkProgress >= 1.0f)
    {
        if (m_CanEnterNodeCallback && !m_CanEnterNodeCallback(cache.toNode.m_Id))
        {
            m_LinkProgress = destinationHoldProgress;
            m_ActualStartTime = serverTime - (plannedDuration * destinationHoldProgress);
            Vector2 holdPos = BezierFollower::Evaluate(cache.fromNode, cache.toNode, cache.link, destinationHoldProgress);
            m_X = holdPos.x;
            m_Z = holdPos.z;
            m_Heading = BezierFollower::Heading(cache.fromNode, cache.toNode, cache.link, destinationHoldProgress);
            return;
        }

        m_X = cache.toNode.m_PosX;
        m_Z = cache.toNode.m_PosZ;

        float exactArrivalTime = m_ActualStartTime + plannedDuration;
        m_OverTime = serverTime - exactArrivalTime; 

        m_EventQueue.push({ ControllerEventType::ARRIVED, cache.toNode.m_Id });
        
        m_CurrentLinkIndex++;
        m_IsMovingLink = false; 

        if (m_CurrentLinkIndex >= m_CachedLinks.size()) 
        {
            m_LinkProgress = 0.0f;
            m_OverTime = 0.0f; 
        }
        return; 
    }

    if (m_IsMovingLink)
    {
        Vector2 pos = BezierFollower::Evaluate(cache.fromNode, cache.toNode, cache.link, m_LinkProgress);
        m_X = pos.x; m_Z = pos.z;
        m_Heading = BezierFollower::Heading(cache.fromNode, cache.toNode, cache.link, m_LinkProgress);
    }
}
