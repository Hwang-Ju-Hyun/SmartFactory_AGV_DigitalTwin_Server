#include "ESP32RobotController.hpp"
#include "Map.hpp"
#include "TrajectoryBuilder.hpp"
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

ESP32RobotController::ESP32RobotController(TCPSessionPtr _robotSession)
    : m_RobotSession(std::make_shared<RobotSession>(_robotSession, 0))
{
 
}

ESP32RobotController::ESP32RobotController(
    RobotSessionPtr _robotSession, ESP32TrajectoryExecutionConfig _trajectoryConfig)
    : m_RobotSession(_robotSession)
    , m_TrajectoryConfig(_trajectoryConfig)
{
}

bool ESP32RobotController::FollowRoute(const RoutePacket& _routePacket)
{
    if (!m_RobotSession)
        return false;

    if (!m_TrajectoryConfig.enabled || !m_RobotSession->SupportsTrajectoryCommand())
    {
        m_CurrentRoute = _routePacket;
        m_RobotSession->SendRoute(_routePacket);
        return true;
    }

    if (_routePacket.nodes.size() < 2)
    {
        std::cout << "[PhysicalFleet] Trajectory rejected: route needs at least two nodes\n";
        return false;
    }

    std::vector<uint32_t> nodeIDs;
    nodeIDs.reserve(_routePacket.nodes.size());
    for (const RouteNodeTime& node : _routePacket.nodes)
        nodeIDs.push_back(node.nodeID);

    MapManager& map = MapManager::GetInstance();
    for (size_t i = 0; i + 1 < nodeIDs.size(); ++i)
    {
        const MapLink& link = map.FindLink(nodeIDs[i], nodeIDs[i + 1]);
        if (link.m_Type != 0)
        {
            std::cout << "[PhysicalFleet] Trajectory rejected: only directed LINE links are allowed. "
                      << nodeIDs[i] << " -> " << nodeIDs[i + 1] << "\n";
            return false;
        }
    }

    m_CurrentRoute = _routePacket;
    m_CurrentEdgeStartIndex = 0;
    m_DispatchNextEdgePending = false;
    if (!DispatchCurrentPhysicalEdge())
    {
        m_CurrentRoute = {};
        return false;
    }

    std::cout << "[PhysicalFleet] Automatic LINE route armed edge-by-edge. nodes="
              << nodeIDs.size() << " scale="
              << m_TrajectoryConfig.millimetersPerMapUnit << " mm/unit\n";
    return true;
}

bool ESP32RobotController::DispatchCurrentPhysicalEdge()
{
    if (!m_RobotSession || m_CurrentRoute.nodes.size() < 2 ||
        m_CurrentEdgeStartIndex + 1 >= m_CurrentRoute.nodes.size())
    {
        return false;
    }

    const std::vector<uint32_t> nodeIDs{
        m_CurrentRoute.nodes[m_CurrentEdgeStartIndex].nodeID,
        m_CurrentRoute.nodes[m_CurrentEdgeStartIndex + 1].nodeID
    };

    TrajectoryBuildOptions options;
    options.hasTrustedStartHeading = true;
    options.startHeadingRad = m_HasConfirmedStartHeading
        ? m_ConfirmedStartHeadingRad
        : m_RobotSession->GetStatus().heading;
    options.millimetersPerMapUnit = m_TrajectoryConfig.millimetersPerMapUnit;
    options.spacingMm = m_TrajectoryConfig.millimetersPerMapUnit;
    options.cruiseSpeedMmPerSecond = m_TrajectoryConfig.cruiseSpeedMmPerSecond;
    options.cornerStopThresholdRad =
        m_TrajectoryConfig.initialHeadingToleranceRad;
    options.lineEndpointOnly = true;
    options.stopAtEveryNodeBoundary = true;

    std::cout << "[PhysicalFleet] Build edge " << nodeIDs.front()
              << " -> " << nodeIDs.back()
              << " startHeadingRad=" << options.startHeadingRad
              << " headingSource="
              << (!m_HasConfirmedStartHeading
                      ? "ROBOT_STATUS"
                      : (m_ConfirmedStartHeadingFromVision
                             ? "VISION_ACCEPTED"
                             : "NOMINAL_NODE"))
              << " visionSequence="
              << (m_ConfirmedStartHeadingFromVision
                      ? m_ConfirmedStartHeadingVisionSequence
                      : 0)
              << "\n";

    RobotProtocol::TrajectoryCommandPayload trajectory;
    std::string error;
    if (!TrajectoryBuilder::Build(nodeIDs, 0, options, trajectory, error))
    {
        std::cout << "[PhysicalFleet] Trajectory build rejected: " << error << "\n";
        return false;
    }

    if (!m_RobotSession->SendTrajectory(std::move(trajectory)))
        return false;

    m_DispatchNextEdgePending = false;
    std::cout << "[PhysicalFleet] LINE edge sent. " << nodeIDs.front()
              << " -> " << nodeIDs.back() << "\n";
    return true;
}

void ESP32RobotController::CancelRoute()
{
    const uint32_t agvID = m_CurrentRoute.agvID != 0 ? m_CurrentRoute.agvID : (m_RobotSession ? m_RobotSession->GetAgvID() : 0);
    if (m_RobotSession)
        m_RobotSession->SendCancelRoute(agvID);
    m_CurrentRoute = {};
    m_CurrentEdgeStartIndex = 0;
    m_DispatchNextEdgePending = false;
}

StatusPacket ESP32RobotController::GetStatus()
{
    if (!m_RobotSession)
        return StatusPacket{};

    return m_RobotSession->GetStatus();
}

bool ESP32RobotController::HasEvent() const
{
    return !m_LocalEvents.empty() ||
        (m_RobotSession && m_RobotSession->HasEvent());
}

ControllerEvent ESP32RobotController::PopEvent()
{
    if (!m_LocalEvents.empty())
    {
        ControllerEvent event = m_LocalEvents.front();
        m_LocalEvents.pop();
        return event;
    }
    if (!m_RobotSession)
        return ControllerEvent{ ControllerEventType::NONE, 0, 0 };

    // RoutePlanner가 계획상의 다음 node인지 검증한 뒤 occupancy를 갱신한다.
    return m_RobotSession->PopEvent();
}

void ESP32RobotController::Update(float dt, float serverTime)
{
    (void)dt;
    (void)serverTime;
    if (m_DispatchNextEdgePending && !DispatchCurrentPhysicalEdge())
    {
        ControllerEvent event;
        event.type = ControllerEventType::EXECUTION_BLOCKED;
        event.nodeID = m_CurrentRoute.nodes[m_CurrentEdgeStartIndex].nodeID;
        event.relatedNodeID =
            m_CurrentRoute.nodes[m_CurrentEdgeStartIndex + 1].nodeID;
        m_LocalEvents.push(event);
        m_DispatchNextEdgePending = false;
    }
}

bool ESP32RobotController::IsExpectedPhysicalArrival(uint32_t nodeID) const
{
    return m_TrajectoryConfig.enabled && m_CurrentRoute.nodes.size() >= 2 &&
        m_CurrentEdgeStartIndex + 1 < m_CurrentRoute.nodes.size() &&
        m_CurrentRoute.nodes[m_CurrentEdgeStartIndex + 1].nodeID == nodeID;
}

bool ESP32RobotController::ConfirmCorrectedPhysicalArrival(
    uint32_t nodeID,
    std::optional<PhysicalFleetHeadingAnchor> visionAnchor)
{
    if (!IsExpectedPhysicalArrival(nodeID))
        return false;

    float confirmedHeadingRad = 0.0f;
    if (!TryGetExpectedArrivalHeading(nodeID, confirmedHeadingRad))
        return false;

    // Prefer the accepted fresh Vision heading. If it is unavailable, anchor
    // to the confirmed incoming-link heading instead of reusing the ESP32's
    // accumulated open-loop estimate.
    const ResolvedPhysicalFleetHeading resolved =
        ResolvePhysicalFleetHeading(confirmedHeadingRad, visionAnchor);
    if (!std::isfinite(resolved.headingRad))
        return false;
    m_ConfirmedStartHeadingRad = resolved.headingRad;
    m_HasConfirmedStartHeading = true;
    m_ConfirmedStartHeadingFromVision = resolved.usedVision;
    m_ConfirmedStartHeadingVisionSequence = resolved.visionSequence;
    std::cout << "[PhysicalFleet] Arrival heading anchored. node=" << nodeID
              << " headingRad=" << resolved.headingRad
              << " headingSource="
              << (resolved.usedVision ? "VISION_ACCEPTED" : "NOMINAL_NODE")
              << " visionSequence=" << resolved.visionSequence << "\n";

    ++m_CurrentEdgeStartIndex;
    if (m_CurrentEdgeStartIndex + 1 < m_CurrentRoute.nodes.size())
        m_DispatchNextEdgePending = true;
    else
    {
        m_CurrentRoute = {};
        m_CurrentEdgeStartIndex = 0;
        m_DispatchNextEdgePending = false;
    }
    return true;
}

bool ESP32RobotController::TryGetExpectedArrivalHeading(
    uint32_t nodeID,
    float& outHeadingRad) const
{
    if (!IsExpectedPhysicalArrival(nodeID))
        return false;

    const MapNode from = MapManager::GetInstance().GetMapNode(
        m_CurrentRoute.nodes[m_CurrentEdgeStartIndex].nodeID);
    const MapNode to = MapManager::GetInstance().GetMapNode(nodeID);
    outHeadingRad = std::atan2(to.m_PosZ - from.m_PosZ,
                               to.m_PosX - from.m_PosX);
    return std::isfinite(outHeadingRad);
}

bool ESP32RobotController::TryGetExpectedPhysicalEdge(
    uint32_t targetNodeID,
    uint32_t& outStartNodeID) const
{
    if (!IsExpectedPhysicalArrival(targetNodeID))
        return false;

    outStartNodeID =
        m_CurrentRoute.nodes[m_CurrentEdgeStartIndex].nodeID;
    return true;
}

uint32_t ESP32RobotController::GetActivePhysicalRouteID() const
{
    return m_RobotSession ?
        m_RobotSession->GetLastSentTrajectoryRouteID() : 0;
}

bool ESP32RobotController::SupportsNodeCorrection() const
{
    return m_RobotSession && m_RobotSession->SupportsNodeCorrection();
}
