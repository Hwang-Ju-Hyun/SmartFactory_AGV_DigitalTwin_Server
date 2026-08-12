#include "ESP32RobotController.hpp"
#include "Map.hpp"
#include "TrajectoryBuilder.hpp"
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

    TrajectoryBuildOptions options;
    options.hasTrustedStartHeading = true;
    options.startHeadingRad = m_RobotSession->GetStatus().heading;
    options.millimetersPerMapUnit = m_TrajectoryConfig.millimetersPerMapUnit;
    options.spacingMm = m_TrajectoryConfig.millimetersPerMapUnit;
    options.cruiseSpeedMmPerSecond = m_TrajectoryConfig.cruiseSpeedMmPerSecond;
    options.lineEndpointOnly = true;
    options.stopAtEveryNodeBoundary = true;

    RobotProtocol::TrajectoryCommandPayload trajectory;
    std::string error;
    if (!TrajectoryBuilder::Build(nodeIDs, 0, options, trajectory, error))
    {
        std::cout << "[PhysicalFleet] Trajectory build rejected: " << error << "\n";
        return false;
    }

    if (!m_RobotSession->SendTrajectory(std::move(trajectory)))
        return false;

    m_CurrentRoute = _routePacket;
    std::cout << "[PhysicalFleet] Automatic LINE trajectory sent. nodes=" << nodeIDs.size()
              << " scale=" << m_TrajectoryConfig.millimetersPerMapUnit << " mm/unit\n";
    return true;
}

void ESP32RobotController::CancelRoute()
{
    const uint32_t agvID = m_CurrentRoute.agvID != 0 ? m_CurrentRoute.agvID : (m_RobotSession ? m_RobotSession->GetAgvID() : 0);
    if (m_RobotSession)
        m_RobotSession->SendCancelRoute(agvID);
}

StatusPacket ESP32RobotController::GetStatus()
{
    if (!m_RobotSession)
        return StatusPacket{};

    return m_RobotSession->GetStatus();
}

bool ESP32RobotController::HasEvent() const
{
    return m_RobotSession && m_RobotSession->HasEvent();
}

ControllerEvent ESP32RobotController::PopEvent()
{
    if (!m_RobotSession)
        return ControllerEvent{ ControllerEventType::NONE, 0, 0 };

    // RoutePlanner가 계획상의 다음 node인지 검증한 뒤 occupancy를 갱신한다.
    return m_RobotSession->PopEvent();
}

void ESP32RobotController::Update(float dt, float serverTime)
{
    (void)dt;
    (void)serverTime;
}
