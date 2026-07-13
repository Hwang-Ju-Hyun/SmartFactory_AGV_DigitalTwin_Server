#include "ESP32RobotController.hpp"

ESP32RobotController::ESP32RobotController(TCPSessionPtr _robotSession)
    : m_RobotSession(std::make_shared<RobotSession>(_robotSession, 0))
{
 
}

ESP32RobotController::ESP32RobotController(RobotSessionPtr _robotSession)
    : m_RobotSession(_robotSession)
{
}

void ESP32RobotController::FollowRoute(const RoutePacket& _routePacket)
{
    m_CurrentRoute = _routePacket;
    if (!_routePacket.nodes.empty())
        m_LastKnownNodeID = _routePacket.nodes.front().nodeID;

    if (m_RobotSession)
        m_RobotSession->SendRoute(_routePacket);
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

    ControllerEvent event = m_RobotSession->PopEvent();
    if (event.type == ControllerEventType::ARRIVED)
    {
        if (m_NodeLeaveCallback && m_LastKnownNodeID != 0 && m_LastKnownNodeID != event.nodeID)
            m_NodeLeaveCallback(m_LastKnownNodeID);

        if (m_CanEnterNodeCallback)
            m_CanEnterNodeCallback(event.nodeID);

        m_LastKnownNodeID = event.nodeID;
    }

    return event;
}

void ESP32RobotController::Update(float dt, float serverTime)
{
    (void)dt;
    (void)serverTime;
}
