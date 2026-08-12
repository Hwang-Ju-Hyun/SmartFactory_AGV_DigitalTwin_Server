#include "UnityRobotController.hpp"

UnityRobotController::UnityRobotController()
{
}

UnityRobotController::UnityRobotController(uint32_t agvID, float x, float z, float heading)
    : m_Simulator(agvID, x, z, heading)
{
}

UnityRobotController::~UnityRobotController() {}

bool UnityRobotController::FollowRoute(const RoutePacket& _routePacket)
{   
    while(!m_EventQueue.empty()) { m_EventQueue.pop(); } 
    m_Simulator.LoadRoute(_routePacket);
    return true;
}

void UnityRobotController::CancelRoute()
{
    m_Simulator.CancelRoute();
    while(!m_EventQueue.empty()) { m_EventQueue.pop(); }
}

StatusPacket UnityRobotController::GetStatus()
{
    return m_Simulator.GetStatus();
}

bool UnityRobotController::HasEvent() const { return !m_EventQueue.empty() || m_Simulator.HasEvent(); }

ControllerEvent UnityRobotController::PopEvent()
{
    DrainSimulatorEvents();
    if(m_EventQueue.empty()) return ControllerEvent{ControllerEventType::NONE, 0, 0};
    ControllerEvent event = m_EventQueue.front();
    m_EventQueue.pop();
    return event;
}

void UnityRobotController::Update(float dt, float serverTime)
{   
    MovementCallbacks callbacks;
    callbacks.tryOccupyEdge = m_TryOccupyEdgeCallback;
    callbacks.onNodeLeave = m_NodeLeaveCallback;
    callbacks.canEnterNode = m_CanEnterNodeCallback;
    callbacks.isNodeFree = m_IsNodeFreeCallback;

    m_Simulator.Update(dt, serverTime, callbacks);
    DrainSimulatorEvents();
}

void UnityRobotController::DrainSimulatorEvents()
{
    while (m_Simulator.HasEvent())
    {
        MovementEvent movementEvent = m_Simulator.PopEvent();
        switch (movementEvent.type)
        {
        case MovementEventType::ARRIVED:
            m_EventQueue.push({ ControllerEventType::ARRIVED, movementEvent.nodeID, movementEvent.relatedNodeID });
            break;
        case MovementEventType::EXECUTION_BLOCKED:
            m_EventQueue.push({ ControllerEventType::EXECUTION_BLOCKED, movementEvent.nodeID, movementEvent.relatedNodeID });
            break;
        case MovementEventType::NONE:
            break;
        }
    }
}
