#include "TaskManager.hpp"
#include "DispatchManager.hpp"
#include "TaskScheduler.hpp"
#include "Robo.hpp"

void TaskManager::Init()
{    
    EventManager::GetInstance().Subscribe(RobotEventType::IDLE_READY,[this](const RobotEvent& _e){OnRobotIdle(_e);});
    EventManager::GetInstance().Subscribe(RobotEventType::PICKUP_COMPLETED,[this](const RobotEvent& _e){OnRobotLoadCompleted(_e);});
    EventManager::GetInstance().Subscribe(RobotEventType::DROP_COMPLETED,[this](const RobotEvent& _e){OnRobotUnloadCompleted(_e);});
}

void TaskManager::OnRobotIdle(const RobotEvent& _e)
{
    uint32_t loadNodeID = DispatchManager::GetInstance().FindBestLoadNode(_e.timestamp,_e.agvID);
    TaskScheduler::GetInstance().AssignRoute(_e.agvID,loadNodeID,_e.timestamp,AGVState::MOVE_TO_PICKUP);
}

void TaskManager::OnRobotLoadCompleted(const RobotEvent& _e)
{
    uint32_t unloadNodeID = DispatchManager::GetInstance().FindBestDispatchNode(_e.timestamp,_e.agvID);
    TaskScheduler::GetInstance().AssignRoute(_e.agvID,unloadNodeID,_e.timestamp,AGVState::MOVE_TO_DROP);
}

void TaskManager::OnRobotUnloadCompleted(const RobotEvent& _e)
{    
    uint32_t homeNodeID = DispatchManager::GetInstance().FindHomeNode(_e.timestamp,_e.agvID);
    TaskScheduler::GetInstance().AssignRoute(_e.agvID,homeNodeID,_e.timestamp,AGVState::MOVE_TO_HOME);
}