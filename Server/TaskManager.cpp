#include "TaskManager.hpp"
#include "DispatchManager.hpp"
#include "RoutePlanner.hpp"
#include "WarehouseManager.hpp"
#include "AGVManager.hpp"
#include <iostream>

void TaskManager::Init()
{    
    EventManager::GetInstance().Subscribe(RobotEventType::IDLE_READY, [this](const RobotEvent& _e){ OnRobotIdle(_e); });
    EventManager::GetInstance().Subscribe(RobotEventType::PICKUP_COMPLETED, [this](const RobotEvent& _e){ OnRobotLoadCompleted(_e); });
    EventManager::GetInstance().Subscribe(RobotEventType::DROP_COMPLETED, [this](const RobotEvent& _e){ OnRobotUnloadCompleted(_e); });
}

void TaskManager::ProcessNextDispatch()
{
    int qSize = m_PendingEvents.size();
    
    for(int i = 0; i < qSize; i++)
    {
        RobotEvent e = m_PendingEvents.front();
        m_PendingEvents.pop();

        if (e.type == RobotEventType::PICKUP_COMPLETED)
        {
            OnRobotLoadCompleted(e); 
        }            
        else if (e.type == RobotEventType::IDLE_READY) 
        {
            OnRobotIdle(e);
        }
    }    
}

void TaskManager::OnRobotIdle(const RobotEvent& _e)
{
    int loadNodeID = DispatchManager::GetInstance().FindBestLoadNode(_e.timestamp, _e.agvID); 
    Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(_e.agvID));
    if(!agv) 
    {
        return;
    }

    if (loadNodeID == -1)
    {
        // 작업장 길막 방지를 위해 집으로 차 빼기
        if (agv->GetToNodeID() != agv->GetHomeNode())
        {            
            RoutePlanner::GetInstance().CreateRoute(_e.agvID, agv->GetHomeNode(), _e.timestamp, MissionPurpose::HOME);
        }
        return; // 집에 있으면 Robo.cpp의 2초 타이머가 다시 이 함수를 깨워줌!
    }
    
    WarehouseManager::GetInstance().ReserveStock(loadNodeID);
    RoutePlanner::GetInstance().CreateRoute(_e.agvID, loadNodeID, _e.timestamp, MissionPurpose::PICKUP);
}

void TaskManager::OnRobotLoadCompleted(const RobotEvent& _e)
{
    int unloadNodeID = DispatchManager::GetInstance().FindBestDispatchNode(_e.timestamp, _e.agvID);
    if (unloadNodeID == -1)
    {
        returnl
    }
    Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(_e.agvID));
    if(!agv) 
    {
        return;
    }

    if (unloadNodeID == -1)
    {
        // 하차지가 날 때까지 대기실에 번호표 뽑고 갓길(집)로 피신!
        m_PendingEvents.push(_e);

        if (agv->GetToNodeID() != agv->GetHomeNode() && agv->GetMissionPurpose() != MissionPurpose::HOME)
        {            
            RoutePlanner::GetInstance().CreateRoute(_e.agvID, agv->GetHomeNode(), _e.timestamp, MissionPurpose::HOME);
        }
        return;              
    }
    
    RoutePlanner::GetInstance().CreateRoute(_e.agvID, unloadNodeID, _e.timestamp, MissionPurpose::DROP);
}

void TaskManager::OnRobotUnloadCompleted(const RobotEvent& _e)
{    
    RobotEvent newEvent = {RobotEventType::IDLE_READY, _e.agvID, _e.timestamp};
    OnRobotIdle(newEvent);

    // 내가 하차를 끝냈으니 자리가 났다! 대기실 인원들 깨우기!
    ProcessNextDispatch();
}