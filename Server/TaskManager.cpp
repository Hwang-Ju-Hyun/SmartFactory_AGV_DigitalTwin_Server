#include "TaskManager.hpp"
#include "DispatchManager.hpp"
#include "TaskScheduler.hpp"
#include "Robo.hpp"
#include "RoutePlanner.hpp"
#include "WarehouseManager.hpp"

void TaskManager::Init()
{    
    EventManager::GetInstance().Subscribe(RobotEventType::IDLE_READY,[this](const RobotEvent& _e){OnRobotIdle(_e);});
    EventManager::GetInstance().Subscribe(RobotEventType::PICKUP_COMPLETED,[this](const RobotEvent& _e){OnRobotLoadCompleted(_e);});
    EventManager::GetInstance().Subscribe(RobotEventType::DROP_COMPLETED,[this](const RobotEvent& _e){OnRobotUnloadCompleted(_e);});
}

void TaskManager::ProcessNextDispatch()
{
    while(m_PendingEvents.empty()==false)
    {
        RobotEvent e = m_PendingEvents.front();
        uint32_t loadNodeID = DispatchManager::GetInstance().FindBestLoadNode(e.timestamp,e.agvID);

        // 재고가 없는 경우 로봇을 집으로 보내기
        if(loadNodeID==0)
        {
            uint32_t homeNode = DispatchManager::GetInstance().FindHomeNode(e.timestamp,e.agvID);
            RoutePlanner::GetInstance().CreateRoute(e.agvID,homeNode,e.timestamp,MissionPurpose::HOME);
            m_PendingEvents.pop();
            continue;
        }

        //재고 있으면 최적의 노드 찾아서 가자.
        WarehouseManager::GetInstance().ReserveStock(loadNodeID);
        RoutePlanner::GetInstance().CreateRoute(e.agvID,loadNodeID,e.timestamp,MissionPurpose::PICKUP);

        m_PendingEvents.pop();
    }    
}

void TaskManager::OnRobotIdle(const RobotEvent& _e)
{
    uint32_t loadNodeID = DispatchManager::GetInstance().FindBestLoadNode(_e.timestamp,_e.agvID);
    RoutePlanner::GetInstance().CreateRoute(_e.agvID, loadNodeID, _e.timestamp, MissionPurpose::PICKUP);
}

void TaskManager::OnRobotLoadCompleted(const RobotEvent& _e)
{
    uint32_t unloadNodeID = DispatchManager::GetInstance().FindBestDispatchNode(_e.timestamp,_e.agvID);
    RoutePlanner::GetInstance().CreateRoute(_e.agvID, unloadNodeID, _e.timestamp, MissionPurpose::DROP);
}

void TaskManager::OnRobotUnloadCompleted(const RobotEvent& _e)
{    
    RobotEvent newEvent = {RobotEventType::IDLE_READY, _e.agvID, _e.timestamp};
    m_PendingEvents.push(newEvent);
    ProcessNextDispatch();
}