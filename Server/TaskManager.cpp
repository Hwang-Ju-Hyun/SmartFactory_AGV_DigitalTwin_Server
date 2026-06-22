#include "TaskManager.hpp"
#include "DispatchManager.hpp"
#include "TaskScheduler.hpp"
#include "Robo.hpp"
#include "RoutePlanner.hpp"
#include "WarehouseManager.hpp"
#include <iostream>

void TaskManager::Init()
{    
    EventManager::GetInstance().Subscribe(RobotEventType::IDLE_READY, [this](const RobotEvent& _e){ OnRobotIdle(_e); });
    EventManager::GetInstance().Subscribe(RobotEventType::PICKUP_COMPLETED, [this](const RobotEvent& _e){ OnRobotLoadCompleted(_e); });
    EventManager::GetInstance().Subscribe(RobotEventType::DROP_COMPLETED, [this](const RobotEvent& _e){ OnRobotUnloadCompleted(_e); });
}

//다목적 이벤트 대기실 (자리가 날 때마다 호출됨)
void TaskManager::ProcessNextDispatch()
{
    int qSize = m_PendingEvents.size();
    
    // 무한 루프 방지를 위해 딱 '현재 대기실에 있는 인원수'만큼만 검사함
    for(int i = 0; i < qSize; i++)
    {
        RobotEvent e = m_PendingEvents.front();
        m_PendingEvents.pop();

        if (e.type == RobotEventType::PICKUP_COMPLETED)
        {
            // 하차 자리가 없어서 대기 중이던 로봇 깨우기!
            OnRobotLoadCompleted(e); 
        }
        else if (e.type == RobotEventType::IDLE_READY)
        {
            // 상차 자리가 없어서 대기 중이던 로봇 깨우기!
            OnRobotIdle(e);
        }
    }    
}

void TaskManager::OnRobotIdle(const RobotEvent& _e)
{
    int loadNodeID = DispatchManager::GetInstance().FindBestLoadNode(_e.timestamp, _e.agvID); 
    
    if (loadNodeID == -1)
    {
        // 1. 현재 로봇 정보 가져오기
        Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(_e.agvID));
        if(!agv) return;

        // 2. 지금 내가 서 있는 곳이 '내 지정 주차장(Home)'인가?
        if (agv->GetToNodeID() == agv->GetHomeNode())
        {
            // 이미 내 주차장에 안전하게 알박기 중! 
            // 가만히 있으면 Robo.cpp의 타이머가 2초 뒤에 다시 이 함수를 깨워줌.
            return; 
        }
        else
        {
            // 3. 작업장에 멍때리고 서 있으면 민폐! 빨리 집(Home)으로 차 빼기!
            std::cout << "[관제탑] AGV " << _e.agvID << "번 일자리 없음! 작업장 통행을 위해 HOME(" << agv->GetHomeNode() << ")으로 복귀." << std::endl;
            
            RoutePlanner::GetInstance().CreateRoute(_e.agvID, agv->GetHomeNode(), _e.timestamp, MissionPurpose::HOME);
            return;
        }
    }
    RoutePlanner::GetInstance().CreateRoute(_e.agvID, loadNodeID, _e.timestamp, MissionPurpose::PICKUP);
    WarehouseManager::GetInstance().ReserveStock(loadNodeID);
}

void TaskManager::OnRobotLoadCompleted(const RobotEvent& _e)
{
    int unloadNodeID = DispatchManager::GetInstance().FindBestDispatchNode(_e.timestamp, _e.agvID);
    
    if (unloadNodeID == -1)
    {                
        Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(_e.agvID));
        if (!agv) return;

        // 1. 하차지가 날 때까지 무조건 대기실(m_PendingEvents)에 번호표 뽑고 대기!
        m_PendingEvents.push(_e);

        // 2. 상차장(Load Node)에서 멍때리고 길막하면 안 됨! 집(Home)으로 차 빼기!
        // 단, 이미 집(Home)에 있거나, 집으로 이동 중이라면 불필요한 경로 갱신은 안 함
        if (agv->GetToNodeID() != agv->GetHomeNode() && agv->GetMissionPurpose() != MissionPurpose::HOME)
        {
            std::cout << "[관제탑] AGV " << _e.agvID << "번 하차지 만석! 길막 방지를 위해 HOME으로 이동하며 대기." << std::endl;
            
            // 짐을 든 상태지만, 길을 비켜주기 위해 일단 HOME으로 출발!
            RoutePlanner::GetInstance().CreateRoute(_e.agvID, agv->GetHomeNode(), _e.timestamp, MissionPurpose::HOME);
        }                
        return;              
    }    
    RoutePlanner::GetInstance().CreateRoute(_e.agvID, unloadNodeID, _e.timestamp, MissionPurpose::DROP);
}

void TaskManager::OnRobotUnloadCompleted(const RobotEvent& _e)
{        
    RobotEvent newEvent = {RobotEventType::DROP_COMPLETED, _e.agvID, _e.timestamp};
    OnRobotIdle(newEvent);        
    ProcessNextDispatch();
}