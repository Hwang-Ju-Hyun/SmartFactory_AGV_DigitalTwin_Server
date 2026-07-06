#include "TaskManager.hpp"
#include "DispatchManager.hpp"
#include "RoutePlanner.hpp"
#include "WarehouseManager.hpp"
#include "AGVManager.hpp"
#include <iostream>
#include"NetworkManagerServer.hpp"

void TaskManager::Init()
{    
    EventManager::GetInstance().Subscribe(RobotEventType::IDLE_READY, [this](const RobotEvent& _e){ OnRobotIdle(_e); });
    EventManager::GetInstance().Subscribe(RobotEventType::PICKUP_COMPLETED, [this](const RobotEvent& _e){ OnRobotLoadCompleted(_e); });
    EventManager::GetInstance().Subscribe(RobotEventType::DROP_COMPLETED, [this](const RobotEvent& _e){ OnRobotUnloadCompleted(_e); });
}

void TaskManager::ProcessNextDispatch()
{
    int qSize = m_WaitingAGVs.size();
    
    // 현재 서버의 시간을 가져옵니다.
    float currentTime = NetworkManagerServer::sInstance->GetTotalElapsedServerTime();
    
    for(int i = 0; i < qSize; i++)
    {
        uint32_t agvID = m_WaitingAGVs.front();
        m_WaitingAGVs.pop();

        // 번호표 뽑고 기다리던 AGV들을 다시 깨워서 하차지 배정을 시도합니다.
        RobotEvent e;
        e.agvID = agvID;
        e.timestamp = currentTime; 

        OnRobotLoadCompleted(e);
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
    if (agv->GetState() != AGVState::IDLE)
    {
        std::cout << "AGV " << _e.agvID << "번은 진짜 IDLE이 아닙니다! (새 임무 지시 무시)" << std::endl;
        return; 
    }

    if (loadNodeID == -1)
    {
        // 작업장 길막 방지를 위해 집으로 차 빼기
        std::cout<<"TaskManager::OnRobotIDle CReateRoute Call : loadNodeID=-1"<<std::endl;
        RoutePlanner::GetInstance().CreateRoute(_e.agvID, agv->GetHomeNode(), _e.timestamp, MissionPurpose::HOME);
        return; // 집에 있으면 Robo.cpp의 2초 타이머가 다시 이 함수를 깨워줌!
    }
    
    WarehouseManager::GetInstance().ReserveStock(loadNodeID);
    std::cout<<"TaskManager::OnRobotIDle CReateRoute Call"<<std::endl;
    RoutePlanner::GetInstance().CreateRoute(_e.agvID, loadNodeID, _e.timestamp, MissionPurpose::PICKUP);
}

void TaskManager::OnRobotLoadCompleted(const RobotEvent& _e)
{
    int unloadNodeID = DispatchManager::GetInstance().FindBestDispatchNode(_e.timestamp, _e.agvID);
    Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(_e.agvID));
    if(!agv) return;    

    if (unloadNodeID == -1)
    {
        // 하차지가 날 때까지 대기실에 번호표(agvID)만 뽑고 갓길(집)로 피신!
        m_WaitingAGVs.push(_e.agvID); // 🌟 uint32_t만 푸시

        if (agv->GetToNodeID() != agv->GetHomeNode() && agv->GetMissionPurpose() != MissionPurpose::HOME)
        {            
            std::cout<<"[TaskManager] 하차지 없음. AGV " << _e.agvID << "번 HOME으로 대피!" << std::endl;
            RoutePlanner::GetInstance().CreateRoute(_e.agvID, agv->GetHomeNode(), _e.timestamp, MissionPurpose::HOME);
        }
        return;              
    }
    
    std::cout << "[TaskManager] 하차지 확보 완료! AGV " << _e.agvID << "번 DROP 이동 시작." << std::endl;
    RoutePlanner::GetInstance().CreateRoute(_e.agvID, unloadNodeID, _e.timestamp, MissionPurpose::DROP);
}

void TaskManager::OnRobotUnloadCompleted(const RobotEvent& _e)
{    
   RobotEvent newEvent = {RobotEventType::IDLE_READY, _e.agvID, _e.timestamp};
    OnRobotIdle(newEvent);

    // 내가 하차를 끝냈으니 자리가 났다! 번호표 뽑고 기다리던 인원들 깨우기!
    ProcessNextDispatch();
}