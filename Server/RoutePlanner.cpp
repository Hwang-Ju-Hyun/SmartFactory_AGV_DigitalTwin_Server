#include "RoutePlanner.hpp"
#include "AGVManager.hpp"
#include "PathFinder.hpp"
#include "TrafficControlManager.hpp" // 새로 만든 시공간 예약 장부!
#include "Map.hpp"
#include <iostream>

const int WINDOW_SIZE = 16; // WHCA* 논문 권장 탐색 뎁스
const float AGV_SPEED = 3.8f;
const float WAIT_TIME = 1.0f;

void RoutePlanner::Init()
{    
    EventManager::GetInstance().Subscribe(RobotEventType::MOVING_WAITING_COMPLETED,[this](const RobotEvent& _e) { OnRobotStepCompleted(_e); });
    std::cout << "[RoutePlanner Init]" << std::endl;
}

void RoutePlanner::CreateRoute(uint32_t _agvID, uint32_t _targetNodeID, float _serverTime, MissionPurpose _purpose)
{
    Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(_agvID));
    if (!agv) return;
    std::cout
    << "[ROUTE] agv="
    << _agvID
    << " serverTime="
    << _serverTime
    << std::endl;
    uint32_t curNodeID = agv->GetFromNodeID();

    // 1. 이미 도착한 경우 (제자리 알박기)
    if (curNodeID == _targetNodeID)
    {
        TrafficManager::GetInstance().ClearFutureReservations(_agvID, _serverTime);
        //TrafficManager::GetInstance().ReserveNode(curNodeID, _serverTime, _serverTime + 99999.0f, _agvID);
        TrafficManager::GetInstance().ParkNode(curNodeID, _agvID,_serverTime);
        
        agv->SetMissionPurpose(_purpose);
        if (_purpose == MissionPurpose::HOME) 
            agv->AssignNextStep(MapManager::GetInstance().GetMapNode(curNodeID), MapManager::GetInstance().GetMapNode(curNodeID), AGVState::IDLE,_serverTime);
        else if (_purpose == MissionPurpose::PICKUP) 
            agv->AssignNextStep(MapManager::GetInstance().GetMapNode(curNodeID), MapManager::GetInstance().GetMapNode(curNodeID), AGVState::LOADING,_serverTime);
        else if (_purpose == MissionPurpose::DROP) 
            agv->AssignNextStep(MapManager::GetInstance().GetMapNode(curNodeID), MapManager::GetInstance().GetMapNode(curNodeID), AGVState::UNLOADING,_serverTime);
        return;
    }

    // 2. WHCA* 코어 호출 (시공간 탐색)
    if (m_RRAEngines.find(_targetNodeID) == m_RRAEngines.end())
    {
        m_RRAEngines[_targetNodeID] = RRAStar();
        m_RRAEngines[_targetNodeID].Init(_targetNodeID); 
    }
    PathFinder pf;
    std::vector<uint32_t> path = pf.FindPath(curNodeID, _targetNodeID, _agvID, _serverTime, WINDOW_SIZE, m_RRAEngines[_targetNodeID]);
    
    if (path.size() < 2)
    {
        // [실패] 사방이 막혔거나 유효한 경로(최소 길이 2)를 찾지 못한 경우
        m_MasterPlans.erase(_agvID); 

        std::cout << "[관제탑] AGV " << _agvID << "번 시공간 체증 감지! 대기 후 재탐색." << std::endl;
                        
        // 발밑만 안전하게 확보하고 대기
        TrafficManager::GetInstance().ClearFutureReservations(_agvID, _serverTime);
        TrafficManager::GetInstance().ParkNode(curNodeID, _agvID,_serverTime);
        
        // 2초 뒤 재시도 큐에 등록
        m_PendingRoutes.push_back({ _agvID, _targetNodeID, _purpose, 2.0f });
        
        return; // 실패했으니 여기서 함수 조기 종료!
    }

    // =========================================================
    // 여기부터는 성공했을 때의 로직
    // =========================================================
    
    // 새 길을 찾았으니 과거~현재 예약만 남기고 미래 장부 삭제 및 새 예약
    TrafficManager::GetInstance().ClearFutureReservations(_agvID, _serverTime);
    ReserveRouteTimeline(_agvID, path, _serverTime, _targetNodeID);

    // 로봇 마스터 장부에 계획표 저장
    RoutePlan plan;
    plan.agvID = _agvID;
    plan.currentStepIndex = 0;
    plan.purpose = _purpose;
    plan.finalTargetNodeID = _targetNodeID;

    // 여기서 path.size() - 1 을 할 때 path.size()가 2 이상임이 위에서 보장됨!
    for (size_t i = 0; i < path.size() - 1; ++i)
    {
        plan.steps.push_back({ path[i], path[i + 1] });
    }
    m_MasterPlans[_agvID] = plan;

    // 첫 발걸음 내딛기!
    auto fromNode = MapManager::GetInstance().GetMapNode(plan.steps[0].fromNodeID);
    auto toNode = MapManager::GetInstance().GetMapNode(plan.steps[0].toNodeID);
    
    agv->SetMissionPurpose(_purpose);
    agv->AssignNextStep(fromNode, toNode, AGVState::MOVING,_serverTime);
    
    std::cout << "[관제탑] AGV " << _agvID << "번 경로 발급 완료. 목적지: " << _targetNodeID << std::endl;
}

void RoutePlanner::ReserveRouteTimeline(uint32_t _agvID, const std::vector<uint32_t>& _path, float _serverTime, uint32_t _finalTargetID)
{
    float accTime = _serverTime;
    
    // 출발 발밑 예약
    TrafficManager::GetInstance().ReserveNode(_path[0], accTime, accTime + 0.04f, _agvID);

    for (size_t i = 0; i < _path.size() - 1; i++)
    {
        uint32_t fromID = _path[i];
        uint32_t toID = _path[i + 1];

        if (fromID == toID) // 제자리 대기(Wait)
        {
            float leaveTime = accTime + WAIT_TIME;
            TrafficManager::GetInstance().ReserveNode(toID, accTime, leaveTime, _agvID);
            accTime = leaveTime; 
            continue;
        }
        
        auto fromNode = MapManager::GetInstance().GetMapNode(fromID);
        auto toNode = MapManager::GetInstance().GetMapNode(toID);

        float dist = std::sqrt(std::pow(fromNode.m_PosX - toNode.m_PosX, 2) + std::pow(fromNode.m_PosZ - toNode.m_PosZ, 2));
        float travelTime = dist / AGV_SPEED;
       
        float linkEnterTime = accTime;               
        float linkLeaveTime = accTime + travelTime;  
        
        TrafficManager::GetInstance().ReserveLink(fromID, toID, linkEnterTime, linkLeaveTime, _agvID);
        TrafficManager::GetInstance().ReserveNode(fromID, accTime, accTime + 0.04f, _agvID);

        float nodeEnterTime = linkLeaveTime;

        if (i + 1 == _path.size() - 1) 
        {         
            if (toID == _finalTargetID)
            {
                //진짜 최종 목적지일 때만 영구 알박기!
                TrafficManager::GetInstance().ParkNode(toID, _agvID, nodeEnterTime);            
            }
            else 
            {
                // 윈도우 컷오프(임시 목적지)라면 영구 알박기(ParkNode) 금지!
                // 다음 AGV 재탐색이 시작될 때까지의 최소한의 공백(예: 0.1초)만 징검다리처럼 예약함
                TrafficManager::GetInstance().ReserveNode(toID, nodeEnterTime, nodeEnterTime + WAIT_TIME, _agvID);
            }             
        }
        else 
        {         
            TrafficManager::GetInstance().ReserveNode(toID, nodeEnterTime, nodeEnterTime + 0.04f, _agvID);
        }        

        accTime = nodeEnterTime;
    }
}

void RoutePlanner::OnRobotStepCompleted(const RobotEvent& _e)
{
    uint32_t agvID = _e.agvID;
    if (m_MasterPlans.find(agvID) == m_MasterPlans.end()) return;
    
    RoutePlan& plan = m_MasterPlans[agvID];
    Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(agvID));

    plan.currentStepIndex++; 

    // 1. 갈 길이 남았다면 (또는 윈도우 컷오프 지점 전이라면)
    if (plan.currentStepIndex < plan.steps.size())
    {
        uint32_t nextFrom = plan.steps[plan.currentStepIndex].fromNodeID;
        uint32_t nextTo   = plan.steps[plan.currentStepIndex].toNodeID;
        
        auto fromNode = MapManager::GetInstance().GetMapNode(nextFrom);
        auto toNode   = MapManager::GetInstance().GetMapNode(nextTo);

        agv->AssignNextStep(fromNode, toNode, AGVState::MOVING, _e.timestamp);
    }
    // 2. 받은 경로의 끝에 도달했을 때
    else 
    {
        MissionPurpose purpose = agv->GetMissionPurpose();
        auto toNode = MapManager::GetInstance().GetMapNode(agv->GetToNodeID());

        // WHCA* 윈도우의 특징: 목적지가 아닌데 경로가 끝났다면 재탐색!
        if (agv->GetToNodeID() != plan.finalTargetNodeID)
        {
            uint32_t finalTarget = plan.finalTargetNodeID;
            m_MasterPlans.erase(agvID); // 일단 헌 계획표 찢음

            std::cout << "[관제탑] AGV " << agvID << "번 윈도우 끝 도달. 진짜 목적지(" << finalTarget << ")를 향해 이어서 탐색!" << std::endl;
            
            // 멈추지 않고 이어서 남은 구간에 대한 새로운 윈도우 경로를 발급!
            CreateRoute(agvID, finalTarget, _e.timestamp, purpose);
            return; 
        }
        if (purpose == MissionPurpose::PICKUP) 
            agv->AssignNextStep(toNode, toNode, AGVState::LOADING, _e.timestamp);
        else if (purpose == MissionPurpose::DROP) 
            agv->AssignNextStep(toNode, toNode, AGVState::UNLOADING, _e.timestamp);
        else if (purpose == MissionPurpose::HOME) 
            agv->AssignNextStep(toNode, toNode, AGVState::IDLE, _e.timestamp);
        
        m_MasterPlans.erase(agvID); 
    }
}