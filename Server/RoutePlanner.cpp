#include "RoutePlanner.hpp"
#include "AGVManager.hpp"
#include "PathFinder.hpp"
#include "TrafficControlManager.hpp" // 새로 만든 시공간 예약 장부!
#include "Map.hpp"
#include <iostream>

const int WINDOW_TIME = 16; // WHCA* 논문 권장 탐색 뎁스
const float AGV_SPEED = 4.0f;
const float WAIT_TIME = 1.0f;
const float CLEARANCE_TIME = 0.6f; // 앞차 꼬리가 빠져나가는 안전 시간

void RoutePlanner::Init()
{    
    EventManager::GetInstance().Subscribe(RobotEventType::MOVING_WAITING_COMPLETED,[this](const RobotEvent& _e) { OnRobotStepCompleted(_e); });
    
}

void RoutePlanner::CreateRoute(uint32_t _agvID, uint32_t _targetNodeID, float _serverTime, MissionPurpose _purpose)
{
    std::cout << "[CREATE ROUTE] AGV " << _agvID << " | Target: " << _targetNodeID << " | Time: " << _serverTime << std::endl;
    Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(_agvID));
    if (!agv) return;     

    m_PendingRoutes.erase(
            std::remove_if(m_PendingRoutes.begin(), m_PendingRoutes.end(),
                [_agvID](const PendingRoute& r) { return r.agvID == _agvID; }),
            m_PendingRoutes.end()
        );
    uint32_t curNodeID = agv->GetCurrentNodeID();

    // 1. 이미 도착한 경우 (제자리 알박기)
    if (curNodeID == _targetNodeID)
    {
        TrafficManager::GetInstance().ClearFutureReservations(_agvID, _serverTime);
        //TrafficManager::GetInstance().ReserveNode(curNodeID, _serverTime, _serverTime + 99999.0f, _agvID);
        TrafficManager::GetInstance().ReserveNode(curNodeID, _serverTime, _serverTime + WINDOW_TIME + 2.0f, _agvID);
        
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
    std::vector<uint32_t> path = pf.FindPath(curNodeID, _targetNodeID, _agvID, _serverTime, WINDOW_TIME, m_RRAEngines[_targetNodeID]);
    
    if (path.size() < 2)
    {
        m_MasterPlans.erase(_agvID); 
        std::cout << "[관제탑] AGV " << _agvID << "번 시공간 체증 감지! 대기 후 재탐색." << std::endl;
                        
        TrafficManager::GetInstance().ClearFutureReservations(_agvID, _serverTime);
        
        bool reserved = TrafficManager::GetInstance().ReserveNode(curNodeID, _serverTime, _serverTime + 1.0f, _agvID);
        if (!reserved)
        {
            std::cout << "AGV " << _agvID << "번 길막힘 패널티 예약 거부됨 (이미 타 AGV가 점유 중)" << std::endl;
        }

        agv->ChangeState(AGVState::WAIT_REPLAN);

        // [핵심 방어] 이미 재시도 큐에 이 AGV가 있는지 확인 (중복 폭주 방지)
        bool alreadyPending = false;
        for (const auto& pending : m_PendingRoutes)
        {
            if (pending.agvID == _agvID)
            {
                alreadyPending = true;
                break;
            }
        }

        if (!alreadyPending)
        {                                        
            m_PendingRoutes.push_back({ _agvID, _targetNodeID, _purpose, 1.0f });
        }        
        
        return; 
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

    {
        std::cout << "AGV " << _agvID << " 경로 확정: ";
    for (size_t i = 0; i < path.size(); ++i) {
        std::cout << path[i];
        if (i < path.size() - 1) std::cout << " -> ";
    }
    std::cout << std::endl;
    }
    

    // 첫 발걸음 내딛기!
    auto fromNode = MapManager::GetInstance().GetMapNode(plan.steps[0].fromNodeID);
    auto toNode = MapManager::GetInstance().GetMapNode(plan.steps[0].toNodeID);
    
    agv->SetMissionPurpose(_purpose);
    agv->AssignNextStep(fromNode, toNode, AGVState::MOVING,_serverTime);
        
}

void RoutePlanner::ReserveRouteTimeline(uint32_t _agvID, const std::vector<uint32_t>& _path, float _serverTime, uint32_t _finalTargetID)
{
   
    float accTime = _serverTime;
    
    // 출발 발밑 예약
    TrafficManager::GetInstance().ReserveNode(_path[0], accTime, accTime + SLOT_DURATION, _agvID);

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
        float travelTime = std::ceil(dist / AGV_SPEED);
       
        float linkEnterTime = accTime;               
        float linkLeaveTime = accTime + travelTime;  
        
        TrafficManager::GetInstance().ReserveLink(fromID, toID, linkEnterTime, linkLeaveTime, _agvID);
        TrafficManager::GetInstance().ReserveNode(fromID, linkEnterTime, linkLeaveTime, _agvID);

        float nodeEnterTime = linkLeaveTime;

        if (i + 1 == _path.size() - 1) 
        {         
            TrafficManager::GetInstance().ReserveNode(toID, nodeEnterTime, nodeEnterTime + WINDOW_TIME + 2.0f, _agvID);
        }    
        else 
        {                                
            TrafficManager::GetInstance().ReserveNode(toID, nodeEnterTime, nodeEnterTime + 1.0f, _agvID);                
        }       

        accTime = nodeEnterTime;
         std::cout
<< "RESERVE "
<< fromID
<< " -> "
<< toID
<< " enter "
<< linkEnterTime
<< " leave "
<< linkLeaveTime
<< std::endl;
    }
}

void RoutePlanner::OnRobotStepCompleted(const RobotEvent& _e)
{
    uint32_t agvID = _e.agvID;
    if (m_MasterPlans.find(agvID) == m_MasterPlans.end()) return;
    
    RoutePlan& plan = m_MasterPlans[agvID];
    Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(agvID));

    plan.currentStepIndex++; 

    if (plan.currentStepIndex < plan.steps.size())
    {
        uint32_t nextFrom = plan.steps[plan.currentStepIndex].fromNodeID;
        uint32_t nextTo   = plan.steps[plan.currentStepIndex].toNodeID;

        auto fromNode = MapManager::GetInstance().GetMapNode(nextFrom);
        auto toNode   = MapManager::GetInstance().GetMapNode(nextTo);

        // =========================================================
        // [핵심 1] 출발 직전 재검사 (미래 도착 시간 검사로 완벽 수정!)
        // =========================================================
        MapLink currentLink = MapManager::GetInstance().FindLink(nextFrom, nextTo);
        float dist = 0.0f;
        if (currentLink.m_Type == 1) dist = currentLink.m_Dist;
        else dist = std::sqrt(std::pow(toNode.m_PosX - fromNode.m_PosX, 2) + std::pow(toNode.m_PosZ - fromNode.m_PosZ, 2));

        float travelTime = std::ceil(dist / AGV_SPEED);
        float arrivalTime = _e.timestamp + travelTime;  //내가 이 링크를 다 건너서 도착할 미래의 시간

        //노드 검사: "내가 도착할 시간(arrivalTime)에 이 노드가 비어있는가
        bool isNodeSafe = TrafficManager::GetInstance().IsNodeAvailable(nextTo, arrivalTime, arrivalTime + 1.0f, agvID);
        //링크 검사: "내가 진입하는 지금부터 빠져나가는 순간까지 이 길이 비어있는가
        bool isLinkSafe = TrafficManager::GetInstance().IsLinkAvailable(nextFrom, nextTo, _e.timestamp, arrivalTime, agvID);

        if (!isNodeSafe || !isLinkSafe || currentLink.m_IsBlocked)
        {
            uint32_t finalTarget = plan.finalTargetNodeID;
            MissionPurpose purpose = plan.purpose;
            
            m_MasterPlans.erase(agvID); // 이제 안전하게 지워도 됩니다.
            
            CreateRoute(agvID, finalTarget, _e.timestamp, purpose); // 대피시킨 안전한 값을 사용!
            return;
        }        

        agv->AssignNextStep(fromNode, toNode, AGVState::MOVING, _e.timestamp);
    }
    // 2. 받은 경로의 끝에 도달했을 때
    else 
    {
        MissionPurpose purpose = agv->GetMissionPurpose();
        auto toNode = MapManager::GetInstance().GetMapNode(agv->GetCurrentNodeID());

        // WHCA* 윈도우의 특징: 목적지가 아닌데 경로가 끝났다면 재탐색!
        if (agv->GetToNodeID() != plan.finalTargetNodeID)
        {
           uint32_t finalTarget = plan.finalTargetNodeID;
            
            m_MasterPlans.erase(agvID); 

            std::cout<<"RoutePlanner::OnRobotStepCompleted CreateRoute Call"<<std::endl;
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

void RoutePlanner::Update(float dt, float serverTime)
{
    std::vector<PendingRoute> retryList;

    for(auto it = m_PendingRoutes.begin();
        it != m_PendingRoutes.end();)
    {
        it->retryTimer -= dt;

        if(it->retryTimer <= 0.0f)
        {
            retryList.push_back(*it);
            it = m_PendingRoutes.erase(it);
        }
        else
        {
            ++it;
        }
    }

    for(auto& route : retryList)
    {
        std::cout<<"RoutePlanner::Update CreateRoute Call : retry"<<std::endl;
        CreateRoute(
            route.agvID,
            route.targetNodeID,
            serverTime,
            route.purpose
        );
    }
}