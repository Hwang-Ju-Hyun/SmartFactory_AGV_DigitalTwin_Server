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

    if (curNodeID == _targetNodeID)
    {
        TrafficManager::GetInstance().ClearFutureReservations(_agvID, _serverTime);
        TrafficManager::GetInstance().ReserveNode(curNodeID, _serverTime, _serverTime + WINDOW_TIME + 2.0f, _agvID);
        
        agv->SetMissionPurpose(_purpose);
        if (_purpose == MissionPurpose::HOME) 
            agv->AssignNextStep(MapManager::GetInstance().GetMapNode(curNodeID), MapManager::GetInstance().GetMapNode(curNodeID), AGVState::IDLE, _serverTime);
        else if (_purpose == MissionPurpose::PICKUP) 
            agv->AssignNextStep(MapManager::GetInstance().GetMapNode(curNodeID), MapManager::GetInstance().GetMapNode(curNodeID), AGVState::LOADING, _serverTime);
        else if (_purpose == MissionPurpose::DROP) 
            agv->AssignNextStep(MapManager::GetInstance().GetMapNode(curNodeID), MapManager::GetInstance().GetMapNode(curNodeID), AGVState::UNLOADING, _serverTime);
        return;
    }

    if (m_RRAEngines.find(_targetNodeID) == m_RRAEngines.end())
    {        
        m_RRAEngines[_targetNodeID] = RRAStar();
        m_RRAEngines[_targetNodeID].Init(_targetNodeID); 
    }

    PathFinder pf;
    std::vector<PathStep> path = pf.FindPath(curNodeID, _targetNodeID, _agvID, _serverTime, WINDOW_TIME, m_RRAEngines[_targetNodeID]);
    
    if (path.size() < 2)
    {
        m_MasterPlans.erase(_agvID); 
        std::cout << "[관제탑] AGV " << _agvID << "번 시공간 체증 감지! 대기 후 재탐색." << std::endl;
                        
        TrafficManager::GetInstance().ClearFutureReservations(_agvID, _serverTime);
        bool reserved = TrafficManager::GetInstance().ReserveNode(curNodeID, _serverTime, _serverTime + 1.0f, _agvID);
        if (!reserved) 
            std::cout << "AGV " << _agvID << "번 길막힘 패널티 예약 거부됨 (이미 타 AGV가 점유 중)" << std::endl;

        agv->ChangeState(AGVState::WAIT_REPLAN);
        m_PendingRoutes.push_back({ _agvID, _targetNodeID, _purpose, 1.0f });
        return; 
    }

    TrafficManager::GetInstance().ClearFutureReservations(_agvID, _serverTime);
    ReserveRouteTimeline(_agvID, path, _serverTime, _targetNodeID);

    RoutePlan plan;
    plan.agvID = _agvID;
    plan.currentStepIndex = 1; // 0번은 현재 발밑, 1번부터 이동 타겟!
    plan.purpose = _purpose;
    plan.finalTargetNodeID = _targetNodeID;
    plan.steps = path; 
    m_MasterPlans[_agvID] = plan;

    std::cout << "AGV " << _agvID << " 경로 확정: ";
    for (size_t i = 0; i < path.size(); ++i) 
    {
        std::cout << path[i].nodeID;
        if (i < path.size() - 1) 
            std::cout << " -> ";
    }
    std::cout << std::endl;

    auto fromNode = MapManager::GetInstance().GetMapNode(path[0].nodeID);
    auto toNode = MapManager::GetInstance().GetMapNode(path[1].nodeID);
    
    agv->SetMissionPurpose(_purpose);
        
    agv->AssignNextStep(fromNode, toNode, AGVState::MOVING, path[0].departureTime);        
}

void RoutePlanner::ReserveRouteTimeline(uint32_t _agvID, const std::vector<PathStep>& _path, float _serverTime, uint32_t _finalTargetID)
{   
    if (_path.empty()) return;
    const float TIME_MARGIN = CLEARANCE_TIME; 

    for (size_t i = 0; i < _path.size(); i++)
    {
        const PathStep& currentStep = _path[i];        
        TrafficManager::GetInstance().ReserveNode(currentStep.nodeID, currentStep.arrivalTime, currentStep.departureTime + TIME_MARGIN, _agvID);

        if (i + 1 < _path.size())
        {
            const PathStep& nextStep = _path[i + 1];        
            TrafficManager::GetInstance().ReserveLink(currentStep.nodeID, nextStep.nodeID, 
                                                      currentStep.departureTime, nextStep.arrivalTime + TIME_MARGIN, _agvID);
        }
        else 
        {
            if (currentStep.nodeID == _finalTargetID)
            {
                TrafficManager::GetInstance().ReserveNode(currentStep.nodeID, currentStep.arrivalTime, 
                                                          currentStep.arrivalTime + WINDOW_TIME + 2.0f, _agvID);
            }
        }
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
        const PathStep& currentStep = plan.steps[plan.currentStepIndex - 1];
        const PathStep& nextStep    = plan.steps[plan.currentStepIndex];

        auto fromNode = MapManager::GetInstance().GetMapNode(currentStep.nodeID);
        auto toNode   = MapManager::GetInstance().GetMapNode(nextStep.nodeID);

        float expectedArrivalTime = nextStep.arrivalTime;

        bool isNodeSafe = TrafficManager::GetInstance().IsNodeAvailable(nextStep.nodeID, expectedArrivalTime, expectedArrivalTime + 1.0f, agvID);
        bool isLinkSafe = TrafficManager::GetInstance().IsLinkAvailable(currentStep.nodeID, nextStep.nodeID, currentStep.departureTime, expectedArrivalTime, agvID);
        MapLink currentLink = MapManager::GetInstance().FindLink(currentStep.nodeID, nextStep.nodeID);

        if (!isNodeSafe || !isLinkSafe || currentLink.m_IsBlocked)
        {
            std::cout << "AGV " << agvID << "번 돌발 감지! 즉시 재탐색" << std::endl;
            uint32_t finalTarget = plan.finalTargetNodeID;
            MissionPurpose purpose = plan.purpose;
            
            m_MasterPlans.erase(agvID); 
            CreateRoute(agvID, finalTarget, _e.timestamp, purpose); 
            return;
        }        
        
        agv->AssignNextStep(fromNode, toNode, AGVState::MOVING, currentStep.departureTime);
    }
    else 
    {
        MissionPurpose purpose = agv->GetMissionPurpose();
        auto toNode = MapManager::GetInstance().GetMapNode(agv->GetCurrentNodeID());

        if (agv->GetToNodeID() != plan.finalTargetNodeID)
        {
            uint32_t finalTarget = plan.finalTargetNodeID;
            m_MasterPlans.erase(agvID); 
            CreateRoute(agvID, finalTarget, _e.timestamp, purpose);
            return;
        }
        
        if (purpose == MissionPurpose::PICKUP) agv->AssignNextStep(toNode, toNode, AGVState::LOADING, _e.timestamp);
        else if (purpose == MissionPurpose::DROP) agv->AssignNextStep(toNode, toNode, AGVState::UNLOADING, _e.timestamp);
        else if (purpose == MissionPurpose::HOME) agv->AssignNextStep(toNode, toNode, AGVState::IDLE, _e.timestamp);
        
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