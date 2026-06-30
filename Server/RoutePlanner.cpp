#include "RoutePlanner.hpp"
#include "AGVManager.hpp"
#include "PathFinder.hpp"
#include "ReservationTable.hpp"
#include "Map.hpp"
#include <iostream>
#include <algorithm>

const int WINDOW_TIME = 16; 
const float CLEARANCE_TIME = 0.6f; 
const float REPLAN_PENALTY_TIME = 1.0f; 

void RoutePlanner::Init()
{    
    EventManager::GetInstance().Subscribe(RobotEventType::MOVING_WAITING_COMPLETED, [this](const RobotEvent& _e) { OnRobotStepCompleted(_e); });
}

bool RoutePlanner::TryReservePathTransaction(uint32_t _agvID, const std::vector<PathStep>& _path, uint32_t _finalTargetID, float _serverTime)
{
    if (_path.empty()) return false;
    
    const float TIME_MARGIN = CLEARANCE_TIME; 
    auto& resTable = ReservationTable::GetInstance();

    float initialWaitTime = _path[0].arrivalTime - _serverTime;

    // 1단계: 검증 (Validation)
    if (initialWaitTime > 0.01f)
    {
        if (!resTable.IsNodeFree(_path[0].nodeID, _serverTime, _path[0].arrivalTime + TIME_MARGIN, _agvID))
        {
             
            return false;
        } 
    }

    for (size_t i = 0; i < _path.size(); i++)
    {
        const PathStep& cur = _path[i];                
        float nodeLeaveTime = (cur.nodeID == _finalTargetID) ? cur.arrivalTime + WINDOW_TIME + 2.0f : cur.departureTime + TIME_MARGIN;
        
        if (!resTable.IsNodeFree(cur.nodeID, cur.arrivalTime, nodeLeaveTime, _agvID))
        {
                
        } 

        if (i + 1 < _path.size())
        {
            const PathStep& next = _path[i + 1];        
            if (!resTable.IsEdgeFree(cur.nodeID, next.nodeID, cur.departureTime, next.arrivalTime + TIME_MARGIN, _agvID)) 
            {
                std::cout
            << "EDGE FAIL "
            << cur.nodeID
            << "->"
            << next.nodeID
            << '\n';
                return false; 
            }
        }
    }

    // 2단계: 기록 (Commit) - Enum 타입을 꼼꼼하게 부여합니다.
    if (initialWaitTime > 0.01f)
    {
        resTable.ReserveNode(_path[0].nodeID, _serverTime, _path[0].arrivalTime + TIME_MARGIN, _agvID, ReservationType::Normal);
    }

    for (size_t i = 0; i < _path.size(); i++)
    {
        const PathStep& cur = _path[i];        
        bool isGoal = (cur.nodeID == _finalTargetID);
        
        float nodeLeaveTime = isGoal ? cur.arrivalTime + WINDOW_TIME + 2.0f : cur.departureTime + TIME_MARGIN;
        ReservationType nodeType = isGoal ? ReservationType::Goal : ReservationType::Normal;
        
        resTable.ReserveNode(cur.nodeID, cur.arrivalTime, nodeLeaveTime, _agvID, nodeType);

        if (i + 1 < _path.size())
        {
            const PathStep& next = _path[i + 1];        
            resTable.ReserveEdge(cur.nodeID, next.nodeID, cur.departureTime, next.arrivalTime + TIME_MARGIN, _agvID, ReservationType::Normal);
        }
    }
    return true;
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
        ReservationTable::GetInstance().ClearFutureReservations(_agvID, _serverTime);
        ReservationTable::GetInstance().ReserveNode(curNodeID, _serverTime, _serverTime + WINDOW_TIME + 2.0f, _agvID, ReservationType::Goal);
        
        agv->SetMissionPurpose(_purpose);
        if (_purpose == MissionPurpose::HOME) agv->AssignNextStep(MapManager::GetInstance().GetMapNode(curNodeID), MapManager::GetInstance().GetMapNode(curNodeID), AGVState::IDLE, _serverTime, _serverTime + 1.0f);
        else if (_purpose == MissionPurpose::PICKUP) agv->AssignNextStep(MapManager::GetInstance().GetMapNode(curNodeID), MapManager::GetInstance().GetMapNode(curNodeID), AGVState::LOADING, _serverTime, _serverTime + 1.0f);
        else if (_purpose == MissionPurpose::DROP) agv->AssignNextStep(MapManager::GetInstance().GetMapNode(curNodeID), MapManager::GetInstance().GetMapNode(curNodeID), AGVState::UNLOADING, _serverTime, _serverTime + 1.0f);
        return;
    }

    if (m_RRAEngines.find(_targetNodeID) == m_RRAEngines.end())
    {        
        m_RRAEngines[_targetNodeID] = RRAStar();
        m_RRAEngines[_targetNodeID].Init(_targetNodeID); 
    }

    PathFinder pf;
    std::vector<PathStep> path = pf.FindPath(curNodeID, _targetNodeID, _agvID, _serverTime, WINDOW_TIME, m_RRAEngines[_targetNodeID]);
    
    //  길막힘 패널티 9999초를 1초(REPLAN_PENALTY_TIME)로 축소하고 Waiting 타입 지정!
    if (path.size() < 2)
    {
        m_MasterPlans.erase(_agvID); 
        std::cout << "[관제탑] AGV " << _agvID << "번 시공간 체증 감지! " << REPLAN_PENALTY_TIME << "초 대기 후 재탐색." << std::endl;
                        
        ReservationTable::GetInstance().ClearFutureReservations(_agvID, _serverTime);
        ReservationTable::GetInstance().ReserveNode(curNodeID, _serverTime, _serverTime + REPLAN_PENALTY_TIME, _agvID, ReservationType::Waiting);

        agv->ChangeState(AGVState::WAIT_REPLAN);
        m_PendingRoutes.push_back({ _agvID, _targetNodeID, _purpose, REPLAN_PENALTY_TIME });
        return; 
    }
    
    ReservationTable::GetInstance().ClearFutureReservations(_agvID, _serverTime);

    // 트랜잭션 실패 시에도 1초(REPLAN_PENALTY_TIME) 패널티 지정!
    if (!TryReservePathTransaction(_agvID, path, _targetNodeID, _serverTime))
    {
        std::cout << " [트랜잭션 거부] AGV " << _agvID << " 경로 예약 중 타 차량 선점 발견. 재탐색 대기." << std::endl;
        
        ReservationTable::GetInstance().ReserveNode(curNodeID, _serverTime, _serverTime + REPLAN_PENALTY_TIME, _agvID, ReservationType::Waiting);
        agv->ChangeState(AGVState::WAIT_REPLAN);
        m_PendingRoutes.push_back({ _agvID, _targetNodeID, _purpose, REPLAN_PENALTY_TIME });
        return;
    }

    RoutePlan plan;
    plan.agvID = _agvID;
    plan.currentStepIndex = 1; 
    plan.purpose = _purpose;
    plan.finalTargetNodeID = _targetNodeID;
    plan.steps = path; 
    m_MasterPlans[_agvID] = plan;

    std::cout << "AGV " << _agvID << " 경로 확정: ";
    for (size_t i = 0; i < path.size(); ++i) {
        std::cout << path[i].nodeID;
        if (i < path.size() - 1) std::cout << " -> ";
    }
    std::cout << std::endl;

    auto fromNode = MapManager::GetInstance().GetMapNode(path[0].nodeID);
    auto toNode = MapManager::GetInstance().GetMapNode(path[1].nodeID);
    
    agv->SetMissionPurpose(_purpose);
    agv->AssignNextStep(fromNode, toNode, AGVState::MOVING, path[0].departureTime, path[1].arrivalTime);        
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
        
        
        
        bool isBlocked = false;

        if (currentStep.nodeID != nextStep.nodeID)
        {
            MapLink currentLink = MapManager::GetInstance().FindLink(currentStep.nodeID, nextStep.nodeID);
            isBlocked = currentLink.m_IsBlocked;
        }

        if (isBlocked)
        {
            std::cout << "AGV " << agvID << "번 동적 장애물 감지! 즉시 우회 재탐색" << std::endl;
            uint32_t finalTarget = plan.finalTargetNodeID;
            MissionPurpose purpose = plan.purpose;
            
            m_MasterPlans.erase(agvID); 
            CreateRoute(agvID, finalTarget, _e.timestamp, purpose); 
            return;
        }        
                
        agv->AssignNextStep(fromNode, toNode, AGVState::MOVING, currentStep.departureTime, nextStep.arrivalTime);
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
        
        if (purpose == MissionPurpose::PICKUP) agv->AssignNextStep(toNode, toNode, AGVState::LOADING, _e.timestamp, _e.timestamp + 1.0f);
        else if (purpose == MissionPurpose::DROP) agv->AssignNextStep(toNode, toNode, AGVState::UNLOADING, _e.timestamp, _e.timestamp + 1.0f);
        else if (purpose == MissionPurpose::HOME) agv->AssignNextStep(toNode, toNode, AGVState::IDLE, _e.timestamp, _e.timestamp + 1.0f);
        
        m_MasterPlans.erase(agvID); 
    }
}

void RoutePlanner::Update(float dt, float serverTime)
{
    std::vector<PendingRoute> retryList;

    for(auto it = m_PendingRoutes.begin(); it != m_PendingRoutes.end();)
    {
        it->retryTimer -= dt;

        if(it->retryTimer <= 0.0f)
        {
            retryList.push_back(*it);
            it = m_PendingRoutes.erase(it);
        }
        else ++it;
    }

    for(auto& route : retryList)
    {
        CreateRoute(route.agvID, route.targetNodeID, serverTime, route.purpose);
    }
}