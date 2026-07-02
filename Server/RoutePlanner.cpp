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
const float LONG_TERM_HORIZON = WINDOW_TIME * 3.0f; 

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

    // 1단계: 검증
    if (initialWaitTime > 0.01f)
    {
        if (!resTable.IsNodeFree(_path[0].nodeID, _serverTime, _path[0].arrivalTime + TIME_MARGIN, _agvID))
            return false;
    }

    for (size_t i = 0; i < _path.size(); i++)
    {
        const PathStep& cur = _path[i];                
        bool isLastNode = (i == _path.size() - 1);
        
        //9999 대신 LONG_TERM_HORIZON 적용
        float nodeLeaveTime = isLastNode ? cur.arrivalTime + LONG_TERM_HORIZON : cur.departureTime + TIME_MARGIN;
        
        if (!resTable.IsNodeFree(cur.nodeID, cur.arrivalTime, nodeLeaveTime, _agvID))
            return false; 

        if (!isLastNode)
        {
            const PathStep& next = _path[i + 1];        
            if (!resTable.IsEdgeFree(cur.nodeID, next.nodeID, cur.departureTime, next.arrivalTime + TIME_MARGIN, _agvID)) 
                return false; 
        }
    }

    // 🌟 2단계: 기록 (안전 마진을 주어 기존 예약 덮어쓰기)
    resTable.OverrideFutureReservations(_agvID, _serverTime, CLEARANCE_TIME);

    if (initialWaitTime > 0.01f)
    {
        resTable.ReserveNode(_path[0].nodeID, _serverTime, _path[0].arrivalTime + TIME_MARGIN, _agvID, ReservationType::Normal);
    }

    for (size_t i = 0; i < _path.size(); i++)
    {
        const PathStep& cur = _path[i];        
        bool isLastNode = (i == _path.size() - 1);
        
        float nodeLeaveTime = isLastNode ? cur.arrivalTime + LONG_TERM_HORIZON : cur.departureTime + TIME_MARGIN;
        ReservationType nodeType = isLastNode ? ReservationType::Goal : ReservationType::Normal;
        
        resTable.ReserveNode(cur.nodeID, cur.arrivalTime, nodeLeaveTime, _agvID, nodeType);

        if (!isLastNode)
        {
            const PathStep& next = _path[i + 1];        
            resTable.ReserveEdge(cur.nodeID, next.nodeID, cur.departureTime, next.arrivalTime + TIME_MARGIN, _agvID, ReservationType::Normal);
        }
    }
    return true;
}

void RoutePlanner::CreateRoute(uint32_t _agvID, uint32_t _targetNodeID, float _serverTime, MissionPurpose _purpose)
{
    Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(_agvID));
    if (!agv) return;     

    m_PendingRoutes.erase(
        std::remove_if(m_PendingRoutes.begin(), m_PendingRoutes.end(),
            [_agvID](const PendingRoute& r) { return r.agvID == _agvID; }),
        m_PendingRoutes.end()
    );

    uint32_t curNodeID = agv->GetCurrentNodeID();

    // 목적지 도달 시
    if (curNodeID == _targetNodeID)
    {
        ReservationTable::GetInstance().OverrideFutureReservations(_agvID, _serverTime, CLEARANCE_TIME);
        ReservationTable::GetInstance().ReserveNode(curNodeID, _serverTime, _serverTime + LONG_TERM_HORIZON, _agvID, ReservationType::Goal);
        
        agv->SetMissionPurpose(_purpose);
        // ... (AssignNextStep은 기존 유지) ...
        return;
    }

    if (m_RRAEngines.find(_targetNodeID) == m_RRAEngines.end())
    {        
        m_RRAEngines[_targetNodeID] = RRAStar();
        m_RRAEngines[_targetNodeID].Init(_targetNodeID); 
    }

    PathFinder pf;
    std::vector<PathStep> path = pf.FindPath(curNodeID, _targetNodeID, _agvID, _serverTime, WINDOW_TIME, m_RRAEngines[_targetNodeID]);
    
    // 🌟 [핵심 변경: 탐색 실패 시]
    if (path.size() < 2 || !TryReservePathTransaction(_agvID, path, _targetNodeID, _serverTime))
    {
        std::cout << "[관제탑] AGV " << _agvID << "번 경로 확보 실패! 현 위치 점유 연장 후 재탐색 대기." << std::endl;
                        
        // 실패했더라도 절대 장부를 비우지 않습니다. 현재 위치에 안전 버퍼(REPLAN_PENALTY + 1.0f)만큼 Normal 예약을 연장합니다.
        ReservationTable::GetInstance().ReserveNode(curNodeID, _serverTime, _serverTime + REPLAN_PENALTY_TIME + 1.0f, _agvID, ReservationType::Normal);

        agv->ChangeState(AGVState::IDLE); // WAIT_REPLAN 삭제!
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
        
        // 🌟 [방어막 복구] 내가 가려는 미래의 시간에 타 차량이 고립(Waiting)되어 알박기를 했는지 검증합니다.
        float expectedArrivalTime = nextStep.arrivalTime;
        bool isNodeSafe = ReservationTable::GetInstance().IsNodeFree(nextStep.nodeID, expectedArrivalTime, expectedArrivalTime + 0.5f, agvID);
        bool isLinkSafe = true;

        if (currentStep.nodeID != nextStep.nodeID)
        {
            isLinkSafe = ReservationTable::GetInstance().IsEdgeFree(currentStep.nodeID, nextStep.nodeID, currentStep.departureTime, expectedArrivalTime + 0.5f, agvID);
        }

        // 동적 링크 차단 검사
        bool isBlocked = false;
        if (currentStep.nodeID != nextStep.nodeID)
        {
            MapLink currentLink = MapManager::GetInstance().FindLink(currentStep.nodeID, nextStep.nodeID);
            isBlocked = currentLink.m_IsBlocked;
        }

        // 장부가 유효하지 않거나 오염되었다면 즉시 진입을 취소하고 제자리 재탐색을 수행합니다.
        if (!isNodeSafe || !isLinkSafe || isBlocked)
        {
            std::cout << "[동적 충돌 회피] AGV " << agvID << "번 진입 예정 노드/링크(" << nextStep.nodeID 
                      << ")에 타 차량 고립 감지! 즉시 계획을 폐기하고 재탐색합니다." << std::endl;
            
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