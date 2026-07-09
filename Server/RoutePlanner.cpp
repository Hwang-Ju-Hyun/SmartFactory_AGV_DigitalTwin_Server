#include "RoutePlanner.hpp"
#include "AGVManager.hpp"
#include "PathFinder.hpp"
#include "ReservationTable.hpp"
#include "Map.hpp"
#include <iostream>
#include <algorithm>
#include "SharedPackets.hpp"
#include "IRobotController.hpp"
#include "RobotManager.hpp"

const int WINDOW_TIME = 16; 
const float CLEARANCE_TIME = 0.6f; 
const float REPLAN_PENALTY_TIME = 1.0f; 
const float LONG_TERM_HORIZON = WINDOW_TIME * 3.0f; 

void RoutePlanner::Init()
{    
    EventManager::GetInstance().Subscribe(RobotEventType::NODE_ARRIVED, [this](const RobotEvent& _e) { OnRobotStepCompleted(_e); });
}

bool RoutePlanner::TryReservePathTransaction(uint32_t _agvID, const std::vector<PathStep>& _path, uint32_t _finalTargetID, float _serverTime)
{
    if (_path.empty())
    {        
        return false;
    } 
    
    const float TIME_MARGIN = CLEARANCE_TIME; 
    auto& resTable = ReservationTable::GetInstance();

    float initialWaitTime = _path[0].arrivalTime - _serverTime;

    // 1단계: 검증 루프
    if (initialWaitTime > 0.01f) {
        if (!resTable.IsNodeFree(_path[0].nodeID, _serverTime, _path[0].arrivalTime + TIME_MARGIN, _agvID))
            return false;
    }

    for (size_t i = 0; i < _path.size(); i++)
    {
        const PathStep& cur = _path[i];                
        bool isLastNodeInArray = (i == _path.size() - 1);
        
        // 현재 노드가 진짜 최종 목적지인지 확인
        bool reachedGoal = (cur.nodeID == _finalTargetID); 
        
        // 진짜 목적지일 때만 LONG_TERM_HORIZON(무한 점유)을 적용
        float nodeLeaveTime = reachedGoal ? cur.arrivalTime + LONG_TERM_HORIZON : cur.departureTime + TIME_MARGIN;
        
        if (!resTable.IsNodeFree(cur.nodeID, cur.arrivalTime, nodeLeaveTime, _agvID))
            return false; 

        if (!isLastNodeInArray)
        {
            const PathStep& next = _path[i + 1];        
            if (!resTable.IsEdgeFree(cur.nodeID, next.nodeID, cur.departureTime, next.arrivalTime + TIME_MARGIN, _agvID)) 
                return false; 
        }
    }

    // 2단계: 기록 루프
    resTable.OverrideFutureReservations(_agvID, _serverTime, CLEARANCE_TIME);

    if (initialWaitTime > 0.01f) 
    {
        resTable.ReserveNode(_path[0].nodeID, _serverTime, _path[0].arrivalTime + TIME_MARGIN, _agvID, ReservationType::Normal);
    }

    for (size_t i = 0; i < _path.size(); i++)
    {
        const PathStep& cur = _path[i];        
        bool isLastNodeInArray = (i == _path.size() - 1);
                
        bool reachedGoal = (cur.nodeID == _finalTargetID);
        
        float nodeLeaveTime = reachedGoal ? cur.arrivalTime + LONG_TERM_HORIZON : cur.departureTime + TIME_MARGIN;
        ReservationType nodeType = reachedGoal ? ReservationType::Goal : ReservationType::Normal;
        
        resTable.ReserveNode(cur.nodeID, cur.arrivalTime, nodeLeaveTime, _agvID, nodeType);

        if (!isLastNodeInArray)
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

    m_PendingRoutes.erase(std::remove_if(m_PendingRoutes.begin(), m_PendingRoutes.end(),
        [_agvID](const PendingRoute& r) { return r.agvID == _agvID; }), m_PendingRoutes.end());

    uint32_t curNodeID = agv->GetCurrentNodeID();

    if (curNodeID == _targetNodeID) {
        ReservationTable::GetInstance().OverrideFutureReservations(_agvID, _serverTime, CLEARANCE_TIME);
        ReservationTable::GetInstance().ReserveNode(curNodeID, _serverTime, _serverTime + LONG_TERM_HORIZON, _agvID, ReservationType::Goal);
        agv->SetMissionPurpose(_purpose);
        return;
    }

    std::vector<PathStep> path;
    if (TryFindPath(_agvID, _targetNodeID, _serverTime, path)) {
        if (TryReservePathTransaction(_agvID, path, _targetNodeID, _serverTime)) {
            HandlePathFound(_agvID, _targetNodeID, _purpose, path);
        } else {
            HandlePathFailed(_agvID, _targetNodeID, _serverTime, _purpose);
        }
    } else {
        HandlePathFailed(_agvID, _targetNodeID, _serverTime, _purpose);
    }
}

bool RoutePlanner::TryFindPath(uint32_t _agvID, uint32_t _targetNodeID, float _serverTime, std::vector<PathStep>& outPath) 
{
    uint32_t curNodeID = AGVManager::GetInstance().FindAGV(_agvID)->GetCurrentNodeID();
    if (m_RRAEngines.find(_targetNodeID) == m_RRAEngines.end()) 
        m_RRAEngines[_targetNodeID].Init(_targetNodeID); 
    
    PathFinder pf;
    outPath = pf.FindPath(curNodeID, _targetNodeID, _agvID, _serverTime, WINDOW_TIME, m_RRAEngines[_targetNodeID]);
    return (!outPath.empty() && outPath.size() >= 2);
}

void RoutePlanner::HandlePathFound(uint32_t _agvID, uint32_t _targetNodeID, MissionPurpose _purpose, const std::vector<PathStep>& path) {
    RoutePlan plan;
    plan.agvID = _agvID; plan.currentStepIndex = 1; plan.purpose = _purpose;
    plan.finalTargetNodeID = _targetNodeID; plan.steps = path; 
    m_MasterPlans[_agvID] = plan;

    Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(_agvID));
    agv->SetMissionPurpose(_purpose);
    agv->ChangeState(AGVState::MOVING); 

    std::vector<RouteNodeTime> routeNodes;
    for (const auto& step : path) {
        routeNodes.push_back({step.nodeID, step.arrivalTime, step.departureTime});
    }

    IRobotController* controller = RobotManager::GetInstance().GetRobotController(_agvID);
    if (controller)
    {
        controller->FollowRoute({_agvID, routeNodes});
    } 
}

void RoutePlanner::HandlePathFailed(uint32_t _agvID, uint32_t _targetNodeID, float _serverTime, MissionPurpose _purpose) {
    Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(_agvID));
    uint32_t curNodeID = agv->GetCurrentNodeID();

    ReservationTable::GetInstance().ReserveNode(curNodeID, _serverTime, _serverTime + REPLAN_PENALTY_TIME + 1.0f, _agvID, ReservationType::Normal);
    agv->ChangeState(AGVState::IDLE);
    m_PendingRoutes.push_back({ _agvID, _targetNodeID, _purpose, REPLAN_PENALTY_TIME });
}

void RoutePlanner::OnRobotStepCompleted(const RobotEvent& _e)
{
    uint32_t agvID = _e.agvID;
    Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(agvID));
    if (!agv) return;

    if (m_MasterPlans.find(agvID) == m_MasterPlans.end()) return;
    RoutePlan& plan = m_MasterPlans[agvID]; 

    UpdateRobotPosition(agv, plan, _e);
    
    // 🌟 [수정 완료] 멘토님 조언 반영: 불필요한 이중 장부 관리(AssignNextStep)를 생략합니다.
    // 경로의 진행 상태는 오직 컨트롤러가 단일 권한(Single Source of Truth)을 갖고 이동을 전담합니다.
    if (ContinueCurrentRoute(agv, plan)) return;
    
    FinishRoute(agv, plan, _e);
}

void RoutePlanner::UpdateRobotPosition(Robo* agv, RoutePlan& plan, const RobotEvent& _e) {
    agv->SetCurrentNodeID(_e.currentNodeID);
    // 🌟 쪼개진 모듈로 갱신 요청
    OccupancyProvider::GetInstance().OccupyNode(_e.agvID, _e.currentNodeID);
    plan.currentStepIndex++; 
}

bool RoutePlanner::ContinueCurrentRoute(Robo* agv, RoutePlan& plan) {
    // 이동 뼈대 유지 및 인덱스 상태 바운더리 체크만 안전하게 수행
    if (plan.currentStepIndex >= plan.steps.size()) return false;
    return true; 
}

void RoutePlanner::FinishRoute(Robo* agv, RoutePlan& plan, const RobotEvent& _e) {
    uint32_t agvID = _e.agvID;
    if (_e.currentNodeID == plan.finalTargetNodeID) {
        MissionPurpose purpose = agv->GetMissionPurpose();
        agv->StartWorkTimer(1.0f); 
        if (purpose == MissionPurpose::PICKUP) agv->ChangeState(AGVState::LOADING);
        else if (purpose == MissionPurpose::DROP) agv->ChangeState(AGVState::UNLOADING);
        else if (purpose == MissionPurpose::HOME) {
            agv->ChangeState(AGVState::IDLE); agv->StartWorkTimer(0.0f); 
            EventManager::GetInstance().Publish({ RobotEventType::IDLE_READY, agvID, _e.timestamp });
        }
        m_MasterPlans.erase(agvID); 
    } else {
        uint32_t finalTarget = plan.finalTargetNodeID;
        MissionPurpose purpose = plan.purpose;
        float nextStartTime = _e.timestamp; 
        if (!plan.steps.empty()) nextStartTime = std::max(_e.timestamp, plan.steps.back().departureTime);
        m_MasterPlans.erase(agvID); 
        CreateRoute(agvID, finalTarget, nextStartTime, purpose); 
    }
}