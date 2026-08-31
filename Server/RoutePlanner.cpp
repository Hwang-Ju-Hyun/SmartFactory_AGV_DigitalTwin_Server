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
#include "OccupancyProvider.hpp"

const int WINDOW_TIME = 16; 
const float CLEARANCE_TIME = 0.6f; 
const float REPLAN_PENALTY_TIME = 1.0f; 
const float LONG_TERM_HORIZON = WINDOW_TIME * 3.0f; 

namespace
{
    bool RouteMatchesExpectedNodeIDs(const std::vector<RouteNodeTime>& _routeNodes,
                                     const std::vector<uint32_t>& _expectedNodeIDs)
    {
        return _routeNodes.size() == _expectedNodeIDs.size() &&
            std::equal(_routeNodes.begin(), _routeNodes.end(), _expectedNodeIDs.begin(),
                [](const RouteNodeTime& routeNode, uint32_t expectedNodeID)
                {
                    return routeNode.nodeID == expectedNodeID;
                });
    }
}

void RoutePlanner::Init()
{    
    EventManager::GetInstance().Subscribe(RobotEventType::NODE_ARRIVED, [this](const RobotEvent& _e) { OnRobotStepCompleted(_e); });
}

void RoutePlanner::Update(float _deltaTime, float _serverTime)
{
    const size_t pendingCount = m_PendingRoutes.size();
    for (size_t i = 0; i < pendingCount; ++i)
    {
        PendingRoute pending = m_PendingRoutes.front();
        m_PendingRoutes.pop_front();

        pending.retryTimer -= _deltaTime;
        if (pending.retryTimer > 0.0f)
        {
            m_PendingRoutes.push_back(pending);
            continue;
        }

        CreateRoute(pending.agvID, pending.targetNodeID, _serverTime, pending.purpose);
    }
}

bool RoutePlanner::ResendCurrentRouteToController(uint32_t _agvID,
                                                  const std::vector<uint32_t>& _expectedNodeIDs)
{
    auto planIt = m_MasterPlans.find(_agvID);
    if (planIt == m_MasterPlans.end())
        return false;

    const RoutePlan& plan = planIt->second;
    if (plan.steps.size() < 2)
        return false;

    size_t startIndex = 0;
    if (plan.currentStepIndex > 0)
        startIndex = plan.currentStepIndex - 1;

    if (startIndex >= plan.steps.size())
        startIndex = plan.steps.size() - 1;

    std::vector<RouteNodeTime> routeNodes;
    routeNodes.reserve(plan.steps.size() - startIndex);
    for (size_t i = startIndex; i < plan.steps.size(); ++i)
    {
        const PathStep& step = plan.steps[i];
        routeNodes.push_back({ step.nodeID, step.arrivalTime, step.departureTime });
    }

    if (routeNodes.size() < 2)
        return false;

    if (!_expectedNodeIDs.empty() && !RouteMatchesExpectedNodeIDs(routeNodes, _expectedNodeIDs))
    {
        std::cout << "[RoutePlanner] Active route not resent for AGV " << _agvID
                  << ": node sequence does not match the required route\n";
        return false;
    }

    IRobotController* controller = RobotManager::GetInstance().GetRobotController(_agvID);
    if (!controller)
        return false;

    if (!controller->FollowRoute({ _agvID, routeNodes }))
    {
        std::cout << "[RoutePlanner] Active route resend rejected by controller. AGV "
                  << _agvID << "\n";
        return false;
    }
    std::cout << "[RoutePlanner] Resent active route to controller. AGV " << _agvID
              << " nodes=" << routeNodes.size() << "\n";
    return true;
}

void RoutePlanner::OnExecutionBlocked(uint32_t _agvID, uint32_t _currentNodeID, uint32_t _blockedNodeID, float _serverTime)
{
    Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(_agvID));
    if (!agv) return;

    auto planIt = m_MasterPlans.find(_agvID);
    if (planIt == m_MasterPlans.end())
    {
        if (IRobotController* controller = RobotManager::GetInstance().GetRobotController(_agvID))
            controller->CancelRoute();
        agv->SetCurrentNodeID(_currentNodeID);
        agv->ChangeState(AGVState::IDLE);
        return;
    }

    if (planIt->second.strictNodeSequence)
    {
        StopRouteWithoutReplan(_agvID, agv, _serverTime, "execution blocked");
        return;
    }

    const uint32_t targetNodeID = planIt->second.finalTargetNodeID;
    const MissionPurpose purpose = planIt->second.purpose;

    if (IRobotController* controller = RobotManager::GetInstance().GetRobotController(_agvID))
        controller->CancelRoute();

    m_MasterPlans.erase(planIt);
    m_PendingRoutes.erase(std::remove_if(m_PendingRoutes.begin(), m_PendingRoutes.end(),
        [_agvID](const PendingRoute& r) { return r.agvID == _agvID; }), m_PendingRoutes.end());

    agv->SetCurrentNodeID(_currentNodeID);
    agv->ChangeState(AGVState::WAIT_REPLAN);

    ReservationTable::GetInstance().OverrideFutureReservations(_agvID, _serverTime, 0.0f);
    ReservationTable::GetInstance().ReserveNode(_currentNodeID, _serverTime, _serverTime + REPLAN_PENALTY_TIME + CLEARANCE_TIME, _agvID, ReservationType::Normal);
    if (_blockedNodeID != 0)
    {
        constexpr uint32_t TEMP_BLOCK_AGV_ID = 0;
        const float blockEndTime = _serverTime + REPLAN_PENALTY_TIME;
        ReservationTable::GetInstance().ReserveNode(_blockedNodeID, _serverTime, blockEndTime, TEMP_BLOCK_AGV_ID, ReservationType::Normal);
        ReservationTable::GetInstance().ReserveEdge(_currentNodeID, _blockedNodeID, _serverTime, blockEndTime, TEMP_BLOCK_AGV_ID, ReservationType::Normal);
    }

    m_PendingRoutes.push_back({ _agvID, targetNodeID, purpose, 0.1f });
    // std::cout << "[REPLAN] AGV " << _agvID << " execution blocked at node " << _currentNodeID
    //           << " toward " << _blockedNodeID << ", route cancelled\n";
}

void RoutePlanner::StopActiveRouteForSafety(uint32_t _agvID,
                                            float _serverTime,
                                            const char* _reason)
{
    Robo* agv = dynamic_cast<Robo*>(
        AGVManager::GetInstance().FindAGV(_agvID));
    if (!agv)
        return;
    StopRouteWithoutReplan(_agvID, agv, _serverTime, _reason);
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

    if (_path.size() >= 2 && _path[0].departureTime <= _serverTime + 0.05f)
    {
        const uint32_t firstNodeID = _path[0].nodeID;
        const uint32_t nextNodeID = _path[1].nodeID;
        const uint64_t firstEdgeKey = MakeEdgeKey(firstNodeID, nextNodeID);
        auto& occ = OccupancyProvider::GetInstance();

        if (occ.IsNodeOccupiedByOther(nextNodeID, _agvID)) return false;
        if (occ.IsEdgeOccupiedByOther(firstEdgeKey, _agvID)) return false;
    }

    // 1단계: 검증 루프
    if (initialWaitTime > 0.01f) 
    {
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
    if (!agv) 
        return;

    m_PendingRoutes.erase(std::remove_if(m_PendingRoutes.begin(), m_PendingRoutes.end(),
        [_agvID](const PendingRoute& r) { return r.agvID == _agvID; }), m_PendingRoutes.end());

    uint32_t curNodeID = agv->GetCurrentNodeID();

    if (curNodeID == _targetNodeID) 
    {
        ReservationTable::GetInstance().OverrideFutureReservations(_agvID, _serverTime, CLEARANCE_TIME);
        ReservationTable::GetInstance().ReserveNode(curNodeID, _serverTime, _serverTime + LONG_TERM_HORIZON, _agvID, ReservationType::Goal);
        agv->SetMissionPurpose(_purpose);
        return;
    }

    std::vector<PathStep> path;
    if (TryFindPath(_agvID, _targetNodeID, _serverTime, path)) 
    {
        if (TryReservePathTransaction(_agvID, path, _targetNodeID, _serverTime)) 
        {
            HandlePathFound(_agvID, _targetNodeID, _purpose, path, false);
        } 
        else 
        {
            HandlePathFailed(_agvID, _targetNodeID, _serverTime, _purpose);
        }
    } 
    else 
    {
        HandlePathFailed(_agvID, _targetNodeID, _serverTime, _purpose);
    }
}

bool RoutePlanner::CreateRouteMatchingNodes(uint32_t _agvID, const std::vector<uint32_t>& _expectedNodeIDs,
                                            float _serverTime, MissionPurpose _purpose)
{
    if (_expectedNodeIDs.size() < 2)
    {
        std::cout << "[RoutePlanner] Exact route rejected: at least two nodes are required\n";
        return false;
    }

    Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(_agvID));
    if (!agv)
        return false;

    if (agv->GetCurrentNodeID() != _expectedNodeIDs.front())
    {
        std::cout << "[RoutePlanner] Exact route rejected for AGV " << _agvID
                  << ": current node=" << agv->GetCurrentNodeID()
                  << " expected=" << _expectedNodeIDs.front() << "\n";
        return false;
    }

    m_PendingRoutes.erase(std::remove_if(m_PendingRoutes.begin(), m_PendingRoutes.end(),
        [_agvID](const PendingRoute& r) { return r.agvID == _agvID; }), m_PendingRoutes.end());

    const uint32_t targetNodeID = _expectedNodeIDs.back();
    std::vector<PathStep> path;
    if (!TryFindPath(_agvID, targetNodeID, _serverTime, path))
    {
        std::cout << "[RoutePlanner] Exact route rejected: no path found for AGV " << _agvID << "\n";
        return false;
    }

    std::vector<RouteNodeTime> candidateRouteNodes;
    candidateRouteNodes.reserve(path.size());
    for (const PathStep& step : path)
        candidateRouteNodes.push_back({ step.nodeID, step.arrivalTime, step.departureTime });

    const bool matchesExpectedNodes = RouteMatchesExpectedNodeIDs(candidateRouteNodes, _expectedNodeIDs);

    if (!matchesExpectedNodes)
    {
        std::cout << "[RoutePlanner] Exact route rejected for AGV " << _agvID
                  << ": planner returned [";
        for (size_t i = 0; i < path.size(); ++i)
        {
            if (i > 0)
                std::cout << " -> ";
            std::cout << path[i].nodeID;
        }
        std::cout << "]\n";
        return false;
    }

    if (!TryReservePathTransaction(_agvID, path, targetNodeID, _serverTime))
    {
        std::cout << "[RoutePlanner] Exact route rejected: reservation conflict for AGV "
                  << _agvID << "\n";
        return false;
    }

    HandlePathFound(_agvID, targetNodeID, _purpose, path, true);
    return true;
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

void RoutePlanner::HandlePathFound(uint32_t _agvID, uint32_t _targetNodeID, MissionPurpose _purpose,
                                   const std::vector<PathStep>& path, bool _strictNodeSequence) {
    RoutePlan plan;
    plan.agvID = _agvID; plan.currentStepIndex = 1; plan.purpose = _purpose;
    plan.finalTargetNodeID = _targetNodeID; plan.steps = path;
    plan.strictNodeSequence = _strictNodeSequence;
    m_MasterPlans[_agvID] = plan;

    Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(_agvID));
    agv->SetMissionPurpose(_purpose);
    agv->ChangeState(AGVState::MOVING); 

    std::vector<RouteNodeTime> routeNodes;
    for (const auto& step : path) {
        routeNodes.push_back({step.nodeID, step.arrivalTime, step.departureTime});
    }

    IRobotController* controller = RobotManager::GetInstance().GetRobotController(_agvID);
    if (!controller || !controller->FollowRoute({_agvID, routeNodes}))
    {
        const float failureTime = path.empty() ? 0.0f : path.front().departureTime;
        StopRouteWithoutReplan(_agvID, agv, failureTime, "controller rejected route");
    }
}

void RoutePlanner::HandlePathFailed(uint32_t _agvID, uint32_t _targetNodeID, float _serverTime, MissionPurpose _purpose) {
    Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(_agvID));
    uint32_t curNodeID = agv->GetCurrentNodeID();

    ReservationTable::GetInstance().ReserveNode(curNodeID, _serverTime, _serverTime + REPLAN_PENALTY_TIME + 1.0f, _agvID, ReservationType::Normal);
    agv->ChangeState(AGVState::WAIT_REPLAN);
    m_PendingRoutes.push_back({ _agvID, _targetNodeID, _purpose, REPLAN_PENALTY_TIME });
}

void RoutePlanner::OnRobotStepCompleted(const RobotEvent& _e)
{
    uint32_t agvID = _e.agvID;
    Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(agvID));
    if (!agv) 
        return;

    if (m_MasterPlans.find(agvID) == m_MasterPlans.end()) return;
    RoutePlan& plan = m_MasterPlans[agvID]; 

    if (plan.currentStepIndex >= plan.steps.size() ||
        plan.steps[plan.currentStepIndex].nodeID != _e.currentNodeID)
    {
        StopRouteWithoutReplan(agvID, agv, _e.timestamp, "unexpected ARRIVED node");
        return;
    }

    UpdateRobotPosition(agv, plan, _e);
        
    if (ContinueCurrentRoute(agv, plan)) return;
    
    FinishRoute(agv, plan, _e);
}

void RoutePlanner::StopRouteWithoutReplan(uint32_t _agvID, Robo* _agv, float _serverTime, const char* _reason)
{
    if (IRobotController* controller = RobotManager::GetInstance().GetRobotController(_agvID))
        controller->CancelRoute();

    m_MasterPlans.erase(_agvID);
    m_PendingRoutes.erase(std::remove_if(m_PendingRoutes.begin(), m_PendingRoutes.end(),
        [_agvID](const PendingRoute& route) { return route.agvID == _agvID; }), m_PendingRoutes.end());

    const uint32_t confirmedNodeID = _agv->GetCurrentNodeID();
    _agv->ChangeState(AGVState::IDLE);
    _agv->StartWorkTimer(0.0f);

    auto& occupancy = OccupancyProvider::GetInstance();
    occupancy.ClearExecutionState(_agvID);
    occupancy.OccupyNode(_agvID, confirmedNodeID);

    auto& reservations = ReservationTable::GetInstance();
    reservations.OverrideFutureReservations(_agvID, _serverTime, 0.0f);
    reservations.ReserveNode(confirmedNodeID, _serverTime, _serverTime + LONG_TERM_HORIZON,
                             _agvID, ReservationType::Goal);

    std::cout << "[RoutePlanner] STRICT ROUTE SAFE STOP. AGV " << _agvID
              << " reason=" << _reason << " confirmedNode=" << confirmedNodeID << "\n";
}

void RoutePlanner::UpdateRobotPosition(Robo* agv, RoutePlan& plan, const RobotEvent& _e) {
    const uint32_t previousNodeID = agv->GetCurrentNodeID();
    auto& occupancy = OccupancyProvider::GetInstance();
    if (previousNodeID != _e.currentNodeID)
        occupancy.LeaveNode(_e.agvID, previousNodeID);

    occupancy.OccupyNode(_e.agvID, _e.currentNodeID);
    agv->SetCurrentNodeID(_e.currentNodeID);
    plan.currentStepIndex++; 
}

bool RoutePlanner::ContinueCurrentRoute(Robo* agv, RoutePlan& plan) {
    // 이동 뼈대 유지 및 인덱스 상태 바운더리 체크만 안전하게 수행
    if (plan.currentStepIndex >= plan.steps.size()) 
    {
        return false;
    }
    return true; 
}

void RoutePlanner::FinishRoute(Robo* agv, RoutePlan& plan, const RobotEvent& _e) {
    uint32_t agvID = _e.agvID;
    if (_e.currentNodeID == plan.finalTargetNodeID)
    {
        MissionPurpose purpose = agv->GetMissionPurpose();
        agv->StartWorkTimer(1.0f); 
        if (purpose == MissionPurpose::PICKUP) 
            agv->ChangeState(AGVState::LOADING);
        else if (purpose == MissionPurpose::DROP) 
            agv->ChangeState(AGVState::UNLOADING);
        else if (purpose == MissionPurpose::HOME) 
        {
            agv->ChangeState(AGVState::IDLE); agv->StartWorkTimer(0.0f); 
            EventManager::GetInstance().Publish({ RobotEventType::IDLE_READY, agvID, _e.timestamp });
        }
        else
        {
            agv->ChangeState(AGVState::IDLE);
            agv->StartWorkTimer(0.0f);
            std::cout << "[RoutePlanner] Route completed without automatic follow-up. AGV "
                      << agvID << " node=" << _e.currentNodeID << "\n";
        }
        m_MasterPlans.erase(agvID); 
    } 
    else 
    {
        uint32_t finalTarget = plan.finalTargetNodeID;
        MissionPurpose purpose = plan.purpose;
        float nextStartTime = _e.timestamp; 
        if (!plan.steps.empty()) nextStartTime = std::max(_e.timestamp, plan.steps.back().departureTime);
        m_MasterPlans.erase(agvID); 
        CreateRoute(agvID, finalTarget, nextStartTime, purpose); 
    }
}
