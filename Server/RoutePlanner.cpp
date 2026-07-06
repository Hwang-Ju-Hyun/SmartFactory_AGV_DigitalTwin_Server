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

    // 2단계: 기록 (안전 마진을 주어 기존 예약 덮어쓰기)
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
std::cout
<< "\n=========================\n"
<< "CreateRoute\n"
<< "AGV : " << _agvID
<< "\nCurrent : " << agv->GetCurrentNodeID()
<< "\nTarget : " << _targetNodeID
<< "\nPurpose : " << (int)_purpose
<< "\n";
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
    std::cout << "Target : " << _targetNodeID << std::endl;
    std::cout << "Last   : " << path.back().nodeID << std::endl;
    //assert(path.back().nodeID == _targetNodeID);
    std::cout << "Path : ";
for (auto& step : path)
{
    std::cout << step.nodeID << " ";
}
std::cout << std::endl;
    //[핵심 변경: 탐색 실패 시]
    if (path.size() < 2 || !TryReservePathTransaction(_agvID, path, _targetNodeID, _serverTime))
    {std::cout
<< "[FAILED]"
<< " AGV=" << _agvID
<< " pathsize=" << path.size()
<< std::endl;
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

    // 서버 장부에 나 움직인다고 상태 갱신
    agv->SetMissionPurpose(_purpose);
    agv->ChangeState(AGVState::MOVING); // (AssignNextStep 대신 가볍게 상태만 바꿈!)

    // A*가 찾은 전체 Node 경로를 LinkID 배열로 변환
    std::vector<uint32_t> routeLinks;
    for (size_t i = 0; i < path.size() - 1; i++)
    {
        MapLink link = MapManager::GetInstance().FindLink(path[i].nodeID, path[i+1].nodeID);
        routeLinks.push_back(link.m_Id); 
    }

    RoutePacket commandPacket;
    commandPacket.agvID = _agvID;
    commandPacket.linkIDs = routeLinks; 

    // 로봇에게 "목적지까지 혼자 달려라" 명령 하달
    IRobotController* controller = RobotManager::GetInstance().GetRobotController(_agvID);
    std::cout
<< "Controller = "
<< controller<<"Purpose : " << (int)_purpose
<< std::endl;
    if (controller)
    {
        controller->FollowRoute(commandPacket);
        std::cout << "[관제탑] AGV " << _agvID << "번 전체 경로 하달 완료. (총 " << routeLinks.size() << "개 링크)" << std::endl;
    }
}

void RoutePlanner::OnRobotStepCompleted(const RobotEvent& _e)
{
    uint32_t agvID = _e.agvID;
    Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(agvID));
    if (!agv) return;

    if (m_MasterPlans.find(agvID) == m_MasterPlans.end()) return;
    RoutePlan& plan = m_MasterPlans[agvID]; 

    // 1. 서버 장부 갱신: 로봇의 실제 위치를 업데이트합니다.
    agv->SetCurrentNodeID(_e.currentNodeID);
        
    size_t prevIndex = plan.currentStepIndex;
    plan.currentStepIndex++; 

    // 2. 부분 경로가 아직 남아있는 경우 (로봇은 이미 계속 달리고 있음)
    if (plan.currentStepIndex < plan.steps.size())
    {
        const PathStep& currentStep = plan.steps[prevIndex]; // 명시적 이전 스텝
        const PathStep& nextStep    = plan.steps[plan.currentStepIndex]; // 명시적 다음 스텝
        
        auto fromNode = MapManager::GetInstance().GetMapNode(currentStep.nodeID);
        auto toNode   = MapManager::GetInstance().GetMapNode(nextStep.nodeID);
        
        // 서버의 장부만 다음 노드를 향해 '이동 중'으로 갱신 (서버 장부 동기화 용도)
        agv->AssignNextStep(fromNode, toNode, AGVState::MOVING, currentStep.departureTime, nextStep.arrivalTime);
        return;
    }

    // ==========================================================
    // 3. 발급받은 경로(부분 경로)의 끝에 도달했을 경우
    // ==========================================================
    
    if (_e.currentNodeID == plan.finalTargetNodeID)
    {
        // [CASE A: 거기가 진짜 최종 목적지일 때]
        MissionPurpose purpose = agv->GetMissionPurpose();
        agv->StartWorkTimer(1.0f); // 1초 상하차 타이머 시작 (추후 하드웨어 센서 신호로 교체 가능)

        if (purpose == MissionPurpose::PICKUP) 
        {
            agv->ChangeState(AGVState::LOADING);
            std::cout << "[관제탑] AGV " << agvID << "번 상차지 도착. LOADING 시작." << std::endl;
        }
        else if (purpose == MissionPurpose::DROP) 
        {
            agv->ChangeState(AGVState::UNLOADING);
            std::cout << "[관제탑] AGV " << agvID << "번 하차지 도착. UNLOADING 시작." << std::endl;
        }
        else if (purpose == MissionPurpose::HOME) 
        {
            agv->ChangeState(AGVState::IDLE);
            agv->StartWorkTimer(0.0f); // 집에서는 타이머 필요 없음
            
            RobotEvent homeEvent = { RobotEventType::IDLE_READY, agvID, _e.timestamp, _e.currentNodeID };
            EventManager::GetInstance().Publish(homeEvent);
            std::cout << "[관제탑] AGV " << agvID << "번 HOME 복귀 완료." << std::endl;
        }
        
        m_MasterPlans.erase(agvID); // 새 경로 발급 전에 깔끔하게 장부 삭제 완료!
    }
    else
    {
        // [CASE B: 타임 윈도우 때문에 발급받은 '부분 경로'가 끝났을 때]
        std::cout << "[관제탑] AGV " << agvID << "번 부분 경로(Window Time) 도달. 남은 경로 재탐색!" << std::endl;
        
        uint32_t finalTarget = plan.finalTargetNodeID;
        MissionPurpose purpose = plan.purpose;
        
        //  로봇이 지각했을 경우 과거로 돌아가는 시간 역전 버그 방지
        float nextStartTime = _e.timestamp; 
        if (!plan.steps.empty()) 
        {
            nextStartTime = std::max(_e.timestamp, plan.steps.back().departureTime);
        }
        
        m_MasterPlans.erase(agvID); 
        
        CreateRoute(agvID, finalTarget, nextStartTime, purpose); 
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
        else 
        {
            ++it;
        }
    }

    for(auto& route : retryList)
    {
        CreateRoute(route.agvID, route.targetNodeID, serverTime, route.purpose);
    }
}