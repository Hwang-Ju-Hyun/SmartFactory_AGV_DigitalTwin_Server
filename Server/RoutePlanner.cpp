#include "RoutePlanner.hpp"
#include "AGVManager.hpp"
#include "PathFinder.hpp"
#include "TrafficControlManager.hpp"
#include <iostream>

void RoutePlanner::Init()
{
    // 로봇이 한 칸 이동을 마칠 때마다 관제탑이 반응하도록 수신기를 단다.
    EventManager::GetInstance().Subscribe(RobotEventType::MOVING_WAITING_COMPLETED, [this](const RobotEvent& _e) { OnRobotStepCompleted(_e); });
}

void RoutePlanner::CreateRoute(uint32_t _agvID, uint32_t _targetNodeID, float _serverTime, MissionPurpose _purpose)
{
    Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(_agvID));
    if (!agv) return;
    
    TrafficControlManager::GetInstance().ClearAgvReservations(_agvID);

    uint32_t curNodeID = agv->GetToNodeID(); // 로봇이 마지막으로 도달한(혹은 도달할) 노드

    AstarPathFinder apf;
    std::vector<uint32_t> path = apf.FindPath(
                                    curNodeID,
                                    _targetNodeID, 
                                    MapManager::GetInstance().GetNodes(), 
                                    MapManager::GetInstance().GetLinks(), 
                                    _agvID, 
                                    _serverTime);

    if (!path.empty())
    {
        ReserveRouteTimeline(_agvID, path, _serverTime);

        // 3. A* 경로 배열을 'Step' 단위의 계획표로 변환해서 마스터 장부에 저장
        RoutePlan plan;
        plan.agvID = _agvID;
        plan.currentStepIndex = 0;
        plan.purpose = _purpose;

        for (size_t i = 0; i < path.size() - 1; ++i)
        {
            plan.steps.push_back({ path[i], path[i + 1] });
        }
        m_MasterPlans[_agvID] = plan;

        auto fromNode=MapManager::GetInstance().GetMapNode(plan.steps[0].fromNodeID);
        auto toNode=MapManager::GetInstance().GetMapNode(plan.steps[0].toNodeID);
        
        // 4. 로봇에게 첫 번째 지시 하달! "출발해!"
        agv->SetMissionPurpose(_purpose);
        agv->AssignNextStep(fromNode,toNode,AGVState::MOVING);
        
        std::cout << "[관제탑] AGV " << _agvID << "번 경로 발급 완료. 목적지: " << _targetNodeID << std::endl;
    }
    else
    {
        // 길 찾기 실패 시 비상 대기 (5초 보호)
        std::cout << "AGV " << _agvID << "번 경로 없음. 임시 대기 보호 전개." << std::endl;
        TrafficControlManager::GetInstance().ReserveNode(curNodeID, _serverTime, _serverTime + 5.0f, _agvID);
        
        //  재시도 큐 등은 나중에 여기에 다시 붙이면 됨!
    }
}

void RoutePlanner::OnRobotStepCompleted(const RobotEvent& _e)
{
    uint32_t agvID = _e.agvID;
    
    // 계획표에 없는 로봇이면 무시
    if (m_MasterPlans.find(agvID) == m_MasterPlans.end()) return;
    
    RoutePlan& plan = m_MasterPlans[agvID];
    Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(agvID));

    plan.currentStepIndex++; // 한 칸 완료했으니 다음 칸으로!

    // 아직 갈 길이 남았다면? 다음 한 칸을 찔러줌
    if (plan.currentStepIndex < plan.steps.size())
    {
        uint32_t nextFrom = plan.steps[plan.currentStepIndex].fromNodeID;
        uint32_t nextTo   = plan.steps[plan.currentStepIndex].toNodeID;
        
        MapNode fromNode = MapManager::GetInstance().GetNodes().at(nextFrom);
        MapNode toNode   = MapManager::GetInstance().GetNodes().at(nextTo);

        // 만약 nextFrom == nextTo 라면 로봇은 알아서 제자리에서 1초간 대기(WAIT)하는 물리 연산을 수행함! 상태 변경 필요 없음!
        agv->AssignNextStep(fromNode, toNode, AGVState::MOVING);
    }
    else // 목적지 도착
    {
        MissionPurpose purpose = agv->GetMissionPurpose();
        
        auto toNode=MapManager::GetInstance().GetMapNode(agv->GetToNodeID());

        if (purpose == MissionPurpose::PICKUP) 
            agv->AssignNextStep(toNode,toNode, AGVState::LOADING);
        else if (purpose == MissionPurpose::DROP) 
            agv->AssignNextStep(toNode,toNode, AGVState::UNLOADING);
        else if (purpose == MissionPurpose::HOME) 
            agv->AssignNextStep(toNode, toNode, AGVState::ARRIVED);
        
        m_MasterPlans.erase(agvID); // 미션 끝! 장부에서 파기
    }
}

// 예전 Robo.cpp에 있던 ReserveTimeLine 코드가 통째로 여기로 이사 옵니다!
void RoutePlanner::ReserveRouteTimeline(uint32_t _agvID, const std::vector<uint32_t>& _path, float _serverTime)
{
    if (_path.empty()) return;

    Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(_agvID));
    if (!agv) return;

    // 1. 기존 장부 초기화
    TrafficControlManager::GetInstance().ClearAgvReservations(_agvID);
    TrafficControlManager::GetInstance().ClearLinkReservations(_agvID);

    float accTime = _serverTime;
    float speed = agv->GetSpeed();    

    
    // 2. 출발 노드 찰나의 예약 (출발 직전 발밑)
    uint32_t startNodeID = _path[0];
    TrafficControlManager::GetInstance().ReserveNode(startNodeID, accTime, accTime + 0.1f, _agvID);        

    // 3. 경로 배열을 순회하며 시간표(Timeline) 작성
    for (size_t i = 0; i < _path.size() - 1; i++)
    {
        uint32_t fromID = _path[i];
        uint32_t toID = _path[i + 1];

        //  WHCA* 핵심: A*가 준 경로가 "제자리 대기(Wait)" 라면?
        if (fromID == toID)
        {
            float enterTime = accTime;
            float leaveTime = accTime + agv->GetWaitTime();

            // 링크 이동은 안 하니까 링크 예약은 패스! 노드만 점유 연장!
            TrafficControlManager::GetInstance().ReserveNode(toID, enterTime, leaveTime, _agvID);
            accTime = leaveTime; // 1초 흘려보냄
            continue;
        }
        
        // 정상적인 다음 노드로의 이동 (from -> to)
        MapNode fromNode = MapManager::GetInstance().GetNodes().at(fromID);
        MapNode toNode = MapManager::GetInstance().GetNodes().at(toID);

        float dist = std::sqrt(std::pow(fromNode.m_PosX - toNode.m_PosX, 2) + std::pow(fromNode.m_PosZ - toNode.m_PosZ, 2));
        float travelTime = dist / speed;
       
        float linkEnterTime = accTime;               
        float linkLeaveTime = accTime + travelTime;  
        
        // 링크(도로) 점유
        TrafficControlManager::GetInstance().ReserveLink(fromID, toID, linkEnterTime, linkLeaveTime, _agvID);
        
        float nodeEnterTime = linkLeaveTime;

        // 최종 목적지에 도착했다면
        if (i + 1 == _path.size() - 1) 
        {         
            // 상차/하차를 하거나 새 명령을 받을 때까지 무한대로 알박기
            // (나중에 스케줄러가 새 길 줄 때 알아서 지워주니까 ㄱㅊ)            
            TrafficControlManager::GetInstance().ReserveNode(toID, nodeEnterTime, nodeEnterTime+5.f, _agvID);            
        }
        else 
        {         
            // 중간 경유지는 멈추지 않고 스쳐 지나가므로 0.1초만 점유
            TrafficControlManager::GetInstance().ReserveNode(toID, nodeEnterTime, nodeEnterTime + 0.1f, _agvID);
        }        

        accTime = nodeEnterTime; // 도착 시간이 다음 출발 시간이 됨
    }
}



void RoutePlanner::Update(float _deltaTime, float _serverTime)
{
    for (auto iter = m_PendingRoutes.begin(); iter != m_PendingRoutes.end(); )
    {
        iter->retryTimer -= _deltaTime;

        if (iter->retryTimer <= 0.0f)
        {
            uint32_t retryAgvID = iter->agvID;
            uint32_t targetID = iter->targetNodeID;
            MissionPurpose purpose = iter->purpose;
            
            iter = m_PendingRoutes.erase(iter); // 꺼내기
            
            std::cout << "[관제탑]  AGV " << retryAgvID << "번 체증 풀렸는지 재탐색 시도" << std::endl;
            CreateRoute(retryAgvID, targetID, _serverTime, purpose); // 다시 길찾기 
        }
        else
        {
            iter++;
        }
    }
}