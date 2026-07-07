#include "UnityRobotController.hpp"
#include "Map.hpp"
#include "BezierFollower.hpp"

UnityRobotController::UnityRobotController()
    : m_CurrentLinkIndex(0)
    , m_LinkProgress(0.0f)
    , m_Speed(4.0f)
    , m_X(0.0f)
    , m_Z(0.0f)
{
}

UnityRobotController::~UnityRobotController()
{
    
}
#include <iostream>

void UnityRobotController::FollowRoute(const RoutePacket& _routePacket)
{   
   while(!m_EventQueue.empty()) 
    { m_EventQueue.pop(); } // 찌꺼기 청소

    m_CurrentRoute = _routePacket;
    m_CurrentLinkIndex = 0;
    m_LinkProgress = 0.0f;
    m_CachedLinks.clear();

    // 🌟 노드 배열을 순회하며 출발/도착 시간을 링크와 매핑합니다.
    for (size_t i = 0; i < _routePacket.nodes.size() - 1; i++)
    {
        uint32_t fromID = _routePacket.nodes[i].nodeID;
        uint32_t toID   = _routePacket.nodes[i+1].nodeID;
        
        // 이 링크를 탈 때의 출발 시간과 도착 시간
        float depTime = _routePacket.nodes[i].departureTime;
        float arrTime = _routePacket.nodes[i+1].arrivalTime;

        MapLink link = MapManager::GetInstance().FindLink(fromID, toID);
        MapNode from = MapManager::GetInstance().GetMapNode(fromID);
        MapNode to   = MapManager::GetInstance().GetMapNode(toID);

        m_CachedLinks.push_back({ link, from, to, depTime, arrTime });
    }
}

void UnityRobotController::CancelRoute()
{
    // 유니티 로봇에게 경로 취소를 전달하는 로직 구현
}

StatusPacket UnityRobotController::GetStatus()
{
    StatusPacket p;
    p.agvID=m_CurrentRoute.agvID;
    if(m_CurrentLinkIndex < m_CurrentRoute.linkIDs.size())
    {
        p.currentLinkID = m_CurrentRoute.linkIDs[m_CurrentLinkIndex];
    }
    else
    {
        p.currentLinkID = 0; // 경로가 없거나 완료된 경우
    }
    p.progress = m_LinkProgress;
    p.x = m_X;
    p.z = m_Z;
    p.heading = m_Heading; 
    p.velocity = m_Speed; // 로봇의 현재 주행 속도
    p.battery = 100.0f; // 임시

    return p;
}

bool UnityRobotController::HasEvent() const
{
    return !m_EventQueue.empty();
}

ControllerEvent UnityRobotController::PopEvent()
{
    if(m_EventQueue.empty())
    {
        return ControllerEvent{ControllerEventType::NONE, 0};
    }
    ControllerEvent event = m_EventQueue.front();
    m_EventQueue.pop();
    return event;
}
#include <iostream>

void UnityRobotController::Update(float dt, float serverTime)
{   
if (m_CachedLinks.empty() || m_CurrentLinkIndex >= m_CachedLinks.size()) 
        return; 

    const CachedLink& cache = m_CachedLinks[m_CurrentLinkIndex];
    
    // 🌟 [핵심 1] 대기(Wait) 처리: 서버 시간이 아직 출발 시간에 도달하지 않았다면 멈춰 있습니다.
    if (serverTime < cache.departureTime) {
        m_X = cache.fromNode.m_PosX;
        m_Z = cache.fromNode.m_PosZ;
        return; // 교차로나 갓길에서 대기하는 효과 완벽 구현!
    }

    // 🌟 [핵심 2] 속도 대신 시간 비율(Time Ratio)로 Progress 계산
    float duration = cache.arrivalTime - cache.departureTime;
    
    if (duration <= 0.001f) {
        m_LinkProgress = 1.0f; // 즉시 공간이동 방어
    } else {
        m_LinkProgress = (serverTime - cache.departureTime) / duration;
    }

    // 🌟 [핵심 3] 프레임 드랍으로 인해 도착 시간을 초과(1.0 이상)했을 경우의 이어달리기
    while (m_LinkProgress >= 1.0f)
    {
        m_X = m_CachedLinks[m_CurrentLinkIndex].toNode.m_PosX;
        m_Z = m_CachedLinks[m_CurrentLinkIndex].toNode.m_PosZ;

        m_EventQueue.push({ ControllerEventType::ARRIVED, m_CachedLinks[m_CurrentLinkIndex].toNode.m_Id });
        
        m_CurrentLinkIndex++;
        
        if (m_CurrentLinkIndex >= m_CachedLinks.size()) {
            m_LinkProgress = 0.0f;
            break;
        }

        // 🌟 다음 링크로 넘어왔으므로 서버 시간에 맞춰 Progress 재계산
        const CachedLink& nextCache = m_CachedLinks[m_CurrentLinkIndex];
        
        // 만약 다음 노드 출발 시간 전이라면 여기서 while문 탈출 후 대기!
        if (serverTime < nextCache.departureTime) {
            m_LinkProgress = 0.0f;
            break;
        }
        
        float nextDuration = nextCache.arrivalTime - nextCache.departureTime;
        if (nextDuration <= 0.001f) m_LinkProgress = 1.0f;
        else m_LinkProgress = (serverTime - nextCache.departureTime) / nextDuration;
    }

    // 베지어 곡선 좌표 계산
    if (m_CurrentLinkIndex < m_CachedLinks.size() && m_LinkProgress > 0.0f)
    {
        const CachedLink& currentCache = m_CachedLinks[m_CurrentLinkIndex];
        Vector2 pos = BezierFollower::Evaluate(currentCache.fromNode, currentCache.toNode, currentCache.link, m_LinkProgress);
        m_X = pos.x;
        m_Z = pos.z;
        m_Heading = BezierFollower::Heading(currentCache.fromNode, currentCache.toNode, currentCache.link, m_LinkProgress);
    }
}
