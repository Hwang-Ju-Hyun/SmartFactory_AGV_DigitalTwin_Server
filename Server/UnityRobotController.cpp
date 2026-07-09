#include "UnityRobotController.hpp"
#include "Map.hpp"
#include "BezierFollower.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>

const float AGV_TAIL_OFFSET = 0.8f; 

UnityRobotController::UnityRobotController()
    : m_CurrentLinkIndex(0), m_LinkProgress(0.0f), m_IsMovingLink(false)
    , m_ActualStartTime(0.0f), m_OverTime(0.0f), m_HasReleasedFromNode(false)
    , m_X(0.0f), m_Z(0.0f), m_Heading(0.0f) {}

UnityRobotController::~UnityRobotController() {}

void UnityRobotController::FollowRoute(const RoutePacket& _routePacket)
{   
    while(!m_EventQueue.empty()) { m_EventQueue.pop(); } 

    m_CurrentRoute = _routePacket;
    m_CurrentLinkIndex = 0;
    m_LinkProgress = 0.0f;
    m_CachedLinks.clear();
    
    m_OverTime = 0.0f;
    m_IsMovingLink = false;
    m_ActualStartTime = 0.0f;
    m_HasReleasedFromNode = false; 

    for (size_t i = 0; i < _routePacket.nodes.size() - 1; i++)
    {
        uint32_t fromID = _routePacket.nodes[i].nodeID;
        uint32_t toID   = _routePacket.nodes[i+1].nodeID;
        float depTime   = _routePacket.nodes[i].departureTime;
        float arrTime   = _routePacket.nodes[i+1].arrivalTime;

        MapLink link = MapManager::GetInstance().FindLink(fromID, toID);
        MapNode from = MapManager::GetInstance().GetMapNode(fromID);
        MapNode to   = MapManager::GetInstance().GetMapNode(toID);

        m_CachedLinks.push_back({ link, from, to, depTime, arrTime });
    }
}

void UnityRobotController::CancelRoute()
{
    m_CachedLinks.clear();
    m_IsMovingLink = false;
    while(!m_EventQueue.empty()) { m_EventQueue.pop(); }
}

StatusPacket UnityRobotController::GetStatus()
{
    StatusPacket p;
    p.agvID = m_CurrentRoute.agvID;
    if(m_CurrentLinkIndex < m_CachedLinks.size()) p.currentLinkID = m_CachedLinks[m_CurrentLinkIndex].link.m_Id;
    else p.currentLinkID = 0; 
    
    p.progress = m_LinkProgress;
    p.x = m_X; p.z = m_Z; p.heading = m_Heading; 
    p.velocity = 4.0f; p.battery = 100.0f; 
    return p;
}

bool UnityRobotController::HasEvent() const { return !m_EventQueue.empty(); }

ControllerEvent UnityRobotController::PopEvent()
{
    if(m_EventQueue.empty()) return ControllerEvent{ControllerEventType::NONE, 0};
    ControllerEvent event = m_EventQueue.front();
    m_EventQueue.pop();
    return event;
}

void UnityRobotController::Update(float dt, float serverTime)
{   
    if (m_CachedLinks.empty() || m_CurrentLinkIndex >= m_CachedLinks.size()) return; 

    const CachedLink& cache = m_CachedLinks[m_CurrentLinkIndex];
    float plannedDuration = cache.arrivalTime - cache.departureTime;
    if (plannedDuration <= 0.001f) plannedDuration = 1.0f;
    
    // ==========================================
    // 1. 대기 및 출발 결심 
    // ==========================================
    if (!m_IsMovingLink)
    {
        if (serverTime < cache.departureTime) return; 

        // 동적 남은 시간 및 도착 예상 시각 산출
        float remainingDuration = plannedDuration - m_OverTime;
        if (remainingDuration <= 0.01f) remainingDuration = 0.01f;
        float dynamicExpectedArrival = serverTime + remainingDuration;

        // 원자적 검증 및 점유 획득 시도
        //todo
        // if (m_TryOccupyEdgeCallback && !m_TryOccupyEdgeCallback(cache.fromNode.m_Id, cache.toNode.m_Id, serverTime, dynamicExpectedArrival)) 
        //     return; 

        m_IsMovingLink = true;
        m_HasReleasedFromNode = false; 

        float compensatedTime = serverTime - m_OverTime;
        m_ActualStartTime = std::max(compensatedTime, cache.departureTime);
        m_OverTime = 0.0f; 
    }
    
    m_LinkProgress = (serverTime - m_ActualStartTime) / plannedDuration;

    // ==========================================
    // 2. [수정 완료] 차체 물리 기반 동적 꼬리 탈출 연산
    // ==========================================
    if (m_IsMovingLink && !m_HasReleasedFromNode)
    {
        float linkDist = (cache.link.m_Type == 1) ? cache.link.m_Dist : 
            std::sqrt(std::pow(cache.toNode.m_PosX - cache.fromNode.m_PosX, 2) + std::pow(cache.toNode.m_PosZ - cache.fromNode.m_PosZ, 2));
        
        if (linkDist <= 0.0f) linkDist = 1.0f;
        
        //차량의 꼬리가 완전히 노드 반경을 벗어나는 비율 계산
        float dynamicReleaseRatio = AGV_TAIL_OFFSET / linkDist;
        dynamicReleaseRatio = std::max(0.15f, std::min(dynamicReleaseRatio, 0.85f)); // 극단적 예외 방어 클램핑

        if (m_LinkProgress >= dynamicReleaseRatio)
        {
            if (m_NodeLeaveCallback) m_NodeLeaveCallback(cache.fromNode.m_Id);
            m_HasReleasedFromNode = true;
        }
    }

    // ==========================================
    // 3. 1프레임 도착 판정 (1프레임 지연 부여)
    // ==========================================
    if (m_LinkProgress >= 1.0f)
    {
        m_X = cache.toNode.m_PosX;
        m_Z = cache.toNode.m_PosZ;

        float exactArrivalTime = m_ActualStartTime + plannedDuration;
        m_OverTime = serverTime - exactArrivalTime; 

        m_EventQueue.push({ ControllerEventType::ARRIVED, cache.toNode.m_Id });
        
        m_CurrentLinkIndex++;
        m_IsMovingLink = false; 

        if (m_CurrentLinkIndex >= m_CachedLinks.size()) 
        {
            m_LinkProgress = 0.0f;
            m_OverTime = 0.0f; 
        }
        return; 
    }

    if (m_IsMovingLink)
    {
        Vector2 pos = BezierFollower::Evaluate(cache.fromNode, cache.toNode, cache.link, m_LinkProgress);
        m_X = pos.x; m_Z = pos.z;
        m_Heading = BezierFollower::Heading(cache.fromNode, cache.toNode, cache.link, m_LinkProgress);
    }
}