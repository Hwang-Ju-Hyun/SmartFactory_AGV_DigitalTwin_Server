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
    std::cout << "[FollowRoute] AGV=" << _routePacket.agvID << " Links=";
    for(auto id : _routePacket.linkIDs) {
        std::cout << id << " ";
    }
    m_CurrentRoute = _routePacket;
    m_CurrentLinkIndex = 0;
    m_LinkProgress = 0.0f;
    m_Speed=4.f;
    m_CachedLinks.clear();

    for (uint32_t linkID : _routePacket.linkIDs)
        {
            
             MapLink link = MapManager::GetInstance().GetLink(linkID);
             MapNode from = MapManager::GetInstance().GetMapNode(link.m_FromNodeID);
             MapNode to   = MapManager::GetInstance().GetMapNode(link.m_ToNodeID);
            
            // 이제 안전하게 원본 데이터의 주소를 저장할 수 있습니다.
            m_CachedLinks.push_back({ link, from, to });
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

void UnityRobotController::Update(float dt)
{   

    if (m_CachedLinks.empty() || m_CurrentLinkIndex >= m_CachedLinks.size()) 
    { 
        m_Speed = 0.0f; 
        //m_CurrentRoute.linkIDs.clear(); 
        return; 
    }

    const CachedLink& cache = m_CachedLinks[m_CurrentLinkIndex];
    float estimatedLinkLength =cache.link.m_Dist;

    if (estimatedLinkLength <= 0) 
        estimatedLinkLength = 1.0f;
        
    m_LinkProgress += (m_Speed / estimatedLinkLength) * dt;

    if (m_LinkProgress >= 1.0f)
    {         
    std::cout << "[PUSH] AGV=" << m_CurrentRoute.agvID 
                  << " LinkIndex=" << m_CurrentLinkIndex << "/" << m_CachedLinks.size() 
                  << " Node=" << cache.toNode.m_Id << std::endl;
        m_EventQueue.push({ ControllerEventType::ARRIVED, cache.toNode.m_Id });
        m_CurrentLinkIndex++;
        m_LinkProgress = 0.0f;             
    }
    else
    {
        Vector2 pos = BezierFollower::Evaluate(cache.fromNode, cache.toNode, cache.link, m_LinkProgress);
        m_X = pos.x;
        m_Z = pos.z;
        m_Heading=BezierFollower::Heading( cache.fromNode, cache.toNode, cache.link, m_LinkProgress);
    }
    
    if(m_CurrentLinkIndex >= m_CurrentRoute.linkIDs.size())
    {   
        return;     
    }   
}
