#include "UnityRobotController.hpp"
#include "Map.hpp"
#include "BezierFollower.hpp"
#include <iostream>

UnityRobotController::UnityRobotController()
    : m_CurrentLinkIndex(0)
    , m_LinkProgress(0.0f)
    , m_IsMovingLink(false)
    , m_ActualStartTime(0.0f)
    , m_X(0.0f)
    , m_Z(0.0f)
    , m_Heading(0.0f)
{
}

UnityRobotController::~UnityRobotController() {}

void UnityRobotController::FollowRoute(const RoutePacket& _routePacket)
{   
    while(!m_EventQueue.empty()) { m_EventQueue.pop(); } //찌꺼기 이벤트 청소

    m_CurrentRoute = _routePacket;
    m_CurrentLinkIndex = 0;
    m_LinkProgress = 0.0f;
    m_CachedLinks.clear();
    m_OverTime = 0.0f;

    m_IsMovingLink = false;
    m_ActualStartTime = 0.0f;

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
    
    // 2. 우체통에 남아있는 발송 전 이벤트 청소
    while(!m_EventQueue.empty()) { m_EventQueue.pop(); }

    // 3 서버에 "나 도로/노드 점유 풀고 여기서 정지할게!" 라고 보고
    // (이 로직을 호출하려면 UnityRobotController가 자신의 agvID를 알아야 하므로
    // m_CurrentRoute.agvID 를 사용하여 장부를 지웁니다)
    if (m_CurrentRoute.agvID != 0) 
    {
        // [주의] Controller가 직접 DB를 참조하지 않게 하려면, 
        // IRobotController에 CancelCallback을 하나 더 뚫거나, 
        // 외부(NetworkManager나 RoutePlanner)에서 CancelRoute() 호출 직후에 
        // ReservationTable::GetInstance().ReleaseAllOccupancyAndReservations()를 호출해 주어야 합니다.
        // 현재 구조상 외부에서 호출하는 것이 결합도를 낮추는 데 좋습니다. (아래 3번 항목 참고)
    }
}

StatusPacket UnityRobotController::GetStatus()
{
    StatusPacket p;
    p.agvID = m_CurrentRoute.agvID;
    
    
    if(m_CurrentLinkIndex < m_CachedLinks.size())
    {
        p.currentLinkID = m_CachedLinks[m_CurrentLinkIndex].link.m_Id;
    }
    else
    {
        p.currentLinkID = 0; 
    }
    
    p.progress = m_LinkProgress;
    p.x = m_X;
    p.z = m_Z;
    p.heading = m_Heading; 
    p.velocity = 4.0f; // 필요시 계산된 속도를 넣거나 0/4 처리
    p.battery = 100.0f; 

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

void UnityRobotController::Update(float dt, float serverTime)
{   
//     if (m_CachedLinks.empty() || m_CurrentLinkIndex >= m_CachedLinks.size()) return; 

//     const CachedLink& cache = m_CachedLinks[m_CurrentLinkIndex];
    
//     // ==========================================
//     // 1. 대기 및 출발 결심 로직 
//     // ==========================================
//     if (!m_IsMovingLink)
//     {
//         if (serverTime < cache.departureTime) return; 

//         // 앞길 점유 확인 콜백
//         if (m_ClearanceCallback && !m_ClearanceCallback(cache.fromNode.m_Id, cache.toNode.m_Id)) 
//             return; 

//         m_IsMovingLink = true;
        
//         // 🌟 [멘토 피드백] 과거로 시간 역행하는 버그 방지! (예약시간보다 빨라지지 않음)
//         float compensatedTime = serverTime - m_OverTime;
//         m_ActualStartTime = std::max(compensatedTime, cache.departureTime);
//         m_OverTime = 0.0f; // 잉여 시간 소진 완료

//         if (m_EdgeEnterCallback) 
//             m_EdgeEnterCallback(cache.fromNode.m_Id, cache.toNode.m_Id); 
//     }

//     // ==========================================
//     // 2. 진행도 계산
//     // ==========================================
//     float plannedDuration = cache.arrivalTime - cache.departureTime;
//     if (plannedDuration <= 0.001f) plannedDuration = 1.0f;
    
//     m_LinkProgress = (serverTime - m_ActualStartTime) / plannedDuration;

//     // ==========================================
//     // 3. 도착 판정 (1프레임 종료의 마법)
//     // ==========================================
//     if (m_LinkProgress >= 1.0f)
//     {std::cout
// << "[ARRIVE]"
// << " AGV=" << m_CurrentRoute.agvID
// << " node="
// << cache.toNode.m_Id
// << " server="
// << serverTime
// << std::endl;
//         m_X = cache.toNode.m_PosX;
//         m_Z = cache.toNode.m_PosZ;

//         float exactArrivalTime = m_ActualStartTime + plannedDuration;
//         m_OverTime = serverTime - exactArrivalTime;

//         // 도착 편지 발송
//         m_EventQueue.push({ ControllerEventType::ARRIVED, cache.toNode.m_Id });
        
//         m_CurrentLinkIndex++;
//         m_IsMovingLink = false;

//         // 🌟 [멘토 피드백] 목적지 최종 도착 시, 깔끔하게 상태를 소멸시킵니다.
//         if (m_CurrentLinkIndex >= m_CachedLinks.size()) 
//         {
//             m_LinkProgress = 0.0f;
//             m_OverTime = 0.0f; // 목적지에 도착했으므로 잉여 시간은 리셋
//         }

//         // 🌟 이번 프레임 물리 이동 종료! (서버가 메인루프 Step 2에서 이벤트를 가져가 OccupyNode를 할 수 있도록 넘겨줌)
//         return; 
//     }

//     // ==========================================
//     // 4. 물리적 렌더링 좌표 계산
//     // ==========================================std::cout
// std::cout
// << "[START LINK]"
// << " AGV=" << m_CurrentRoute.agvID
// << " "
// << cache.fromNode.m_Id
// << "->"
// << cache.toNode.m_Id
// << " server="
// << serverTime
// << std::endl;
//     if (m_IsMovingLink)
//     {
//         Vector2 pos = BezierFollower::Evaluate(cache.fromNode, cache.toNode, cache.link, m_LinkProgress);
//         m_X = pos.x; m_Z = pos.z;
//         m_Heading = BezierFollower::Heading(cache.fromNode, cache.toNode, cache.link, m_LinkProgress);
//     }

if (m_CachedLinks.empty() || m_CurrentLinkIndex >= m_CachedLinks.size()) return; 

    const CachedLink& cache = m_CachedLinks[m_CurrentLinkIndex];
    
    // ==========================================
    // 1. 대기 및 출발 결심 로직 (AdvanceToNextLink의 역할을 여기서 다 해줍니다!)
    // ==========================================
    if (!m_IsMovingLink)
    {
        if (serverTime < cache.departureTime) return; 

        // 앞길 점유 확인 콜백
        if (m_ClearanceCallback && !m_ClearanceCallback(cache.fromNode.m_Id, cache.toNode.m_Id)) 
            return; 

        m_IsMovingLink = true;
        
        // 과거로 시간 역행하는 버그 방지! (예약시간보다 빨라지지 않음)
        float compensatedTime = serverTime - m_OverTime;
        m_ActualStartTime = std::max(compensatedTime, cache.departureTime);
        m_OverTime = 0.0f; // 잉여 시간 소진 완료

        // 출발! 링크(도로) 점유
        if (m_EdgeEnterCallback) 
            m_EdgeEnterCallback(cache.fromNode.m_Id, cache.toNode.m_Id); 
    }

    // ==========================================
    // 2. 진행도 계산
    // ==========================================
    float plannedDuration = cache.arrivalTime - cache.departureTime;
    if (plannedDuration <= 0.001f) plannedDuration = 1.0f;
    
    m_LinkProgress = (serverTime - m_ActualStartTime) / plannedDuration;

    // ==========================================
    // 3. 도착 판정 (🌟 1프레임 종료의 마법 🌟)
    // ==========================================
    // while 대신 if를 써서 1프레임에 노드 하나씩만 처리하도록 합니다!
    if (m_LinkProgress >= 1.0f)
    {
        std::cout << "[ARRIVE] AGV=" << m_CurrentRoute.agvID << " node=" << cache.toNode.m_Id << std::endl;
        
        m_X = cache.toNode.m_PosX;
        m_Z = cache.toNode.m_PosZ;

        float exactArrivalTime = m_ActualStartTime + plannedDuration;
        m_OverTime = serverTime - exactArrivalTime; // 다음 프레임을 위한 시간 보정

        // 🌟 도착 편지 발송
        m_EventQueue.push({ ControllerEventType::ARRIVED, cache.toNode.m_Id });
        
        m_CurrentLinkIndex++;
        m_IsMovingLink = false; // 일단 정지 상태로 전환

        // 목적지 최종 도착 시 상태 소멸
        if (m_CurrentLinkIndex >= m_CachedLinks.size()) 
        {
            std::cout << "[END ROUTE] AGV=" << m_CurrentRoute.agvID << std::endl;
            m_LinkProgress = 0.0f;
            m_OverTime = 0.0f; 
        }

        // 🌟🌟 [가장 중요] 무조건 여기서 함수를 끝내서 서버가 편지를 읽을 시간을 줍니다! 🌟🌟
        return; 
    }

    // ==========================================
    // 4. 물리적 렌더링 좌표 계산
    // ==========================================
    if (m_IsMovingLink)
    {
        Vector2 pos = BezierFollower::Evaluate(cache.fromNode, cache.toNode, cache.link, m_LinkProgress);
        m_X = pos.x; m_Z = pos.z;
        m_Heading = BezierFollower::Heading(cache.fromNode, cache.toNode, cache.link, m_LinkProgress);
    }
}

// bool UnityRobotController::AdvanceToNextLink(float serverTime,float plannedDuration)
// {
//     std::cout
//         << "[AdvanceToNextLink] AGV=" << m_CurrentRoute.agvID
//         << " currentIndex=" << m_CurrentLinkIndex
//         << " size=" << m_CachedLinks.size()
//         << std::endl;
//     m_CurrentLinkIndex++;
//     m_IsMovingLink = false;

//     if (m_CurrentLinkIndex >= m_CachedLinks.size()) 
//     {
//          std::cout << "[END ROUTE]" << std::endl;
//         m_LinkProgress = 0.0f;
//         return false; 
//     }

//     const CachedLink& nextCache = m_CachedLinks[m_CurrentLinkIndex];

//     // 초과된 잉여 시간 계산 (넘겨받은 plannedDuration 사용)
//     float overTime = (serverTime - m_ActualStartTime) - plannedDuration;

//     //  콜백 안전 검사 및 조건 분리
//     if (serverTime < nextCache.departureTime || 
//         (m_ClearanceCallback && !m_ClearanceCallback(nextCache.fromNode.m_Id, nextCache.toNode.m_Id))) 
//     {
//         std::cout
//         << "[WAIT NEXT]"
//         << " server=" << serverTime
//         << " dep=" << nextCache.departureTime
//         << std::endl;

//         m_LinkProgress = 0.0f;
//         return false; 
//     }

//     if (m_ClearanceCallback &&
//     !m_ClearanceCallback(nextCache.fromNode.m_Id,
//                          nextCache.toNode.m_Id))
// {
//     std::cout
//         << "[BLOCK NEXT]"
//         << " "
//         << nextCache.fromNode.m_Id
//         << "->"
//         << nextCache.toNode.m_Id
//         << std::endl;

//     return false;
// }

//     m_IsMovingLink = true;
//     m_ActualStartTime = serverTime - overTime;
    
//     if (m_EdgeEnterCallback) m_EdgeEnterCallback(nextCache.fromNode.m_Id, nextCache.toNode.m_Id);

//     float nextDuration = nextCache.arrivalTime - nextCache.departureTime;
//     if (nextDuration <= 0.001f) nextDuration = 1.0f;
//     m_LinkProgress = (serverTime - m_ActualStartTime) / nextDuration;

//     return true;
// }