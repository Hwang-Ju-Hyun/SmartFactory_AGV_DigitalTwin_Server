#pragma once
#include <vector>
#include <unordered_map>
#include <cstdint>
#include "Event.hpp"
#include "Robo.hpp" 
#include <queue>
#include "RRAstar.hpp"

// 로봇이 수행할 '딱 한 칸'의 지시사항
struct RouteStep 
{
    uint32_t fromNodeID;
    uint32_t toNodeID;
};

// 특정 로봇의 처음부터 끝까지의 전체 계획표
struct RoutePlan 
{
    uint32_t agvID;
    MissionPurpose purpose;
    uint32_t finalTargetNodeID;
    
    // 기존의 from, to 짝을 맞추던 steps 대신 PathStep 리스트를 통째로 들고 있습니다.
    std::vector<PathStep> steps; 
    size_t currentStepIndex; 
};
struct PendingRoute 
{
    uint32_t agvID;
    uint32_t targetNodeID;
    MissionPurpose purpose;
    float retryTimer; // 몇 초 뒤에 다시 길찾기 시도할지
};

class RoutePlanner 
{
public:
    static RoutePlanner& GetInstance() { static RoutePlanner instance; return instance; }

    // 초기화 시 이벤트(STEP_COMPLETED) 구독
    void Init(); 
    
    // 길을 찾고, 예약을 걸고, 계획표를 만듭니다.
    void CreateRoute(uint32_t _agvID, uint32_t _targetNodeID, float _serverTime, MissionPurpose _purpose);

    void Update(float _deltaTime, float _serverTime); 
private:
    RoutePlanner() = default;
    
    void OnRobotStepCompleted(const RobotEvent& _e);

    void ReserveRouteTimeline(uint32_t _agvID, const std::vector<PathStep>& _path, float _serverTime, uint32_t _finalTargetID);

    void OnLinkBlocked(uint32_t _fromNodeID,uint32_t _toNodeID,float _serverTime);

    std::unordered_map<uint32_t, RoutePlan> m_MasterPlans; // 중앙 계획표 장부
    std::unordered_map<uint32_t, RRAStar> m_RRAEngines;    
public:
    std::deque<PendingRoute> m_PendingRoutes; // 비상 대기열    
};