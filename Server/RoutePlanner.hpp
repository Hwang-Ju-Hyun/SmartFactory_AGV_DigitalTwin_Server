#pragma once
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <deque>
#include "Event.hpp"
#include "Robo.hpp" 
#include "RRAstar.hpp"

// 로봇이 수행할 '딱 한 칸'의 지시사항 (더 이상 안 쓰지만 호환성을 위해 유지)
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
    
    // A*의 시공간 스텝을 그대로 들고 있습니다.
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

    void Init(); 
    
    void CreateRoute(uint32_t _agvID, uint32_t _targetNodeID, float _serverTime, MissionPurpose _purpose);
    void Update(float _deltaTime, float _serverTime); 
    
    void ClearRRAEngines() { m_RRAEngines.clear(); }
private:
    RoutePlanner() = default;
    
    void OnRobotStepCompleted(const RobotEvent& _e);

    // 유일한 예약 함수. 전체 검사 후 한 번에 기록합니다.
    bool TryReservePathTransaction(uint32_t _agvID, const std::vector<PathStep>& _path, uint32_t _finalTargetID, float _serverTime);

    void OnLinkBlocked(uint32_t _fromNodeID, uint32_t _toNodeID, float _serverTime);

    std::unordered_map<uint32_t, RoutePlan> m_MasterPlans; // 중앙 계획표 장부
    std::unordered_map<uint32_t, RRAStar> m_RRAEngines;        
public:
    std::deque<PendingRoute> m_PendingRoutes; // 비상 대기열    

private:
    bool TryFindPath(uint32_t _agvID, uint32_t _targetNodeID, float _serverTime, std::vector<PathStep>& outPath);
    void HandlePathFound(uint32_t _agvID, uint32_t _targetNodeID, MissionPurpose _purpose, const std::vector<PathStep>& path);
    void HandlePathFailed(uint32_t _agvID, uint32_t _targetNodeID, float _serverTime, MissionPurpose _purpose);

    void UpdateRobotPosition(Robo* agv, RoutePlan& plan, const RobotEvent& _e);
    bool ContinueCurrentRoute(Robo* agv, RoutePlan& plan);
    void FinishRoute(Robo* agv, RoutePlan& plan, const RobotEvent& _e);
};