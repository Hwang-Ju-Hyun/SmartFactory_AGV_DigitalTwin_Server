#pragma once
#include "IRobotController.hpp"
#include <queue>
#include <functional> 
#include "Map.hpp"

struct CachedLink 
{
    MapLink link;
    MapNode fromNode;
    MapNode toNode;
    float departureTime; 
    float arrivalTime;   
};

class UnityRobotController : public IRobotController
{   
private:
    RoutePacket m_CurrentRoute;
    size_t m_CurrentLinkIndex; 
    float m_LinkProgress;
    
    bool m_IsMovingLink = false;      
    float m_ActualStartTime = 0.0f;   
    float m_OverTime = 0.0f;
    bool m_HasReleasedFromNode = false; // 🌟 오타 및 멤버 변수 일치 완료
    float m_ExecutionWaitTime = 0.0f;
    uint32_t m_ExecutionWaitAttempts = 0;
    bool m_BlockEventSent = false;

    float m_X = 0.0f;
    float m_Z = 0.0f;
    float m_Heading = 0.0f;

    std::vector<CachedLink> m_CachedLinks;
    std::queue<ControllerEvent> m_EventQueue;
    
    std::function<bool(uint32_t, uint32_t, float, float)> m_TryOccupyEdgeCallback;
    std::function<void(uint32_t)> m_NodeLeaveCallback;
    std::function<bool(uint32_t)> m_CanEnterNodeCallback;
    std::function<bool(uint32_t)> m_IsNodeFreeCallback;

public:
    UnityRobotController();
    UnityRobotController(uint32_t agvID, float x, float z, float heading = 0.0f);
    virtual ~UnityRobotController() override;

    virtual void FollowRoute(const RoutePacket& _routePacket) override;
    virtual void CancelRoute() override;
    virtual StatusPacket GetStatus() override;  

    virtual bool HasEvent() const override;
    virtual ControllerEvent PopEvent() override;
    virtual void Update(float dt, float serverTime) override;
        
    virtual void SetTryOccupyEdgeCallback(std::function<bool(uint32_t, uint32_t, float, float)> callback) override { m_TryOccupyEdgeCallback = callback; }
    virtual void SetNodeLeaveCallback(std::function<void(uint32_t)> callback) override { m_NodeLeaveCallback = callback; }
    virtual void SetCanEnterNodeCallback(std::function<bool(uint32_t)> callback) override { m_CanEnterNodeCallback = callback; }
    virtual void SetIsNodeFreeCallback(std::function<bool(uint32_t)> callback) override { m_IsNodeFreeCallback = callback; }
};
