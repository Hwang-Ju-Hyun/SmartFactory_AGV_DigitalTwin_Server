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

    float m_X = 0.0f;
    float m_Z = 0.0f;
    float m_Heading = 0.0f;

    std::vector<CachedLink> m_CachedLinks;
    std::queue<ControllerEvent> m_EventQueue;
    
    std::function<bool(uint32_t, uint32_t)> m_ClearanceCallback;
    std::function<void(uint32_t, uint32_t)> m_EdgeEnterCallback;

public:
    UnityRobotController();
    virtual ~UnityRobotController() override;

    virtual void FollowRoute(const RoutePacket& _routePacket) override;
    virtual void CancelRoute() override;
    virtual StatusPacket GetStatus() override;  

    virtual bool HasEvent() const override;
    virtual ControllerEvent PopEvent() override;
    
    virtual void Update(float dt, float serverTime) override;
        
    virtual void SetClearanceCallback(std::function<bool(uint32_t, uint32_t)> callback) override { m_ClearanceCallback = callback; }
    virtual void SetEdgeEnterCallback(std::function<void(uint32_t, uint32_t)> callback) override { m_EdgeEnterCallback = callback; }

    //bool AdvanceToNextLink(float serverTime,float plannedDuration);
};