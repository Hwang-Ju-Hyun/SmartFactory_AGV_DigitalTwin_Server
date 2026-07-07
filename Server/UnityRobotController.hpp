#pragma once
#include "IRobotController.hpp"
#include <queue>
#include "Map.hpp"

struct CachedLink 
{
    MapLink link;
    MapNode fromNode;
    MapNode toNode;
    float departureTime; // fromNode 출발 시간
    float arrivalTime;   // toNode 도착 시간
};

class UnityRobotController : public IRobotController
{    
private:
    RoutePacket m_CurrentRoute;
    int m_CurrentLinkIndex;
    float m_LinkProgress;
    float m_Speed;

    float m_X;
    float m_Z;
    float m_Heading;

    std::vector<CachedLink> m_CachedLinks;
    std::queue<ControllerEvent> m_EventQueue;
public:
    UnityRobotController();
    virtual ~UnityRobotController() override;

    virtual void FollowRoute(const RoutePacket& _routePacket) override;
    virtual void CancelRoute() override;
    virtual StatusPacket GetStatus() override;    

    virtual bool HasEvent() const override;
    virtual ControllerEvent PopEvent() override;
    
    virtual void Update(float dt,float serverTime) override;
};
