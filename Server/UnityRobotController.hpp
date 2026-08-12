#pragma once
#include "IRobotController.hpp"
#include "MovementSimulator.hpp"
#include <queue>
#include <functional> 

class UnityRobotController : public IRobotController
{   
private:
    MovementSimulator m_Simulator;
    std::queue<ControllerEvent> m_EventQueue;
    
    std::function<bool(uint32_t, uint32_t, float, float)> m_TryOccupyEdgeCallback;
    std::function<void(uint32_t)> m_NodeLeaveCallback;
    std::function<bool(uint32_t)> m_CanEnterNodeCallback;
    std::function<bool(uint32_t)> m_IsNodeFreeCallback;

public:
    UnityRobotController();
    UnityRobotController(uint32_t agvID, float x, float z, float heading = 0.0f);
    virtual ~UnityRobotController() override;

    virtual bool FollowRoute(const RoutePacket& _routePacket) override;
    virtual void CancelRoute() override;
    virtual StatusPacket GetStatus() override;  

    virtual bool HasEvent() const override;
    virtual ControllerEvent PopEvent() override;
    virtual void Update(float dt, float serverTime) override;
        
    virtual void SetTryOccupyEdgeCallback(std::function<bool(uint32_t, uint32_t, float, float)> callback) override { m_TryOccupyEdgeCallback = callback; }
    virtual void SetNodeLeaveCallback(std::function<void(uint32_t)> callback) override { m_NodeLeaveCallback = callback; }
    virtual void SetCanEnterNodeCallback(std::function<bool(uint32_t)> callback) override { m_CanEnterNodeCallback = callback; }
    virtual void SetIsNodeFreeCallback(std::function<bool(uint32_t)> callback) override { m_IsNodeFreeCallback = callback; }

private:
    void DrainSimulatorEvents();
};
