#pragma once
#include "IRobotController.hpp"
#include "RobotSession.hpp"
#include "TCPSession.hpp"
#include <functional>

class ESP32RobotController : public IRobotController
{
private:
    RobotSessionPtr m_RobotSession;
    RoutePacket m_CurrentRoute;

    std::function<bool(uint32_t, uint32_t, float, float)> m_TryOccupyEdgeCallback;
    std::function<void(uint32_t)> m_NodeLeaveCallback;
    std::function<bool(uint32_t)> m_CanEnterNodeCallback;
    std::function<bool(uint32_t)> m_IsNodeFreeCallback;

public:
    ESP32RobotController(TCPSessionPtr _robotSession);
    ESP32RobotController(RobotSessionPtr _robotSession);
    virtual ~ESP32RobotController()override{};
    void FollowRoute(const RoutePacket& _routePacket) override;
    void CancelRoute() override;
    StatusPacket GetStatus() override;
    bool HasEvent() const override;
    ControllerEvent PopEvent() override;
    virtual void Update(float dt,float serverTime) override;
    virtual void SetTryOccupyEdgeCallback(std::function<bool(uint32_t, uint32_t, float, float)> callback) override { m_TryOccupyEdgeCallback = callback; }
    virtual void SetNodeLeaveCallback(std::function<void(uint32_t)> callback) override { m_NodeLeaveCallback = callback; }
    virtual void SetCanEnterNodeCallback(std::function<bool(uint32_t)> callback) override { m_CanEnterNodeCallback = callback; }
    virtual void SetIsNodeFreeCallback(std::function<bool(uint32_t)> callback) override { m_IsNodeFreeCallback = callback; }
};
