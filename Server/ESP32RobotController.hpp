#pragma once
#include "IRobotController.hpp"
#include "TCPSession.hpp"

class ESP32RobotController : public IRobotController
{
private:
    TCPSessionPtr m_RobotSession; // ESP32 로봇과의 TCP 세션
public:
    ESP32RobotController(TCPSessionPtr _robotSession);
    virtual ~ESP32RobotController()override{};
    void FollowRoute(const RoutePacket& _routePacket) override;
    void CancelRoute() override;
    StatusPacket GetStatus() override;    
    virtual void Update(float dt,float serverTime) override;
    virtual void SetTryOccupyEdgeCallback(std::function<bool(uint32_t, uint32_t, float, float)> callback) override {}
    virtual void SetNodeLeaveCallback(std::function<void(uint32_t)> callback) override {}
    virtual void SetCanEnterNodeCallback(std::function<bool(uint32_t)> callback) override {}
    virtual void SetIsNodeFreeCallback(std::function<bool(uint32_t)> callback) override {}
};
