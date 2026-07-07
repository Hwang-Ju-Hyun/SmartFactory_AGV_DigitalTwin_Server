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
};