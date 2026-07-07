#pragma once
#include "SharedPackets.hpp"
#include "TCPSession.hpp"
enum class ControllerEventType : uint8_t 
{ 
    NONE = 0, 
    ARRIVED, 
    ERROR_SLIP, 
    EMERGENCY_STOP 
};

struct ControllerEvent 
{ 
    ControllerEventType type = ControllerEventType::NONE; 
    uint32_t nodeID = 0; 
};

class IRobotController
{
public:
    IRobotController(){};
    IRobotController(TCPSessionPtr _robotSession){};
    virtual ~IRobotController()=default;
public:    
    // 관제탑이 경로(임무)를 하달할 때 호출
    virtual void FollowRoute(const RoutePacket& routePacket) = 0;

    // 관제탑이 긴급 정지나 경로 취소를 명령할 때 호출
    virtual void CancelRoute() = 0;

    // 로봇의 현재 상태(위치, 속도, 배터리 등)를 서버가 요구할 때 반환
    virtual StatusPacket GetStatus() = 0;

    virtual bool HasEvent() const = 0;
    virtual ControllerEvent PopEvent() = 0;

    // 로봇 내부의 물리적/논리적 상태 업데이트 (매 프레임 호출)
    virtual void Update(float dt,float serverTime) = 0;
};