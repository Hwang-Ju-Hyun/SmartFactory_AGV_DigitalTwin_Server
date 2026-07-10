#pragma once
#include "SharedPackets.hpp"
#include "TCPSession.hpp"
#include <functional>

enum class ControllerEventType : uint8_t 
{ 
    NONE = 0, 
    ARRIVED, 
    ERROR_SLIP, 
    EMERGENCY_STOP,
    EXECUTION_BLOCKED
};

struct ControllerEvent 
{ 
    ControllerEventType type = ControllerEventType::NONE; 
    uint32_t nodeID = 0; 
    uint32_t relatedNodeID = 0;
};

class IRobotController
{
public:
    IRobotController(){};
    IRobotController(TCPSessionPtr _robotSession){};
    virtual ~IRobotController()=default;
    
    virtual void FollowRoute(const RoutePacket& routePacket) = 0;
    virtual void CancelRoute() = 0;
    virtual StatusPacket GetStatus() = 0;

    virtual bool HasEvent() const = 0;
    virtual ControllerEvent PopEvent() = 0;
    virtual void Update(float dt, float serverTime) = 0;

    // Check와 Occupy를 묶은 Atomic 콜백
    virtual void SetTryOccupyEdgeCallback(std::function<bool(uint32_t, uint32_t, float, float)> callback) = 0;
    virtual void SetNodeLeaveCallback(std::function<void(uint32_t)> callback) = 0;
    virtual void SetCanEnterNodeCallback(std::function<bool(uint32_t)> callback) = 0;
    virtual void SetIsNodeFreeCallback(std::function<bool(uint32_t)> callback) = 0;
};
