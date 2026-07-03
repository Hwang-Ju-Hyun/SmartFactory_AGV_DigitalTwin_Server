#pragma once
#include "IRobotController.hpp"

class UnityRobotController : public IRobotController
{
public:
    UnityRobotController() = default;
    virtual ~UnityRobotController() override = default;

    virtual void FollowRoute(const RoutePacket& routePacket) override;
    virtual void CancelRoute() override;
    virtual StatusPacket GetStatus() override;
    virtual bool IsArrived() const override;
    virtual void Update(float dt) override;
};
