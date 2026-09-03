#pragma once
#include "IRobotController.hpp"
#include "PhysicalFleetHeadingAnchor.hpp"
#include "PhysicalFleetCorrection.hpp"
#include "PhysicalFleetDispatchState.hpp"
#include "RobotSession.hpp"
#include "TCPSession.hpp"
#include <functional>
#include <queue>

struct ESP32TrajectoryExecutionConfig
{
    bool enabled = false;
    float millimetersPerMapUnit = 50.0f;
    float cruiseSpeedMmPerSecond = 80.0f;
    float initialHeadingToleranceRad =
        PhysicalFleetCorrectionPolicy::kHeadingToleranceRad;
    bool requireDepartureRelease = false;
};

class ESP32RobotController : public IRobotController
{
private:
    RobotSessionPtr m_RobotSession;
    RoutePacket m_CurrentRoute;
    ESP32TrajectoryExecutionConfig m_TrajectoryConfig;
    size_t m_CurrentEdgeStartIndex = 0;
    bool m_DispatchNextEdgePending = false;
    bool m_HasConfirmedStartHeading = false;
    float m_ConfirmedStartHeadingRad = 0.0f;
    bool m_ConfirmedStartHeadingFromVision = false;
    uint32_t m_ConfirmedStartHeadingVisionSequence = 0;
    PhysicalFleetDispatchState m_PhysicalDispatchState;
    std::queue<ControllerEvent> m_LocalEvents;

    std::function<bool(uint32_t, uint32_t, float, float)> m_TryOccupyEdgeCallback;
    std::function<void(uint32_t)> m_NodeLeaveCallback;
    std::function<bool(uint32_t)> m_CanEnterNodeCallback;
    std::function<bool(uint32_t)> m_IsNodeFreeCallback;

public:
    ESP32RobotController(TCPSessionPtr _robotSession);
    ESP32RobotController(RobotSessionPtr _robotSession,
                         ESP32TrajectoryExecutionConfig _trajectoryConfig = {});
    virtual ~ESP32RobotController()override{};
    bool FollowRoute(const RoutePacket& _routePacket) override;
    void CancelRoute() override;
    StatusPacket GetStatus() override;
    bool HasEvent() const override;
    ControllerEvent PopEvent() override;
    bool IsExpectedPhysicalArrival(uint32_t nodeID) const;
    bool CommitCorrectedPhysicalArrival(
        uint32_t nodeID,
        std::optional<PhysicalFleetHeadingAnchor> visionAnchor = std::nullopt);
    bool ReleasePhysicalDeparture(
        uint32_t startNodeID,
        uint32_t targetNodeID,
        PhysicalFleetHeadingAnchor visionAnchor);
    bool IsPhysicalDepartureHeld(uint32_t startNodeID,
                                 uint32_t targetNodeID) const;
    bool HasPhysicalDepartureHold() const;
    uint32_t GetCommittedPhysicalNodeID() const;
    bool HasCommittedPhysicalArrival() const;
    bool TryGetExpectedPhysicalEdge(uint32_t targetNodeID,
                                    uint32_t& outStartNodeID) const;
    bool TryGetExpectedArrivalHeading(uint32_t nodeID,
                                      float& outHeadingRad) const;
    uint32_t GetActivePhysicalRouteID() const;
    bool SupportsNodeCorrection() const;
    virtual void Update(float dt,float serverTime) override;
    virtual void SetTryOccupyEdgeCallback(std::function<bool(uint32_t, uint32_t, float, float)> callback) override { m_TryOccupyEdgeCallback = callback; }
    virtual void SetNodeLeaveCallback(std::function<void(uint32_t)> callback) override { m_NodeLeaveCallback = callback; }
    virtual void SetCanEnterNodeCallback(std::function<bool(uint32_t)> callback) override { m_CanEnterNodeCallback = callback; }
    virtual void SetIsNodeFreeCallback(std::function<bool(uint32_t)> callback) override { m_IsNodeFreeCallback = callback; }

private:
    bool DispatchCurrentPhysicalEdge();
    bool HoldCurrentPhysicalDeparture();
    bool ApplyConfirmedHeading(
        float nominalHeadingRad,
        std::optional<PhysicalFleetHeadingAnchor> visionAnchor);
};
