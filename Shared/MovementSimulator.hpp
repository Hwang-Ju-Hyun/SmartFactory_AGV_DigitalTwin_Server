#pragma once
#include "Map.hpp"
#include "SharedPackets.hpp"
#include <cstdint>
#include <functional>
#include <queue>
#include <vector>

struct MovementCachedLink
{
    MapLink link;
    MapNode fromNode;
    MapNode toNode;
    float departureTime;
    float arrivalTime;
};

enum class MovementEventType : uint8_t
{
    NONE = 0,
    ARRIVED,
    EXECUTION_BLOCKED
};

struct MovementEvent
{
    MovementEventType type = MovementEventType::NONE;
    uint32_t nodeID = 0;
    uint32_t relatedNodeID = 0;
};

struct MovementCallbacks
{
    std::function<bool(uint32_t, uint32_t, float, float)> tryOccupyEdge;
    std::function<void(uint32_t)> onNodeLeave;
    std::function<bool(uint32_t)> canEnterNode;
    std::function<bool(uint32_t)> isNodeFree;
};

class MovementSimulator
{
private:
    RoutePacket m_CurrentRoute;
    size_t m_CurrentLinkIndex = 0;
    float m_LinkProgress = 0.0f;
    float m_DistanceOnLink = 0.0f;
    float m_CurrentVelocity = 0.0f;
    float m_CurrentAngularVelocity = 0.0f;

    bool m_IsMovingLink = false;
    float m_ActualStartTime = 0.0f;
    float m_OverTime = 0.0f;
    bool m_HasReleasedFromNode = false;
    float m_ExecutionWaitTime = 0.0f;
    uint32_t m_ExecutionWaitAttempts = 0;
    bool m_BlockEventSent = false;

    float m_X = 0.0f;
    float m_Z = 0.0f;
    float m_Heading = 0.0f;

    std::vector<MovementCachedLink> m_CachedLinks;
    std::queue<MovementEvent> m_EventQueue;

public:
    MovementSimulator() = default;
    MovementSimulator(uint32_t agvID, float x, float z, float heading = 0.0f);

    void LoadRoute(const RoutePacket& routePacket);
    void CancelRoute();
    void Update(float dt, float serverTime, const MovementCallbacks& callbacks = {});

    StatusPacket GetStatus() const;
    bool HasEvent() const;
    MovementEvent PopEvent();

    bool HasRoute() const { return !m_CachedLinks.empty(); }
    bool IsRouteComplete() const { return m_CachedLinks.empty() || m_CurrentLinkIndex >= m_CachedLinks.size(); }
    uint32_t GetAgvID() const { return m_CurrentRoute.agvID; }
};
