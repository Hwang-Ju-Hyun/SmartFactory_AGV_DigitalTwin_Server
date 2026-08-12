#pragma once

#include "Map.hpp"
#include "PacketSerializer.hpp"
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

struct TrajectoryBuildOptions
{
    // A path tangent is not a substitute for the robot's actual pose because
    // using it would erase a required initial turn.
    bool hasTrustedStartHeading = false;
    float startHeadingRad = 0.0f;
    float millimetersPerMapUnit = 1.0f;
    float spacingMm = 50.0f;
    float cruiseSpeedMmPerSecond = 100.0f;
    float cornerStopThresholdRad = 0.35f;
    bool lineEndpointOnly = false;
    bool stopAtEveryNodeBoundary = false;
};

class TrajectoryBuilder
{
public:
    // Production entry point. MapData.json remains the source of geometry.
    static bool Build(const std::vector<uint32_t>& nodeIDs,
                      uint32_t routeID,
                      const TrajectoryBuildOptions& options,
                      RobotProtocol::TrajectoryCommandPayload& outTrajectory,
                      std::string& outError);

    // Geometry-injected overload used by deterministic smoke tests and map
    // import validation. It follows the same directed-link rules as Build().
    static bool BuildFromGeometry(
        const std::vector<uint32_t>& nodeIDs,
        const std::unordered_map<uint32_t, MapNode>& nodes,
        const std::vector<MapLink>& links,
        uint32_t routeID,
        const TrajectoryBuildOptions& options,
        RobotProtocol::TrajectoryCommandPayload& outTrajectory,
        std::string& outError);
};
