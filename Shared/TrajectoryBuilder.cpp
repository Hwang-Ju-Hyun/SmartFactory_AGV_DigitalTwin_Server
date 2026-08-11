#include "TrajectoryBuilder.hpp"
#include "BezierFollower.hpp"
#include <algorithm>
#include <cmath>
#include <utility>

namespace
{
    constexpr int kArcLengthSamples = 96;
    constexpr float kGeometryEpsilon = 0.0001f;
    constexpr float kPi = 3.14159265358979323846f;

    struct ArcLengthSample
    {
        float t = 0.0f;
        float distance = 0.0f;
    };

    float NormalizeAngle(float angle)
    {
        while (angle > kPi) angle -= 2.0f * kPi;
        while (angle < -kPi) angle += 2.0f * kPi;
        return angle;
    }

    float Distance(const Vector2& lhs, const Vector2& rhs)
    {
        return std::hypot(rhs.x - lhs.x, rhs.z - lhs.z);
    }

    const MapLink* FindDirectedLink(const std::vector<MapLink>& links,
                                    uint32_t fromNodeID,
                                    uint32_t toNodeID)
    {
        auto it = std::find_if(links.begin(), links.end(),
            [fromNodeID, toNodeID](const MapLink& link)
            {
                return link.m_FromNodeID == fromNodeID &&
                       link.m_ToNodeID == toNodeID;
            });
        return it == links.end() ? nullptr : &(*it);
    }

    bool ValidateOptions(const TrajectoryBuildOptions& options, std::string& outError)
    {
        if (!options.hasTrustedStartHeading ||
            !std::isfinite(options.startHeadingRad))
        {
            outError = "a finite trusted robot start heading is required";
            return false;
        }
        if (!std::isfinite(options.millimetersPerMapUnit) ||
            options.millimetersPerMapUnit <= 0.0f)
        {
            outError = "millimetersPerMapUnit must be finite and positive";
            return false;
        }
        if (!std::isfinite(options.spacingMm) || options.spacingMm <= 0.0f)
        {
            outError = "spacingMm must be finite and positive";
            return false;
        }
        if (!std::isfinite(options.cruiseSpeedMmPerSecond) ||
            options.cruiseSpeedMmPerSecond < 0.0f)
        {
            outError = "cruiseSpeedMmPerSecond must be finite and non-negative";
            return false;
        }
        if (!std::isfinite(options.cornerStopThresholdRad) ||
            options.cornerStopThresholdRad < 0.0f ||
            options.cornerStopThresholdRad > kPi)
        {
            outError = "cornerStopThresholdRad must be between 0 and pi";
            return false;
        }
        return true;
    }

    bool AppendWaypoint(RobotProtocol::TrajectoryCommandPayload& trajectory,
                        const RobotProtocol::TrajectoryWaypoint& waypoint,
                        std::string& outError)
    {
        if (!std::isfinite(waypoint.forwardMm) ||
            !std::isfinite(waypoint.leftMm) ||
            !std::isfinite(waypoint.headingRad) ||
            !std::isfinite(waypoint.targetSpeedMmPerSecond) ||
            waypoint.targetSpeedMmPerSecond < 0.0f)
        {
            outError = "trajectory contains a non-finite coordinate or invalid speed";
            return false;
        }
        if (trajectory.waypoints.size() >= RobotProtocol::kMaxTrajectoryWaypoints)
        {
            outError = "trajectory exceeds the 64-waypoint protocol limit";
            return false;
        }
        trajectory.waypoints.push_back(waypoint);
        return true;
    }

    std::vector<ArcLengthSample> BuildArcLengthTable(const MapNode& fromNode,
                                                      const MapNode& toNode,
                                                      const MapLink& link)
    {
        std::vector<ArcLengthSample> table;
        table.reserve(kArcLengthSamples + 1);

        Vector2 previous = BezierFollower::Evaluate(fromNode, toNode, link, 0.0f);
        float accumulatedDistance = 0.0f;
        table.push_back({ 0.0f, 0.0f });
        for (int i = 1; i <= kArcLengthSamples; ++i)
        {
            const float t = static_cast<float>(i) / static_cast<float>(kArcLengthSamples);
            const Vector2 current = BezierFollower::Evaluate(fromNode, toNode, link, t);
            accumulatedDistance += Distance(previous, current);
            table.push_back({ t, accumulatedDistance });
            previous = current;
        }
        return table;
    }

    float FindParameterAtDistance(const std::vector<ArcLengthSample>& table,
                                  float targetDistance)
    {
        auto upper = std::lower_bound(table.begin(), table.end(), targetDistance,
            [](const ArcLengthSample& sample, float distance)
            {
                return sample.distance < distance;
            });

        if (upper == table.begin()) return upper->t;
        if (upper == table.end()) return table.back().t;

        const ArcLengthSample& next = *upper;
        const ArcLengthSample& previous = *(upper - 1);
        const float span = next.distance - previous.distance;
        if (span <= kGeometryEpsilon) return next.t;

        const float alpha = (targetDistance - previous.distance) / span;
        return previous.t + (next.t - previous.t) * alpha;
    }

    RobotProtocol::TrajectoryWaypoint ToLocalWaypoint(
        const Vector2& worldPosition,
        float worldHeading,
        const Vector2& origin,
        float originHeading,
        float millimetersPerMapUnit,
        float targetSpeedMmPerSecond)
    {
        const float dx = worldPosition.x - origin.x;
        const float dz = worldPosition.z - origin.z;
        const float cosHeading = std::cos(originHeading);
        const float sinHeading = std::sin(originHeading);

        RobotProtocol::TrajectoryWaypoint waypoint;
        waypoint.forwardMm = (cosHeading * dx + sinHeading * dz) * millimetersPerMapUnit;
        waypoint.leftMm = (-sinHeading * dx + cosHeading * dz) * millimetersPerMapUnit;
        waypoint.headingRad = NormalizeAngle(worldHeading - originHeading);
        waypoint.targetSpeedMmPerSecond = targetSpeedMmPerSecond;
        return waypoint;
    }
}

bool TrajectoryBuilder::Build(const std::vector<uint32_t>& nodeIDs,
                              uint32_t routeID,
                              const TrajectoryBuildOptions& options,
                              RobotProtocol::TrajectoryCommandPayload& outTrajectory,
                              std::string& outError)
{
    const auto nodes = MapManager::GetInstance().GetNodes();
    const auto& links = MapManager::GetInstance().GetLinks();
    return BuildFromGeometry(nodeIDs, nodes, links, routeID, options,
                             outTrajectory, outError);
}

bool TrajectoryBuilder::BuildFromGeometry(
    const std::vector<uint32_t>& nodeIDs,
    const std::unordered_map<uint32_t, MapNode>& nodes,
    const std::vector<MapLink>& links,
    uint32_t routeID,
    const TrajectoryBuildOptions& options,
    RobotProtocol::TrajectoryCommandPayload& outTrajectory,
    std::string& outError)
{
    outTrajectory = {};
    outError.clear();
    RobotProtocol::TrajectoryCommandPayload trajectory;

    if (!ValidateOptions(options, outError)) return false;
    if (nodeIDs.size() < 2 || nodeIDs.size() > RobotProtocol::kMaxRouteNodes)
    {
        outError = "trajectory requires between 2 and 64 route nodes";
        return false;
    }

    std::vector<const MapNode*> routeNodes;
    std::vector<const MapLink*> routeLinks;
    routeNodes.reserve(nodeIDs.size());
    routeLinks.reserve(nodeIDs.size() - 1);

    for (uint32_t nodeID : nodeIDs)
    {
        auto nodeIt = nodes.find(nodeID);
        if (nodeIt == nodes.end())
        {
            outError = "route references an unknown node " + std::to_string(nodeID);
            return false;
        }
        routeNodes.push_back(&nodeIt->second);
    }

    for (size_t i = 0; i + 1 < nodeIDs.size(); ++i)
    {
        const MapLink* link = FindDirectedLink(links, nodeIDs[i], nodeIDs[i + 1]);
        if (!link)
        {
            outError = "missing directed link " + std::to_string(nodeIDs[i]) +
                       " -> " + std::to_string(nodeIDs[i + 1]);
            return false;
        }
        if (link->m_Type != 0 && link->m_Type != 1)
        {
            outError = "unsupported link type on " + std::to_string(nodeIDs[i]) +
                       " -> " + std::to_string(nodeIDs[i + 1]);
            return false;
        }
        routeLinks.push_back(link);
    }

    const MapNode& firstNode = *routeNodes.front();
    const Vector2 origin{ firstNode.m_PosX, firstNode.m_PosZ };
    const float originHeading = NormalizeAngle(options.startHeadingRad);

    trajectory.routeID = routeID;
    trajectory.formatVersion = RobotProtocol::kTrajectoryFormatVersion;
    trajectory.startNodeID = nodeIDs.front();
    trajectory.finalNodeID = nodeIDs.back();
    trajectory.millimetersPerMapUnit = options.millimetersPerMapUnit;

    RobotProtocol::TrajectoryWaypoint startWaypoint = ToLocalWaypoint(
        origin, originHeading, origin, originHeading,
        options.millimetersPerMapUnit, options.cruiseSpeedMmPerSecond);
    startWaypoint.nodeID = nodeIDs.front();
    startWaypoint.flags = RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY;
    if (!AppendWaypoint(trajectory, startWaypoint, outError)) return false;

    const MapNode& secondNode = *routeNodes[1];
    const MapLink& firstLink = *routeLinks.front();
    const float firstLinkHeading =
        BezierFollower::Heading(firstNode, secondNode, firstLink, 0.0f);
    const float initialHeadingChange =
        std::abs(NormalizeAngle(firstLinkHeading - originHeading));
    if (initialHeadingChange > options.cornerStopThresholdRad)
    {
        RobotProtocol::TrajectoryWaypoint& stoppedStart = trajectory.waypoints.back();
        stoppedStart.flags |= RobotProtocol::TRAJECTORY_FLAG_STOP;
        stoppedStart.targetSpeedMmPerSecond = 0.0f;

        RobotProtocol::TrajectoryWaypoint initialRotate = ToLocalWaypoint(
            origin, firstLinkHeading, origin, originHeading,
            options.millimetersPerMapUnit, 0.0f);
        initialRotate.nodeID = 0;
        initialRotate.flags = RobotProtocol::TRAJECTORY_FLAG_ROTATE_IN_PLACE;
        if (!AppendWaypoint(trajectory, initialRotate, outError)) return false;
    }

    for (size_t linkIndex = 0; linkIndex < routeLinks.size(); ++linkIndex)
    {
        const MapNode& fromNode = *routeNodes[linkIndex];
        const MapNode& toNode = *routeNodes[linkIndex + 1];
        const MapLink& link = *routeLinks[linkIndex];
        const std::vector<ArcLengthSample> arcTable =
            BuildArcLengthTable(fromNode, toNode, link);
        const float linkLengthMapUnits = arcTable.back().distance;
        if (!std::isfinite(linkLengthMapUnits) || linkLengthMapUnits <= kGeometryEpsilon)
        {
            outError = "zero-length or invalid geometry on link " +
                       std::to_string(fromNode.m_Id) + " -> " +
                       std::to_string(toNode.m_Id);
            return false;
        }

        const float linkLengthMm = linkLengthMapUnits * options.millimetersPerMapUnit;
        const float requestedSegmentCount = std::ceil(linkLengthMm / options.spacingMm);
        if (!std::isfinite(requestedSegmentCount) ||
            requestedSegmentCount > static_cast<float>(RobotProtocol::kMaxTrajectoryWaypoints))
        {
            outError = "link requires more than the 64-waypoint protocol limit";
            return false;
        }
        const size_t segmentCount = std::max<size_t>(
            1, static_cast<size_t>(requestedSegmentCount));

        for (size_t segment = 1; segment <= segmentCount; ++segment)
        {
            const float targetDistance = linkLengthMapUnits *
                (static_cast<float>(segment) / static_cast<float>(segmentCount));
            const float t = FindParameterAtDistance(arcTable, targetDistance);
            const Vector2 worldPosition = BezierFollower::Evaluate(fromNode, toNode, link, t);
            const float worldHeading = BezierFollower::Heading(fromNode, toNode, link, t);
            RobotProtocol::TrajectoryWaypoint waypoint = ToLocalWaypoint(
                worldPosition, worldHeading, origin, originHeading,
                options.millimetersPerMapUnit, options.cruiseSpeedMmPerSecond);

            if (segment == segmentCount)
            {
                waypoint.nodeID = toNode.m_Id;
                waypoint.flags |= RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY;
            }

            if (linkIndex + 1 == routeLinks.size() && segment == segmentCount)
            {
                waypoint.flags |= RobotProtocol::TRAJECTORY_FLAG_STOP |
                                  RobotProtocol::TRAJECTORY_FLAG_FINAL;
                waypoint.targetSpeedMmPerSecond = 0.0f;
            }

            if (!AppendWaypoint(trajectory, waypoint, outError)) return false;
        }

        if (linkIndex + 1 >= routeLinks.size()) continue;

        const MapNode& nextNode = *routeNodes[linkIndex + 2];
        const MapLink& nextLink = *routeLinks[linkIndex + 1];
        const float incomingHeading = BezierFollower::Heading(fromNode, toNode, link, 1.0f);
        const float outgoingHeading = BezierFollower::Heading(toNode, nextNode, nextLink, 0.0f);
        const float headingChange = std::abs(NormalizeAngle(outgoingHeading - incomingHeading));
        if (headingChange <= options.cornerStopThresholdRad) continue;

        RobotProtocol::TrajectoryWaypoint& corner = trajectory.waypoints.back();
        corner.flags |= RobotProtocol::TRAJECTORY_FLAG_STOP;
        corner.targetSpeedMmPerSecond = 0.0f;

        const Vector2 cornerPosition{ toNode.m_PosX, toNode.m_PosZ };
        RobotProtocol::TrajectoryWaypoint rotateWaypoint = ToLocalWaypoint(
            cornerPosition, outgoingHeading, origin, originHeading,
            options.millimetersPerMapUnit, 0.0f);
        rotateWaypoint.nodeID = 0;
        rotateWaypoint.flags = RobotProtocol::TRAJECTORY_FLAG_ROTATE_IN_PLACE;
        if (!AppendWaypoint(trajectory, rotateWaypoint, outError)) return false;
    }

    outTrajectory = std::move(trajectory);
    return true;
}
