#include "MemoryStream.hpp"
#include "PacketSerializer.hpp"
#include "ESP32RobotController.hpp"
#include "PhysicalFleetHeadingAnchor.hpp"
#include "TrajectoryBuilder.hpp"
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
    int g_Failures = 0;

    void Check(bool condition, const std::string& message)
    {
        if (condition) return;
        ++g_Failures;
        std::cerr << "[TrajectorySmokeTest] FAIL: " << message << "\n";
    }

    bool Near(float lhs, float rhs, float tolerance = 0.05f)
    {
        return std::abs(lhs - rhs) <= tolerance;
    }

    bool HasFlag(const RobotProtocol::TrajectoryWaypoint& waypoint, uint8_t flag)
    {
        return (waypoint.flags & flag) != 0;
    }

    MapNode Node(uint32_t id, float x, float z)
    {
        MapNode node{};
        node.m_Id = id;
        node.m_PosX = x;
        node.m_PosZ = z;
        node.type = 0;
        return node;
    }

    MapLink Line(uint32_t id, uint32_t from, uint32_t to)
    {
        MapLink link{};
        link.m_Id = id;
        link.m_FromNodeID = from;
        link.m_ToNodeID = to;
        link.m_Type = 0;
        return link;
    }

    MapLink Curve(uint32_t id, uint32_t from, uint32_t to,
                  float cx1, float cz1, float cx2, float cz2)
    {
        MapLink link = Line(id, from, to);
        link.m_Type = 1;
        link.m_CX1 = cx1;
        link.m_CZ1 = cz1;
        link.m_CX2 = cx2;
        link.m_CZ2 = cz2;
        return link;
    }

    void TestMixedLineBezierLine()
    {
        const std::unordered_map<uint32_t, MapNode> nodes = {
            { 5, Node(5, 0.0f, -8.0f) },
            { 3, Node(3, 0.0f, -14.0f) },
            { 1, Node(1, 4.0f, -16.0f) },
            { 2, Node(2, 16.0f, -16.0f) }
        };
        const std::vector<MapLink> links = {
            Line(1, 5, 3),
            Curve(2, 3, 1, 0.0f, -16.0f, 2.0f, -16.0f),
            Line(3, 1, 2)
        };

        TrajectoryBuildOptions options;
        options.hasTrustedStartHeading = true;
        options.startHeadingRad = -1.5707963f;
        options.millimetersPerMapUnit = 10.0f;
        options.spacingMm = 20.0f;
        options.cruiseSpeedMmPerSecond = 120.0f;
        options.cornerStopThresholdRad = 0.35f;

        RobotProtocol::TrajectoryCommandPayload trajectory;
        std::string error;
        const bool built = TrajectoryBuilder::BuildFromGeometry(
            { 5, 3, 1, 2 }, nodes, links, 77, options, trajectory, error);
        Check(built, "mixed LINE/BEZIER/LINE build failed: " + error);
        if (!built) return;

        Check(trajectory.routeID == 77, "routeID was not preserved");
        Check(trajectory.formatVersion == RobotProtocol::kTrajectoryFormatVersion,
              "trajectory format version is wrong");
        Check(trajectory.startNodeID == 5 && trajectory.finalNodeID == 2,
              "start/final node IDs are wrong");
        Check(!trajectory.waypoints.empty(), "trajectory has no waypoints");
        Check(trajectory.waypoints.size() <= RobotProtocol::kMaxTrajectoryWaypoints,
              "trajectory exceeded protocol waypoint limit");

        const auto& first = trajectory.waypoints.front();
        const auto& last = trajectory.waypoints.back();
        Check(Near(first.forwardMm, 0.0f) && Near(first.leftMm, 0.0f),
              "first waypoint is not robot-local origin");
        Check(HasFlag(first, RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY),
              "first waypoint lacks NODE_BOUNDARY");
        Check(Near(last.forwardMm, 80.0f) && Near(last.leftMm, 160.0f),
              "final local coordinate is wrong");
        Check(last.nodeID == 2, "final waypoint node ID is wrong");
        Check(HasFlag(last, RobotProtocol::TRAJECTORY_FLAG_FINAL) &&
              HasFlag(last, RobotProtocol::TRAJECTORY_FLAG_STOP),
              "final waypoint lacks FINAL/STOP");

        bool sawCurveInterior = false;
        bool sawUnexpectedRotate = false;
        for (const auto& waypoint : trajectory.waypoints)
        {
            if (waypoint.forwardMm > 60.0f && waypoint.forwardMm < 80.0f &&
                waypoint.leftMm > 0.0f && waypoint.leftMm < 40.0f)
            {
                sawCurveInterior = true;
            }
            sawUnexpectedRotate = sawUnexpectedRotate ||
                HasFlag(waypoint, RobotProtocol::TRAJECTORY_FLAG_ROTATE_IN_PLACE);
        }
        Check(sawCurveInterior, "Bezier interior samples were not generated");
        Check(!sawUnexpectedRotate,
              "tangent-continuous LINE/BEZIER path inserted an unnecessary rotation");

        OutputMemoryStream encoded;
        Check(RobotProtocol::WriteTrajectoryCommandPayload(encoded, trajectory),
              "trajectory serializer rejected a valid payload");
        InputMemoryStream encodedInput(
            const_cast<char*>(encoded.GetBuffer()), encoded.GetLength());
        RobotProtocol::TrajectoryCommandPayload decoded;
        Check(RobotProtocol::ReadTrajectoryCommandPayload(encodedInput, decoded),
              "trajectory serializer round-trip failed");
        Check(decoded.waypoints.size() == trajectory.waypoints.size(),
              "round-trip waypoint count changed");
        Check(decoded.routeID == trajectory.routeID &&
              decoded.finalNodeID == trajectory.finalNodeID,
              "round-trip identifiers changed");

        TrajectoryBuildOptions tooDense = options;
        tooDense.spacingMm = 1.0f;
        RobotProtocol::TrajectoryCommandPayload rejected;
        error.clear();
        Check(!TrajectoryBuilder::BuildFromGeometry(
                  { 5, 3, 1, 2 }, nodes, links, 78, tooDense, rejected, error),
              "builder silently accepted more than 64 waypoints");
        Check(rejected.waypoints.empty(),
              "failed build leaked a partial trajectory");
    }

    void TestSharpCornerRotation()
    {
        const std::unordered_map<uint32_t, MapNode> nodes = {
            { 10, Node(10, 0.0f, 0.0f) },
            { 11, Node(11, 10.0f, 0.0f) },
            { 12, Node(12, 10.0f, 10.0f) }
        };
        const std::vector<MapLink> links = {
            Line(10, 10, 11),
            Line(11, 11, 12)
        };

        TrajectoryBuildOptions options;
        options.hasTrustedStartHeading = true;
        options.startHeadingRad = 0.0f;
        options.millimetersPerMapUnit = 10.0f;
        options.spacingMm = 50.0f;
        options.cruiseSpeedMmPerSecond = 100.0f;
        options.cornerStopThresholdRad = 0.35f;

        RobotProtocol::TrajectoryCommandPayload trajectory;
        std::string error;
        const bool built = TrajectoryBuilder::BuildFromGeometry(
            { 10, 11, 12 }, nodes, links, 88, options, trajectory, error);
        Check(built, "sharp-corner build failed: " + error);
        if (!built) return;

        bool sawStoppedCorner = false;
        bool sawRotation = false;
        for (const auto& waypoint : trajectory.waypoints)
        {
            if (waypoint.nodeID == 11)
                sawStoppedCorner = sawStoppedCorner ||
                    HasFlag(waypoint, RobotProtocol::TRAJECTORY_FLAG_STOP);
            if (HasFlag(waypoint, RobotProtocol::TRAJECTORY_FLAG_ROTATE_IN_PLACE))
            {
                sawRotation = Near(waypoint.forwardMm, 100.0f) &&
                              Near(waypoint.leftMm, 0.0f) &&
                              Near(waypoint.headingRad, 1.5707963f) &&
                              waypoint.nodeID == 0 &&
                              !HasFlag(waypoint,
                                  RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY);
            }
        }
        Check(sawStoppedCorner, "sharp corner did not request STOP");
        Check(sawRotation, "sharp corner did not insert the expected in-place rotation");
    }

    void TestMalformedWaypointCount()
    {
        OutputMemoryStream malformed;
        malformed.Write(static_cast<uint32_t>(1));
        malformed.Write(RobotProtocol::kTrajectoryFormatVersion);
        malformed.Write(static_cast<uint16_t>(RobotProtocol::kMaxTrajectoryWaypoints + 1));
        malformed.Write(static_cast<uint32_t>(10));
        malformed.Write(static_cast<uint32_t>(12));
        malformed.Write(10.0f);

        InputMemoryStream input(
            const_cast<char*>(malformed.GetBuffer()), malformed.GetLength());
        RobotProtocol::TrajectoryCommandPayload decoded;
        Check(!RobotProtocol::ReadTrajectoryCommandPayload(input, decoded),
              "deserializer accepted a waypoint count above the protocol limit");
    }

    void TestInitialHeadingAndWireGuards()
    {
        const std::unordered_map<uint32_t, MapNode> nodes = {
            { 1, Node(1, 0.0f, 0.0f) },
            { 2, Node(2, 10.0f, 0.0f) }
        };
        const std::vector<MapLink> links = { Line(1, 1, 2) };
        TrajectoryBuildOptions options;
        options.hasTrustedStartHeading = true;
        options.startHeadingRad = 1.5707963f;
        options.millimetersPerMapUnit = 10.0f;
        options.spacingMm = 50.0f;

        RobotProtocol::TrajectoryCommandPayload trajectory;
        std::string error;
        Check(TrajectoryBuilder::BuildFromGeometry(
                  { 1, 2 }, nodes, links, 99, options, trajectory, error),
              "trusted initial heading build failed: " + error);
        Check(trajectory.waypoints.size() >= 3,
              "initial rotation waypoint was not inserted");
        if (trajectory.waypoints.size() >= 2)
        {
            const auto& rotate = trajectory.waypoints[1];
            Check(HasFlag(rotate, RobotProtocol::TRAJECTORY_FLAG_ROTATE_IN_PLACE) &&
                  rotate.nodeID == 0 &&
                  !HasFlag(rotate, RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY) &&
                  Near(rotate.headingRad, -1.5707963f),
                  "initial rotation did not preserve trusted start heading");
        }

        OutputMemoryStream encoded;
        Check(RobotProtocol::WriteTrajectoryCommandPayload(encoded, trajectory),
              "valid versioned trajectory failed to serialize");
        std::vector<char> wrongVersion(encoded.GetBuffer(),
                                       encoded.GetBuffer() + encoded.GetLength());
        wrongVersion[sizeof(uint32_t)] = 2;
        InputMemoryStream wrongVersionInput(wrongVersion.data(), wrongVersion.size());
        RobotProtocol::TrajectoryCommandPayload decoded;
        Check(!RobotProtocol::ReadTrajectoryCommandPayload(wrongVersionInput, decoded),
              "deserializer accepted an unknown trajectory format version");

        std::vector<char> trailing(encoded.GetBuffer(),
                                   encoded.GetBuffer() + encoded.GetLength());
        trailing.push_back(static_cast<char>(0x7f));
        InputMemoryStream trailingInput(trailing.data(), trailing.size());
        Check(!RobotProtocol::ReadTrajectoryCommandPayload(trailingInput, decoded),
              "deserializer accepted trailing trajectory bytes");

        TrajectoryBuildOptions missingHeading = options;
        missingHeading.hasTrustedStartHeading = false;
        RobotProtocol::TrajectoryCommandPayload rejected;
        error.clear();
        Check(!TrajectoryBuilder::BuildFromGeometry(
                  { 1, 2 }, nodes, links, 100, missingHeading, rejected, error),
              "builder accepted a route without a trusted start heading");
    }

    void TestPhysicalFleetHeadingToleranceContract()
    {
        constexpr float kReproducedStartHeadingRad = -0.100539568f; // -5.7605 degrees
        const std::unordered_map<uint32_t, MapNode> nodes = {
            { 3, Node(3, 0.0f, 0.0f) },
            { 4, Node(4, 7.0f, 0.0f) }
        };
        const std::vector<MapLink> links = { Line(3, 3, 4) };

        ESP32TrajectoryExecutionConfig physicalFleetConfig;
        Check(Near(physicalFleetConfig.initialHeadingToleranceRad,
                   PhysicalFleetCorrectionPolicy::kHeadingToleranceRad,
                   0.000001f),
              "physical-fleet trajectory and correction heading tolerances diverged");

        TrajectoryBuildOptions options;
        options.hasTrustedStartHeading = true;
        options.startHeadingRad = kReproducedStartHeadingRad;
        options.millimetersPerMapUnit = 50.0f;
        options.spacingMm = 50.0f;
        options.cruiseSpeedMmPerSecond = 80.0f;
        options.cornerStopThresholdRad =
            physicalFleetConfig.initialHeadingToleranceRad;
        options.lineEndpointOnly = true;
        options.stopAtEveryNodeBoundary = true;

        RobotProtocol::TrajectoryCommandPayload trajectory;
        std::string error;
        const bool built = TrajectoryBuilder::BuildFromGeometry(
            { 3, 4 }, nodes, links, 103, options, trajectory, error);
        Check(built, "-5.7605 degree physical-fleet edge build failed: " + error);
        if (!built) return;

        bool sawRotation = false;
        for (const auto& waypoint : trajectory.waypoints)
        {
            sawRotation = sawRotation ||
                HasFlag(waypoint, RobotProtocol::TRAJECTORY_FLAG_ROTATE_IN_PLACE);
        }
        Check(trajectory.waypoints.size() == 2,
              "-5.7605 degree one-edge trajectory did not stay endpoint-only");
        Check(!sawRotation,
              "heading inside the physical-fleet 10 degree contract inserted a rotation");
        Check(std::abs(trajectory.waypoints.back().headingRad) <=
                  physicalFleetConfig.initialHeadingToleranceRad,
              "straight edge exceeded the physical-fleet heading acceptance contract");

        TrajectoryBuildOptions alignedAfterCorrection = options;
        alignedAfterCorrection.startHeadingRad =
            8.0f * (3.14159265358979323846f / 180.0f);
        RobotProtocol::TrajectoryCommandPayload alignedTrajectory;
        error.clear();
        Check(TrajectoryBuilder::BuildFromGeometry(
                  { 3, 4 }, nodes, links, 106, alignedAfterCorrection,
                  alignedTrajectory, error),
              "post-alignment trajectory build failed: " + error);
        bool sawPostAlignmentRotation = false;
        for (const auto& waypoint : alignedTrajectory.waypoints)
        {
            sawPostAlignmentRotation = sawPostAlignmentRotation ||
                HasFlag(waypoint,
                        RobotProtocol::TRAJECTORY_FLAG_ROTATE_IN_PLACE);
        }
        Check(!sawPostAlignmentRotation,
              "trajectory repeated a turn after pre-departure alignment");

        TrajectoryBuildOptions nominalTurn = options;
        nominalTurn.startHeadingRad = -1.5707963267948966f;
        RobotProtocol::TrajectoryCommandPayload turnedTrajectory;
        error.clear();
        const bool turnedBuilt = TrajectoryBuilder::BuildFromGeometry(
            { 3, 4 }, nodes, links, 104, nominalTurn, turnedTrajectory, error);
        Check(turnedBuilt, "90 degree physical-fleet edge build failed: " + error);
        if (!turnedBuilt) return;

        bool sawNominalRotation = false;
        for (const auto& waypoint : turnedTrajectory.waypoints)
        {
            sawNominalRotation = sawNominalRotation ||
                HasFlag(waypoint, RobotProtocol::TRAJECTORY_FLAG_ROTATE_IN_PLACE);
        }
        Check(sawNominalRotation,
              "90 degree physical-fleet heading change lost its rotation waypoint");
    }

    void TestPhysicalFleetVisionHeadingAnchorAndWrapping()
    {
        constexpr float degreesToRadians =
            3.14159265358979323846f / 180.0f;
        const auto vision = ResolvePhysicalFleetHeading(
            90.0f * degreesToRadians,
            PhysicalFleetHeadingAnchor{
                80.4681f * degreesToRadians,
                14200
            });
        Check(vision.usedVision && vision.visionSequence == 14200,
              "accepted Vision heading source or sequence was lost");
        Check(Near(vision.headingRad, 80.4681f * degreesToRadians, 0.0001f),
              "accepted Vision heading was replaced by nominal heading");

        const auto invalidSequence = ResolvePhysicalFleetHeading(
            90.0f * degreesToRadians,
            PhysicalFleetHeadingAnchor{80.0f * degreesToRadians, 0});
        Check(!invalidSequence.usedVision &&
                  Near(invalidSequence.headingRad,
                       90.0f * degreesToRadians,
                       0.0001f),
              "invalid Vision sequence did not fall back to nominal heading");

        const auto invalidHeading = ResolvePhysicalFleetHeading(
            -90.0f * degreesToRadians,
            PhysicalFleetHeadingAnchor{
                std::numeric_limits<float>::quiet_NaN(),
                1
            });
        Check(!invalidHeading.usedVision &&
                  Near(invalidHeading.headingRad,
                       -90.0f * degreesToRadians,
                       0.0001f),
              "non-finite Vision heading did not fall back to nominal heading");

        const std::unordered_map<uint32_t, MapNode> nodes = {
            { 11, Node(11, 0.0f, 0.0f) },
            { 12, Node(12,
                       -std::cos(1.0f * degreesToRadians),
                       -std::sin(1.0f * degreesToRadians)) }
        };
        const std::vector<MapLink> links = { Line(11, 11, 12) };
        TrajectoryBuildOptions options;
        options.hasTrustedStartHeading = true;
        options.startHeadingRad = 179.0f * degreesToRadians;
        options.millimetersPerMapUnit = 50.0f;
        options.spacingMm = 50.0f;
        options.cornerStopThresholdRad =
            PhysicalFleetCorrectionPolicy::kHeadingToleranceRad;
        options.lineEndpointOnly = true;
        options.stopAtEveryNodeBoundary = true;

        RobotProtocol::TrajectoryCommandPayload trajectory;
        std::string error;
        Check(TrajectoryBuilder::BuildFromGeometry(
                  { 11, 12 }, nodes, links, 105, options, trajectory, error),
              "wrapped Vision heading trajectory failed: " + error);
        bool sawRotation = false;
        for (const auto& waypoint : trajectory.waypoints)
        {
            sawRotation = sawRotation || HasFlag(
                waypoint,
                RobotProtocol::TRAJECTORY_FLAG_ROTATE_IN_PLACE);
        }
        Check(!sawRotation,
              "wrapped 179 to -179 degree heading inserted a full rotation");

        PhysicalFleetVisionHeadingCandidate candidate;
        candidate.agvID = 1;
        candidate.nodeID = 11;
        candidate.sourceID = 7;
        candidate.sessionID = 99;
        candidate.visionSequence = 14200;
        candidate.receivedAtMilliseconds = 900;
        candidate.headingRad = 80.4681f * degreesToRadians;
        candidate.calibrationID = "e7c58f032c843335";
        candidate.measuredAndVerified = true;
        Check(ValidatePhysicalFleetVisionHeading(
                  candidate,
                  1,
                  11,
                  "e7c58f032c843335",
                  7,
                  99,
                  1000,
                  200).has_value(),
              "fresh accepted Vision heading was rejected");

        Check(!ValidatePhysicalFleetVisionHeading(
                   candidate,
                   1,
                   11,
                   "e7c58f032c843335",
                   7,
                   99,
                   1000,
                   200,
                   candidate.visionSequence).has_value(),
              "Vision heading was accepted without a newer sequence");

        auto rejected = candidate;
        rejected.measuredAndVerified = false;
        Check(!ValidatePhysicalFleetVisionHeading(
                   rejected, 1, 11, "e7c58f032c843335", 7, 99, 1000, 200)
                   .has_value(),
              "HELD/LOST or unverified heading was accepted for departure");
        rejected = candidate;
        Check(!ValidatePhysicalFleetVisionHeading(
                   rejected, 1, 11, "e7c58f032c843335", 0, 0, 1000, 200)
                   .has_value(),
              "disconnected Vision session released departure");

        rejected = candidate;
        rejected.nodeID = 12;
        Check(!ValidatePhysicalFleetVisionHeading(
                   rejected, 1, 11, "e7c58f032c843335", 7, 99, 1000, 200)
                   .has_value(),
              "Vision heading from another arrival node was accepted");
        rejected = candidate;
        rejected.calibrationID = "other-calibration";
        Check(!ValidatePhysicalFleetVisionHeading(
                   rejected, 1, 11, "e7c58f032c843335", 7, 99, 1000, 200)
                   .has_value(),
              "Vision heading with another calibration was accepted");
        rejected = candidate;
        Check(!ValidatePhysicalFleetVisionHeading(
                   rejected, 1, 11, "e7c58f032c843335", 7, 100, 1000, 200)
                   .has_value(),
              "Vision heading from an old session was accepted");
        rejected = candidate;
        rejected.receivedAtMilliseconds = 799;
        Check(!ValidatePhysicalFleetVisionHeading(
                   rejected, 1, 11, "e7c58f032c843335", 7, 99, 1000, 200)
                   .has_value(),
              "stale Vision heading was accepted");
    }

    void TestPhysicalFleetInitialEdgeTrajectoryDispatch()
    {
        constexpr float pi = 3.14159265358979323846f;
        const std::unordered_map<uint32_t, MapNode> nodes = {
            { 1, Node(1, 0.0f, 0.0f) },
            { 2, Node(2, 7.0f, 0.0f) },
            { 6, Node(6, 0.0f, 7.0f) }
        };
        const std::vector<MapLink> links = {
            Line(1, 1, 2),
            Line(2, 1, 6)
        };
        TrajectoryBuildOptions options;
        options.hasTrustedStartHeading = true;
        options.startHeadingRad = 0.0f;
        options.millimetersPerMapUnit = 50.0f;
        options.spacingMm = 50.0f;
        options.cruiseSpeedMmPerSecond = 80.0f;
        options.cornerStopThresholdRad =
            PhysicalFleetCorrectionPolicy::kHeadingToleranceRad;
        options.lineEndpointOnly = true;
        options.stopAtEveryNodeBoundary = true;

        RobotProtocol::TrajectoryCommandPayload east;
        std::string error;
        Check(TrajectoryBuilder::BuildFromGeometry(
                  { 1, 2 }, nodes, links, 201, options, east, error),
              "initial 1->2 trajectory failed: " + error);
        Check(east.routeID == 201 && east.waypoints.size() == 2,
              "initial 1->2 route context or endpoint count changed");
        bool eastRotates = false;
        for (const auto& waypoint : east.waypoints)
            eastRotates = eastRotates || HasFlag(
                waypoint, RobotProtocol::TRAJECTORY_FLAG_ROTATE_IN_PLACE);
        Check(!eastRotates, "aligned initial 1->2 edge inserted a turn");

        RobotProtocol::TrajectoryCommandPayload north;
        error.clear();
        Check(TrajectoryBuilder::BuildFromGeometry(
                  { 1, 6 }, nodes, links, 202, options, north, error),
              "initial 1->6 trajectory failed: " + error);
        Check(north.routeID == 202 && north.startNodeID == 1 &&
                  north.finalNodeID == 6 && north.waypoints.size() == 3,
              "initial 1->6 route context or waypoint count changed");
        if (north.waypoints.size() == 3)
        {
            const auto& rotate = north.waypoints[1];
            Check(HasFlag(
                      rotate,
                      RobotProtocol::TRAJECTORY_FLAG_ROTATE_IN_PLACE) &&
                      rotate.nodeID == 0 &&
                      Near(rotate.headingRad, pi / 2.0f),
                  "initial 1->6 did not encode a 90 degree CCW turn");
            Check(HasFlag(
                      north.waypoints.back(),
                      RobotProtocol::TRAJECTORY_FLAG_FINAL),
                  "initial 1->6 forward endpoint is not final");
        }
    }

    void TestOptionalHelloCapabilities()
    {
        OutputMemoryStream legacyHello;
        legacyHello.Write(RobotProtocol::kProtocolVersion);
        legacyHello.Write(static_cast<uint8_t>(RobotProtocol::ClientType::ESP32));
        legacyHello.Write(static_cast<uint32_t>(1));

        InputMemoryStream legacyInput(
            const_cast<char*>(legacyHello.GetBuffer()), legacyHello.GetLength());
        RobotProtocol::HelloPayload legacyDecoded;
        Check(RobotProtocol::ReadHelloPayload(legacyInput, legacyDecoded),
              "legacy v1 HELLO without capabilities was rejected");
        Check(legacyDecoded.capabilities == RobotProtocol::CAPABILITY_NONE,
              "legacy v1 HELLO did not default capabilities to zero");

        RobotProtocol::HelloPayload capableHello;
        capableHello.protocolVersion = RobotProtocol::kProtocolVersion;
        capableHello.clientType = RobotProtocol::ClientType::FAKE_ROBOT;
        capableHello.requestedAgvID = 1;
        capableHello.capabilities = RobotProtocol::CAPABILITY_TRAJECTORY_PREVIEW;
        OutputMemoryStream capableEncoded;
        RobotProtocol::WriteHelloPayload(capableEncoded, capableHello);

        InputMemoryStream capableInput(
            const_cast<char*>(capableEncoded.GetBuffer()), capableEncoded.GetLength());
        RobotProtocol::HelloPayload capableDecoded;
        Check(RobotProtocol::ReadHelloPayload(capableInput, capableDecoded),
              "capability-extended HELLO was rejected");
        Check((capableDecoded.capabilities &
               RobotProtocol::CAPABILITY_TRAJECTORY_PREVIEW) != 0,
              "trajectory preview capability was lost during HELLO round-trip");

        RobotProtocol::HelloPayload plainHello;
        plainHello.protocolVersion = RobotProtocol::kProtocolVersion;
        plainHello.clientType = RobotProtocol::ClientType::ESP32;
        plainHello.requestedAgvID = 1;
        plainHello.capabilities = RobotProtocol::CAPABILITY_NONE;
        OutputMemoryStream plainEncoded;
        RobotProtocol::WriteHelloPayload(plainEncoded, plainHello);
        Check(plainEncoded.GetLength() == 7,
              "capability-free HELLO no longer preserves the legacy 7-byte payload");

        OutputMemoryStream malformedHello;
        malformedHello.Write(RobotProtocol::kProtocolVersion);
        malformedHello.Write(static_cast<uint8_t>(RobotProtocol::ClientType::ESP32));
        malformedHello.Write(static_cast<uint32_t>(1));
        malformedHello.Write(static_cast<uint8_t>(0xff));
        InputMemoryStream malformedInput(
            const_cast<char*>(malformedHello.GetBuffer()), malformedHello.GetLength());
        RobotProtocol::HelloPayload malformedDecoded;
        Check(!RobotProtocol::ReadHelloPayload(malformedInput, malformedDecoded),
              "HELLO parser accepted a partial capability field");
    }

    void TestMaximumTrajectoryWireSize()
    {
        RobotProtocol::TrajectoryCommandPayload trajectory;
        trajectory.routeID = 123;
        trajectory.formatVersion = RobotProtocol::kTrajectoryFormatVersion;
        trajectory.startNodeID = 1;
        trajectory.finalNodeID = 2;
        trajectory.millimetersPerMapUnit = 10.0f;
        trajectory.waypoints.resize(RobotProtocol::kMaxTrajectoryWaypoints);

        OutputMemoryStream encoded;
        Check(RobotProtocol::WriteTrajectoryCommandPayload(encoded, trajectory),
              "serializer rejected exactly 64 waypoints");
        constexpr uint32_t expectedPayloadBytes =
            19u + RobotProtocol::kMaxTrajectoryWaypoints * 21u;
        Check(encoded.GetLength() == expectedPayloadBytes,
              "maximum trajectory payload is not 1363 bytes");

        trajectory.waypoints.push_back(RobotProtocol::TrajectoryWaypoint{});
        OutputMemoryStream tooLarge;
        Check(!RobotProtocol::WriteTrajectoryCommandPayload(tooLarge, trajectory),
              "serializer accepted 65 waypoints");
    }

    void TestNodeCorrectionWireFormat()
    {
        RobotProtocol::NodeCorrectionCommandPayload command;
        command.routeID = 7;
        command.nodeID = 8;
        command.commandID = 9;
        command.action = RobotProtocol::NodeCorrectionAction::TURN_CCW;
        command.magnitude = 0.25f;

        OutputMemoryStream encodedCommand;
        Check(RobotProtocol::WriteNodeCorrectionCommandPayload(
                  encodedCommand, command),
              "node correction command serialization failed");
        Check(encodedCommand.GetLength() == 17,
              "node correction command wire size changed");
        InputMemoryStream commandInput(
            const_cast<char*>(encodedCommand.GetBuffer()),
            encodedCommand.GetLength());
        RobotProtocol::NodeCorrectionCommandPayload decodedCommand;
        Check(RobotProtocol::ReadNodeCorrectionCommandPayload(
                  commandInput, decodedCommand),
              "node correction command round-trip failed");
        Check(decodedCommand.routeID == command.routeID &&
                  decodedCommand.nodeID == command.nodeID &&
                  decodedCommand.commandID == command.commandID &&
                  decodedCommand.action == command.action &&
                  Near(decodedCommand.magnitude, command.magnitude),
              "node correction command fields changed during round-trip");

        RobotProtocol::NodeCorrectionReportPayload report;
        report.routeID = command.routeID;
        report.nodeID = command.nodeID;
        report.commandID = command.commandID;
        report.result = RobotProtocol::NodeCorrectionResult::COMPLETED;
        report.detail = 42;
        OutputMemoryStream encodedReport;
        Check(RobotProtocol::WriteNodeCorrectionReportPayload(
                  encodedReport, report),
              "node correction report serialization failed");
        Check(encodedReport.GetLength() == 17,
              "node correction report wire size changed");
        InputMemoryStream reportInput(
            const_cast<char*>(encodedReport.GetBuffer()),
            encodedReport.GetLength());
        RobotProtocol::NodeCorrectionReportPayload decodedReport;
        Check(RobotProtocol::ReadNodeCorrectionReportPayload(
                  reportInput, decodedReport),
              "node correction report round-trip failed");
        Check(decodedReport.routeID == report.routeID &&
                  decodedReport.nodeID == report.nodeID &&
                  decodedReport.commandID == report.commandID &&
                  decodedReport.result == report.result &&
                  decodedReport.detail == report.detail,
              "node correction report fields changed during round-trip");

        command.magnitude = 0.0f;
        OutputMemoryStream invalidCommand;
        Check(!RobotProtocol::WriteNodeCorrectionCommandPayload(
                  invalidCommand, command),
              "node correction serializer accepted zero magnitude");
    }
}

int main()
{
    TestMixedLineBezierLine();
    TestSharpCornerRotation();
    TestMalformedWaypointCount();
    TestInitialHeadingAndWireGuards();
    TestPhysicalFleetHeadingToleranceContract();
    TestPhysicalFleetVisionHeadingAnchorAndWrapping();
    TestPhysicalFleetInitialEdgeTrajectoryDispatch();
    TestOptionalHelloCapabilities();
    TestMaximumTrajectoryWireSize();
    TestNodeCorrectionWireFormat();

    if (g_Failures != 0)
    {
        std::cerr << "[TrajectorySmokeTest] FAILED checks=" << g_Failures << "\n";
        return 1;
    }

    std::cout << "[TrajectorySmokeTest] PASS mixed LINE/BEZIER/LINE, physical-fleet heading contract, corner rotation, serializer limits\n";
    return 0;
}
