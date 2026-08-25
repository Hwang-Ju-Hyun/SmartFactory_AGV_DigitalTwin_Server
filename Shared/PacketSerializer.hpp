#pragma once
#include "MemoryStream.hpp"
#include "Protocol.hpp"
#include "SharedPackets.hpp"
#include <vector>

namespace RobotProtocol
{
    struct RouteCommandPayload
    {
        uint32_t routeID = 0;
        std::vector<RouteNodeTime> nodes;
    };

    struct TrajectoryWaypoint
    {
        // Format v1 robot-local frame: +forward is the robot's trusted start
        // heading, +left is counter-clockwise 90 degrees from +forward.
        float forwardMm = 0.0f;
        float leftMm = 0.0f;
        float headingRad = 0.0f;
        float targetSpeedMmPerSecond = 0.0f;
        uint32_t nodeID = 0;
        uint8_t flags = TRAJECTORY_FLAG_NONE;
    };

    struct TrajectoryCommandPayload
    {
        uint32_t routeID = 0;
        uint8_t formatVersion = kTrajectoryFormatVersion;
        uint32_t startNodeID = 0;
        uint32_t finalNodeID = 0;
        float millimetersPerMapUnit = 0.0f;
        std::vector<TrajectoryWaypoint> waypoints;
    };

    bool IsKnownRobotPacketID(uint16_t rawPacketID);
    bool IsKnownVisionPacketID(uint16_t rawPacketID);
    bool IsKnownPacketID(uint16_t rawPacketID);
    bool PeekPacketID(InputMemoryStream& inStream, PacketID& outPacketID);

    void WritePacketBodyHeader(OutputMemoryStream& outStream, PacketID packetID, uint32_t agvID, uint32_t sequence);
    bool ReadPacketBodyHeader(InputMemoryStream& inStream, PacketBodyHeader& outHeader);

    void WriteHelloPayload(OutputMemoryStream& outStream, const HelloPayload& payload);
    bool ReadHelloPayload(InputMemoryStream& inStream, HelloPayload& outPayload);

    void WriteHelloAckPayload(OutputMemoryStream& outStream, const HelloAckPayload& payload);
    bool ReadHelloAckPayload(InputMemoryStream& inStream, HelloAckPayload& outPayload);

    void WriteStatusPayload(OutputMemoryStream& outStream, const StatusPayload& payload);
    bool ReadStatusPayload(InputMemoryStream& inStream, StatusPayload& outPayload);

    void WriteRouteCommandPayload(OutputMemoryStream& outStream, const RouteCommandPayload& payload);
    bool ReadRouteCommandPayload(InputMemoryStream& inStream, RouteCommandPayload& outPayload);

    bool WriteTrajectoryCommandPayload(OutputMemoryStream& outStream, const TrajectoryCommandPayload& payload);
    bool ReadTrajectoryCommandPayload(InputMemoryStream& inStream, TrajectoryCommandPayload& outPayload);

    void WriteArrivedPayload(OutputMemoryStream& outStream, const ArrivedPayload& payload);
    bool ReadArrivedPayload(InputMemoryStream& inStream, ArrivedPayload& outPayload);

    void WriteErrorPayload(OutputMemoryStream& outStream, const ErrorPayload& payload);
    bool ReadErrorPayload(InputMemoryStream& inStream, ErrorPayload& outPayload);

    void WriteTimePayload(OutputMemoryStream& outStream, const TimePayload& payload);
    bool ReadTimePayload(InputMemoryStream& inStream, TimePayload& outPayload);

    bool WriteVisionHelloPayload(OutputMemoryStream& outStream, const VisionHelloPayload& payload);
    bool ReadVisionHelloPayload(InputMemoryStream& inStream, VisionHelloPayload& outPayload);

    bool WriteVisionHelloAckPayload(OutputMemoryStream& outStream, const VisionHelloAckPayload& payload);
    bool ReadVisionHelloAckPayload(InputMemoryStream& inStream, VisionHelloAckPayload& outPayload);

    bool WriteVisionObservationPayload(OutputMemoryStream& outStream, const VisionObservationPayload& payload);
    bool ReadVisionObservationPayload(InputMemoryStream& inStream, VisionObservationPayload& outPayload);
}
