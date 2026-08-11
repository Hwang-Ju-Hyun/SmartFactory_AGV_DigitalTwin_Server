#include "PacketSerializer.hpp"
#include <cmath>
#include <cstring>
#include <utility>

namespace RobotProtocol
{
    namespace
    {
        bool HasBytes(InputMemoryStream& inStream, uint32_t byteCount)
        {
            return inStream.GetRemainDataSize() >= byteCount;
        }

        bool ReadUInt8(InputMemoryStream& inStream, uint8_t& value)
        {
            if (!HasBytes(inStream, sizeof(uint8_t))) return false;
            inStream.Read(value);
            return true;
        }

        bool ReadUInt16(InputMemoryStream& inStream, uint16_t& value)
        {
            if (!HasBytes(inStream, sizeof(uint16_t))) return false;
            inStream.Read(value);
            return true;
        }

        bool ReadUInt32(InputMemoryStream& inStream, uint32_t& value)
        {
            if (!HasBytes(inStream, sizeof(uint32_t))) return false;
            inStream.Read(value);
            return true;
        }

        bool ReadFloat(InputMemoryStream& inStream, float& value)
        {
            if (!HasBytes(inStream, sizeof(float))) return false;
            inStream.Read(value);
            return true;
        }
    }

    bool IsKnownPacketID(uint16_t rawPacketID)
    {
        switch (static_cast<PacketID>(rawPacketID))
        {
        case PacketID::ROUTE_COMMAND:
        case PacketID::CANCEL_ROUTE:
        case PacketID::TRAJECTORY_COMMAND:
        case PacketID::STATUS:
        case PacketID::ARRIVED:
        case PacketID::PING:
        case PacketID::PONG:
        case PacketID::HELLO:
        case PacketID::HELLO_ACK:
        case PacketID::ERROR_PACKET:
        case PacketID::EMERGENCY_STOP:
            return true;
        default:
            return false;
        }
    }

    bool PeekPacketID(InputMemoryStream& inStream, PacketID& outPacketID)
    {
        if (!HasBytes(inStream, sizeof(uint16_t))) return false;

        uint16_t rawPacketID = 0;
        std::memcpy(&rawPacketID, inStream.GetBuffer(), sizeof(uint16_t));
        if (!IsKnownPacketID(rawPacketID)) return false;

        outPacketID = static_cast<PacketID>(rawPacketID);
        return true;
    }

    void WritePacketBodyHeader(OutputMemoryStream& outStream, PacketID packetID, uint32_t agvID, uint32_t sequence)
    {
        outStream.Write(static_cast<uint16_t>(packetID));
        outStream.Write(agvID);
        outStream.Write(sequence);
    }

    bool ReadPacketBodyHeader(InputMemoryStream& inStream, PacketBodyHeader& outHeader)
    {
        uint16_t packetID = 0;
        if (!ReadUInt16(inStream, packetID)) return false;
        if (!ReadUInt32(inStream, outHeader.agvID)) return false;
        if (!ReadUInt32(inStream, outHeader.sequence)) return false;

        if (!IsKnownPacketID(packetID)) return false;
        outHeader.packetID = packetID;
        return true;
    }

    void WriteHelloPayload(OutputMemoryStream& outStream, const HelloPayload& payload)
    {
        outStream.Write(payload.protocolVersion);
        outStream.Write(static_cast<uint8_t>(payload.clientType));
        outStream.Write(payload.requestedAgvID);
        if (payload.capabilities != CAPABILITY_NONE)
            outStream.Write(payload.capabilities);
    }

    bool ReadHelloPayload(InputMemoryStream& inStream, HelloPayload& outPayload)
    {
        uint8_t rawClientType = 0;
        outPayload.capabilities = CAPABILITY_NONE;
        if (!ReadUInt16(inStream, outPayload.protocolVersion)) return false;
        if (!ReadUInt8(inStream, rawClientType)) return false;
        if (!ReadUInt32(inStream, outPayload.requestedAgvID)) return false;

        // Protocol v1 originally ended after requestedAgvID. Reading the
        // capability mask only when present keeps deployed ESP32 clients
        // wire-compatible.
        const uint32_t optionalBytes = inStream.GetRemainDataSize();
        if (optionalBytes != 0 && optionalBytes != sizeof(uint32_t))
            return false;
        if (optionalBytes == sizeof(uint32_t) &&
            !ReadUInt32(inStream, outPayload.capabilities))
        {
            return false;
        }

        outPayload.clientType = static_cast<ClientType>(rawClientType);
        return true;
    }

    void WriteHelloAckPayload(OutputMemoryStream& outStream, const HelloAckPayload& payload)
    {
        outStream.Write(payload.protocolVersion);
        outStream.Write(payload.accepted);
        outStream.Write(payload.assignedAgvID);
        outStream.Write(static_cast<uint16_t>(payload.errorCode));
    }

    bool ReadHelloAckPayload(InputMemoryStream& inStream, HelloAckPayload& outPayload)
    {
        uint16_t rawErrorCode = 0;
        if (!ReadUInt16(inStream, outPayload.protocolVersion)) return false;
        if (!ReadUInt8(inStream, outPayload.accepted)) return false;
        if (!ReadUInt32(inStream, outPayload.assignedAgvID)) return false;
        if (!ReadUInt16(inStream, rawErrorCode)) return false;

        outPayload.errorCode = static_cast<ErrorCode>(rawErrorCode);
        return true;
    }

    void WriteStatusPayload(OutputMemoryStream& outStream, const StatusPayload& payload)
    {
        outStream.Write(payload.currentNodeID);
        outStream.Write(payload.currentLinkID);
        outStream.Write(payload.progress);
        outStream.Write(payload.x);
        outStream.Write(payload.z);
        outStream.Write(payload.heading);
        outStream.Write(payload.velocity);
        outStream.Write(payload.battery);
        outStream.Write(static_cast<uint8_t>(payload.state));
    }

    bool ReadStatusPayload(InputMemoryStream& inStream, StatusPayload& outPayload)
    {
        uint8_t rawState = 0;
        if (!ReadUInt32(inStream, outPayload.currentNodeID)) return false;
        if (!ReadUInt32(inStream, outPayload.currentLinkID)) return false;
        if (!ReadFloat(inStream, outPayload.progress)) return false;
        if (!ReadFloat(inStream, outPayload.x)) return false;
        if (!ReadFloat(inStream, outPayload.z)) return false;
        if (!ReadFloat(inStream, outPayload.heading)) return false;
        if (!ReadFloat(inStream, outPayload.velocity)) return false;
        if (!ReadFloat(inStream, outPayload.battery)) return false;
        if (!ReadUInt8(inStream, rawState)) return false;

        outPayload.state = static_cast<RobotState>(rawState);
        return true;
    }

    void WriteRouteCommandPayload(OutputMemoryStream& outStream, const RouteCommandPayload& payload)
    {
        outStream.Write(payload.routeID);

        const uint16_t nodeCount = static_cast<uint16_t>(
            payload.nodes.size() > kMaxRouteNodes ? kMaxRouteNodes : payload.nodes.size()
        );
        outStream.Write(nodeCount);

        for (uint16_t i = 0; i < nodeCount; ++i)
        {
            outStream.Write(payload.nodes[i].nodeID);
            outStream.Write(payload.nodes[i].arrivalTime);
            outStream.Write(payload.nodes[i].departureTime);
        }
    }

    bool ReadRouteCommandPayload(InputMemoryStream& inStream, RouteCommandPayload& outPayload)
    {
        uint16_t nodeCount = 0;
        if (!ReadUInt32(inStream, outPayload.routeID)) return false;
        if (!ReadUInt16(inStream, nodeCount)) return false;
        if (nodeCount > kMaxRouteNodes) return false;

        outPayload.nodes.clear();
        outPayload.nodes.reserve(nodeCount);

        for (uint16_t i = 0; i < nodeCount; ++i)
        {
            RouteNodeTime node;
            if (!ReadUInt32(inStream, node.nodeID)) return false;
            if (!ReadFloat(inStream, node.arrivalTime)) return false;
            if (!ReadFloat(inStream, node.departureTime)) return false;
            outPayload.nodes.push_back(node);
        }
        return true;
    }

    bool WriteTrajectoryCommandPayload(OutputMemoryStream& outStream, const TrajectoryCommandPayload& payload)
    {
        if (payload.routeID == 0 ||
            payload.formatVersion != kTrajectoryFormatVersion ||
            payload.startNodeID == 0 ||
            payload.finalNodeID == 0 ||
            payload.waypoints.empty() ||
            payload.waypoints.size() > kMaxTrajectoryWaypoints)
            return false;
        if (!std::isfinite(payload.millimetersPerMapUnit) ||
            payload.millimetersPerMapUnit <= 0.0f)
            return false;
        for (const TrajectoryWaypoint& waypoint : payload.waypoints)
        {
            if (!std::isfinite(waypoint.forwardMm) ||
                !std::isfinite(waypoint.leftMm) ||
                !std::isfinite(waypoint.headingRad) ||
                !std::isfinite(waypoint.targetSpeedMmPerSecond) ||
                waypoint.targetSpeedMmPerSecond < 0.0f)
                return false;
        }

        constexpr uint8_t knownFlags = TRAJECTORY_FLAG_NODE_BOUNDARY |
            TRAJECTORY_FLAG_STOP | TRAJECTORY_FLAG_ROTATE_IN_PLACE |
            TRAJECTORY_FLAG_FINAL;
        for (const TrajectoryWaypoint& waypoint : payload.waypoints)
            if ((waypoint.flags & static_cast<uint8_t>(~knownFlags)) != 0) return false;

        outStream.Write(payload.routeID);
        outStream.Write(payload.formatVersion);
        const uint16_t waypointCount = static_cast<uint16_t>(payload.waypoints.size());
        outStream.Write(waypointCount);
        outStream.Write(payload.startNodeID);
        outStream.Write(payload.finalNodeID);
        outStream.Write(payload.millimetersPerMapUnit);
        for (const TrajectoryWaypoint& waypoint : payload.waypoints)
        {
            outStream.Write(waypoint.forwardMm);
            outStream.Write(waypoint.leftMm);
            outStream.Write(waypoint.headingRad);
            outStream.Write(waypoint.targetSpeedMmPerSecond);
            outStream.Write(waypoint.nodeID);
            outStream.Write(waypoint.flags);
        }
        return true;
    }

    bool ReadTrajectoryCommandPayload(InputMemoryStream& inStream, TrajectoryCommandPayload& outPayload)
    {
        outPayload = {};
        TrajectoryCommandPayload decoded;
        uint8_t formatVersion = 0;
        uint16_t waypointCount = 0;
        if (!ReadUInt32(inStream, decoded.routeID)) return false;
        if (!ReadUInt8(inStream, formatVersion)) return false;
        if (formatVersion != kTrajectoryFormatVersion) return false;
        decoded.formatVersion = formatVersion;
        if (!ReadUInt16(inStream, waypointCount)) return false;
        if (waypointCount == 0 || waypointCount > kMaxTrajectoryWaypoints) return false;
        if (!ReadUInt32(inStream, decoded.startNodeID)) return false;
        if (!ReadUInt32(inStream, decoded.finalNodeID)) return false;
        if (!ReadFloat(inStream, decoded.millimetersPerMapUnit)) return false;
        if (decoded.routeID == 0 || decoded.startNodeID == 0 ||
            decoded.finalNodeID == 0 ||
            !std::isfinite(decoded.millimetersPerMapUnit) ||
            decoded.millimetersPerMapUnit <= 0.0f) return false;

        decoded.waypoints.reserve(waypointCount);
        constexpr uint8_t knownFlags = TRAJECTORY_FLAG_NODE_BOUNDARY |
            TRAJECTORY_FLAG_STOP | TRAJECTORY_FLAG_ROTATE_IN_PLACE |
            TRAJECTORY_FLAG_FINAL;
        for (uint16_t i = 0; i < waypointCount; ++i)
        {
            TrajectoryWaypoint waypoint;
            if (!ReadFloat(inStream, waypoint.forwardMm)) return false;
            if (!ReadFloat(inStream, waypoint.leftMm)) return false;
            if (!ReadFloat(inStream, waypoint.headingRad)) return false;
            if (!ReadFloat(inStream, waypoint.targetSpeedMmPerSecond)) return false;
            if (!ReadUInt32(inStream, waypoint.nodeID)) return false;
            if (!ReadUInt8(inStream, waypoint.flags)) return false;
            if (!std::isfinite(waypoint.forwardMm) ||
                !std::isfinite(waypoint.leftMm) ||
                !std::isfinite(waypoint.headingRad) ||
                !std::isfinite(waypoint.targetSpeedMmPerSecond) ||
                waypoint.targetSpeedMmPerSecond < 0.0f ||
                (waypoint.flags & static_cast<uint8_t>(~knownFlags)) != 0)
                return false;
            decoded.waypoints.push_back(waypoint);
        }

        if (inStream.GetRemainDataSize() != 0) return false;
        outPayload = std::move(decoded);
        return true;
    }

    void WriteArrivedPayload(OutputMemoryStream& outStream, const ArrivedPayload& payload)
    {
        outStream.Write(payload.currentNodeID);
    }

    bool ReadArrivedPayload(InputMemoryStream& inStream, ArrivedPayload& outPayload)
    {
        return ReadUInt32(inStream, outPayload.currentNodeID);
    }

    void WriteErrorPayload(OutputMemoryStream& outStream, const ErrorPayload& payload)
    {
        outStream.Write(static_cast<uint16_t>(payload.errorCode));
        outStream.Write(payload.detail);
    }

    bool ReadErrorPayload(InputMemoryStream& inStream, ErrorPayload& outPayload)
    {
        uint16_t rawErrorCode = 0;
        if (!ReadUInt16(inStream, rawErrorCode)) return false;
        if (!ReadUInt32(inStream, outPayload.detail)) return false;

        outPayload.errorCode = static_cast<ErrorCode>(rawErrorCode);
        return true;
    }

    void WriteTimePayload(OutputMemoryStream& outStream, const TimePayload& payload)
    {
        outStream.Write(payload.timestampMs);
    }

    bool ReadTimePayload(InputMemoryStream& inStream, TimePayload& outPayload)
    {
        return ReadUInt32(inStream, outPayload.timestampMs);
    }
}
