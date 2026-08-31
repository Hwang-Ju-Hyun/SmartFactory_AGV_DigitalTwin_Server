#include "PacketSerializer.hpp"
#include <algorithm>
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

        void WriteUInt64(OutputMemoryStream& outStream, uint64_t value)
        {
            outStream.Write(static_cast<uint32_t>(value & 0xffffffffu));
            outStream.Write(static_cast<uint32_t>(value >> 32u));
        }

        bool ReadUInt64(InputMemoryStream& inStream, uint64_t& value)
        {
            uint32_t low = 0;
            uint32_t high = 0;
            if (!ReadUInt32(inStream, low)) return false;
            if (!ReadUInt32(inStream, high)) return false;
            value = static_cast<uint64_t>(low) |
                (static_cast<uint64_t>(high) << 32u);
            return true;
        }

        bool IsValidVisionIdentity(const std::string& value)
        {
            return !value.empty() &&
                value.size() <= kMaxVisionIdentityBytes &&
                std::all_of(
                    value.begin(), value.end(),
                    [](unsigned char byte)
                    {
                        return byte >= 0x21u && byte <= 0x7eu;
                    });
        }

        bool WriteVisionIdentity(OutputMemoryStream& outStream,
                                 const std::string& value)
        {
            if (!IsValidVisionIdentity(value)) return false;

            const uint16_t length = static_cast<uint16_t>(value.size());
            outStream.Write(length);
            outStream.Write(value.data(), length);
            return true;
        }

        bool ReadVisionIdentity(InputMemoryStream& inStream,
                                std::string& value)
        {
            uint16_t length = 0;
            if (!ReadUInt16(inStream, length)) return false;
            if (length == 0 || length > kMaxVisionIdentityBytes ||
                !HasBytes(inStream, length)) return false;

            std::string decoded(length, '\0');
            inStream.Read(decoded.data(), length);
            if (!IsValidVisionIdentity(decoded)) return false;
            value = std::move(decoded);
            return true;
        }

        bool IsKnownVisionHelloRejectionReason(uint16_t rawReason)
        {
            switch (static_cast<VisionHelloRejectionReason>(rawReason))
            {
            case VisionHelloRejectionReason::NONE:
            case VisionHelloRejectionReason::PROTOCOL_MISMATCH:
            case VisionHelloRejectionReason::FEATURE_DISABLED:
            case VisionHelloRejectionReason::INVALID_SOURCE:
            case VisionHelloRejectionReason::MAP_CONTRACT_MISMATCH:
            case VisionHelloRejectionReason::POSE_CONTRACT_MISMATCH:
            case VisionHelloRejectionReason::DUPLICATE_SESSION:
            case VisionHelloRejectionReason::MALFORMED_HANDSHAKE:
                return true;
            default:
                return false;
            }
        }

        bool IsKnownVisionTrackingState(uint8_t rawState)
        {
            switch (static_cast<VisionTrackingState>(rawState))
            {
            case VisionTrackingState::MEASURED:
            case VisionTrackingState::HELD:
            case VisionTrackingState::LOST:
                return true;
            default:
                return false;
            }
        }

        bool IsKnownVisionVerificationState(uint8_t rawState)
        {
            switch (static_cast<VisionVerificationState>(rawState))
            {
            case VisionVerificationState::UNKNOWN:
            case VisionVerificationState::VERIFIED:
            case VisionVerificationState::AWAITING_VERIFICATION:
            case VisionVerificationState::REFERENCES_MISSING:
            case VisionVerificationState::MISMATCH:
            case VisionVerificationState::STALE:
            case VisionVerificationState::INVALID:
                return true;
            default:
                return false;
            }
        }

        bool IsKnownNodeCorrectionAction(uint8_t rawAction)
        {
            switch (static_cast<NodeCorrectionAction>(rawAction))
            {
            case NodeCorrectionAction::DRIVE_FORWARD:
            case NodeCorrectionAction::TURN_CW:
            case NodeCorrectionAction::TURN_CCW:
                return true;
            default:
                return false;
            }
        }

        bool IsKnownNodeCorrectionResult(uint8_t rawResult)
        {
            switch (static_cast<NodeCorrectionResult>(rawResult))
            {
            case NodeCorrectionResult::COMPLETED:
            case NodeCorrectionResult::REJECTED:
            case NodeCorrectionResult::FAULT:
                return true;
            default:
                return false;
            }
        }

        constexpr uint16_t kKnownVisionQualityFields =
            VISION_QUALITY_DECISION_MARGIN |
            VISION_QUALITY_CALIBRATION_RMS_ERROR |
            VISION_QUALITY_VERIFICATION_REFERENCE_COUNT |
            VISION_QUALITY_VERIFICATION_RMS_ERROR |
            VISION_QUALITY_VERIFICATION_MAX_ERROR |
            VISION_QUALITY_VERIFICATION_COVERAGE |
            VISION_QUALITY_VERIFICATION_AGE;
    }

    bool IsKnownRobotPacketID(uint16_t rawPacketID)
    {
        switch (static_cast<PacketID>(rawPacketID))
        {
        case PacketID::ROUTE_COMMAND:
        case PacketID::CANCEL_ROUTE:
        case PacketID::TRAJECTORY_COMMAND:
        case PacketID::NODE_CORRECTION_COMMAND:
        case PacketID::STATUS:
        case PacketID::ARRIVED:
        case PacketID::NODE_CORRECTION_REPORT:
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

    bool IsKnownVisionPacketID(uint16_t rawPacketID)
    {
        switch (static_cast<PacketID>(rawPacketID))
        {
        case PacketID::VISION_HELLO:
        case PacketID::VISION_HELLO_ACK:
        case PacketID::VISION_OBSERVATION:
            return true;
        default:
            return false;
        }
    }

    bool IsKnownPacketID(uint16_t rawPacketID)
    {
        return IsKnownRobotPacketID(rawPacketID) ||
            IsKnownVisionPacketID(rawPacketID);
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

    bool WriteNodeCorrectionCommandPayload(
        OutputMemoryStream& outStream,
        const NodeCorrectionCommandPayload& payload)
    {
        const uint8_t rawAction = static_cast<uint8_t>(payload.action);
        if (payload.routeID == 0 || payload.nodeID == 0 ||
            payload.commandID == 0 ||
            !IsKnownNodeCorrectionAction(rawAction) ||
            !std::isfinite(payload.magnitude) || payload.magnitude <= 0.0f)
        {
            return false;
        }

        outStream.Write(payload.routeID);
        outStream.Write(payload.nodeID);
        outStream.Write(payload.commandID);
        outStream.Write(rawAction);
        outStream.Write(payload.magnitude);
        return true;
    }

    bool ReadNodeCorrectionCommandPayload(
        InputMemoryStream& inStream,
        NodeCorrectionCommandPayload& outPayload)
    {
        NodeCorrectionCommandPayload decoded;
        uint8_t rawAction = 0;
        if (!ReadUInt32(inStream, decoded.routeID)) return false;
        if (!ReadUInt32(inStream, decoded.nodeID)) return false;
        if (!ReadUInt32(inStream, decoded.commandID)) return false;
        if (!ReadUInt8(inStream, rawAction)) return false;
        if (!ReadFloat(inStream, decoded.magnitude)) return false;
        if (inStream.GetRemainDataSize() != 0 || decoded.routeID == 0 ||
            decoded.nodeID == 0 || decoded.commandID == 0 ||
            !IsKnownNodeCorrectionAction(rawAction) ||
            !std::isfinite(decoded.magnitude) || decoded.magnitude <= 0.0f)
        {
            return false;
        }
        decoded.action = static_cast<NodeCorrectionAction>(rawAction);
        outPayload = decoded;
        return true;
    }

    bool WriteNodeCorrectionReportPayload(
        OutputMemoryStream& outStream,
        const NodeCorrectionReportPayload& payload)
    {
        const uint8_t rawResult = static_cast<uint8_t>(payload.result);
        if (payload.routeID == 0 || payload.nodeID == 0 ||
            payload.commandID == 0 ||
            !IsKnownNodeCorrectionResult(rawResult))
        {
            return false;
        }

        outStream.Write(payload.routeID);
        outStream.Write(payload.nodeID);
        outStream.Write(payload.commandID);
        outStream.Write(rawResult);
        outStream.Write(payload.detail);
        return true;
    }

    bool ReadNodeCorrectionReportPayload(
        InputMemoryStream& inStream,
        NodeCorrectionReportPayload& outPayload)
    {
        NodeCorrectionReportPayload decoded;
        uint8_t rawResult = 0;
        if (!ReadUInt32(inStream, decoded.routeID)) return false;
        if (!ReadUInt32(inStream, decoded.nodeID)) return false;
        if (!ReadUInt32(inStream, decoded.commandID)) return false;
        if (!ReadUInt8(inStream, rawResult)) return false;
        if (!ReadUInt32(inStream, decoded.detail)) return false;
        if (inStream.GetRemainDataSize() != 0 || decoded.routeID == 0 ||
            decoded.nodeID == 0 || decoded.commandID == 0 ||
            !IsKnownNodeCorrectionResult(rawResult))
        {
            return false;
        }
        decoded.result = static_cast<NodeCorrectionResult>(rawResult);
        outPayload = decoded;
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

    bool WriteVisionHelloPayload(OutputMemoryStream& outStream,
                                 const VisionHelloPayload& payload)
    {
        if (!IsValidVisionIdentity(payload.mapContractID) ||
            !IsValidVisionIdentity(payload.poseContractID)) return false;

        outStream.Write(payload.protocolVersion);
        outStream.Write(payload.sourceID);
        WriteUInt64(outStream, payload.sessionID);
        if (!WriteVisionIdentity(outStream, payload.mapContractID)) return false;
        return WriteVisionIdentity(outStream, payload.poseContractID);
    }

    bool ReadVisionHelloPayload(InputMemoryStream& inStream,
                                VisionHelloPayload& outPayload)
    {
        VisionHelloPayload decoded;
        if (!ReadUInt16(inStream, decoded.protocolVersion)) return false;
        if (!ReadUInt32(inStream, decoded.sourceID)) return false;
        if (!ReadUInt64(inStream, decoded.sessionID)) return false;
        if (!ReadVisionIdentity(inStream, decoded.mapContractID)) return false;
        if (!ReadVisionIdentity(inStream, decoded.poseContractID)) return false;
        if (inStream.GetRemainDataSize() != 0) return false;

        outPayload = std::move(decoded);
        return true;
    }

    bool WriteVisionHelloAckPayload(OutputMemoryStream& outStream,
                                    const VisionHelloAckPayload& payload)
    {
        const uint16_t rawReason =
            static_cast<uint16_t>(payload.rejectionReason);
        if (payload.accepted > 1 ||
            !IsKnownVisionHelloRejectionReason(rawReason) ||
            (payload.accepted == 1 &&
             payload.rejectionReason != VisionHelloRejectionReason::NONE) ||
            (payload.accepted == 0 &&
             payload.rejectionReason == VisionHelloRejectionReason::NONE))
        {
            return false;
        }

        outStream.Write(payload.protocolVersion);
        outStream.Write(payload.accepted);
        outStream.Write(rawReason);
        outStream.Write(payload.sourceID);
        WriteUInt64(outStream, payload.sessionID);
        return true;
    }

    bool ReadVisionHelloAckPayload(InputMemoryStream& inStream,
                                   VisionHelloAckPayload& outPayload)
    {
        VisionHelloAckPayload decoded;
        uint16_t rawReason = 0;
        if (!ReadUInt16(inStream, decoded.protocolVersion)) return false;
        if (!ReadUInt8(inStream, decoded.accepted)) return false;
        if (!ReadUInt16(inStream, rawReason)) return false;
        if (!IsKnownVisionHelloRejectionReason(rawReason)) return false;
        if (!ReadUInt32(inStream, decoded.sourceID)) return false;
        if (!ReadUInt64(inStream, decoded.sessionID)) return false;
        if (inStream.GetRemainDataSize() != 0) return false;

        decoded.rejectionReason =
            static_cast<VisionHelloRejectionReason>(rawReason);
        if (decoded.accepted > 1 ||
            (decoded.accepted == 1 &&
             decoded.rejectionReason != VisionHelloRejectionReason::NONE) ||
            (decoded.accepted == 0 &&
             decoded.rejectionReason == VisionHelloRejectionReason::NONE))
        {
            return false;
        }

        outPayload = decoded;
        return true;
    }

    bool WriteVisionObservationPayload(OutputMemoryStream& outStream,
                                       const VisionObservationPayload& payload)
    {
        const uint8_t rawState = static_cast<uint8_t>(payload.state);
        const uint8_t rawVerificationState =
            static_cast<uint8_t>(payload.verificationState);
        const bool stateHasPose =
            payload.state == VisionTrackingState::MEASURED ||
            payload.state == VisionTrackingState::HELD;
        if (!IsKnownVisionTrackingState(rawState) ||
            !IsKnownVisionVerificationState(rawVerificationState) ||
            stateHasPose != payload.pose.has_value() ||
            !IsValidVisionIdentity(payload.calibrationID) ||
            (payload.quality.qualityFields &
             static_cast<uint16_t>(~kKnownVisionQualityFields)) != 0)
        {
            return false;
        }

        WriteUInt64(outStream, payload.sourceTimestampUs);
        outStream.Write(payload.reportedAgeMs);
        outStream.Write(rawState);
        if (payload.pose.has_value())
        {
            outStream.Write(payload.pose->xMm);
            outStream.Write(payload.pose->zMm);
            outStream.Write(payload.pose->headingDeg);
        }
        if (!WriteVisionIdentity(outStream, payload.calibrationID)) return false;
        outStream.Write(rawVerificationState);
        outStream.Write(payload.quality.qualityFields);
        outStream.Write(payload.quality.decisionMargin);
        outStream.Write(payload.quality.calibrationRmsErrorMm);
        outStream.Write(payload.quality.verificationReferenceCount);
        outStream.Write(payload.quality.verificationRmsErrorMm);
        outStream.Write(payload.quality.verificationMaxErrorMm);
        outStream.Write(payload.quality.verificationCoverageRatio);
        outStream.Write(payload.quality.verificationAgeMs);
        return true;
    }

    bool ReadVisionObservationPayload(InputMemoryStream& inStream,
                                      VisionObservationPayload& outPayload)
    {
        VisionObservationPayload decoded;
        uint8_t rawState = 0;
        uint8_t rawVerificationState = 0;
        if (!ReadUInt64(inStream, decoded.sourceTimestampUs)) return false;
        if (!ReadUInt32(inStream, decoded.reportedAgeMs)) return false;
        if (!ReadUInt8(inStream, rawState) ||
            !IsKnownVisionTrackingState(rawState)) return false;
        decoded.state = static_cast<VisionTrackingState>(rawState);

        if (decoded.state == VisionTrackingState::MEASURED ||
            decoded.state == VisionTrackingState::HELD)
        {
            VisionPose pose;
            if (!ReadFloat(inStream, pose.xMm)) return false;
            if (!ReadFloat(inStream, pose.zMm)) return false;
            if (!ReadFloat(inStream, pose.headingDeg)) return false;
            decoded.pose = pose;
        }

        if (!ReadVisionIdentity(inStream, decoded.calibrationID)) return false;
        if (!ReadUInt8(inStream, rawVerificationState) ||
            !IsKnownVisionVerificationState(rawVerificationState)) return false;
        decoded.verificationState =
            static_cast<VisionVerificationState>(rawVerificationState);
        if (!ReadUInt16(inStream, decoded.quality.qualityFields)) return false;
        if ((decoded.quality.qualityFields &
             static_cast<uint16_t>(~kKnownVisionQualityFields)) != 0) return false;
        if (!ReadFloat(inStream, decoded.quality.decisionMargin)) return false;
        if (!ReadFloat(inStream, decoded.quality.calibrationRmsErrorMm)) return false;
        if (!ReadUInt16(inStream,
                        decoded.quality.verificationReferenceCount)) return false;
        if (!ReadFloat(inStream, decoded.quality.verificationRmsErrorMm)) return false;
        if (!ReadFloat(inStream, decoded.quality.verificationMaxErrorMm)) return false;
        if (!ReadFloat(inStream,
                       decoded.quality.verificationCoverageRatio)) return false;
        if (!ReadUInt32(inStream, decoded.quality.verificationAgeMs)) return false;
        if (inStream.GetRemainDataSize() != 0) return false;

        outPayload = std::move(decoded);
        return true;
    }
}
