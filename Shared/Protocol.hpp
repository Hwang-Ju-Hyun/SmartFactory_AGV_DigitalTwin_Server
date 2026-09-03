#pragma once
#include <cstdint>
#include <optional>
#include <string>

namespace RobotProtocol
{
    constexpr uint16_t kProtocolVersion = 1;
    constexpr uint16_t kMaxRouteNodes = 64;
    constexpr uint8_t kTrajectoryFormatVersion = 1;
    constexpr uint16_t kMaxTrajectoryWaypoints = 64;
    // Vision identities are length-prefixed byte strings on the wire. The
    // current VisionTracker uses 16-character digest IDs, while this limit
    // leaves room for a full SHA-256 hex digest without unbounded allocation.
    constexpr uint16_t kMaxVisionIdentityBytes = 64;

    enum ClientCapability : uint32_t
    {
        CAPABILITY_NONE = 0,
        // Full execution support: follower, safety, STATUS and ARRIVED.
        CAPABILITY_TRAJECTORY_COMMAND = 1u << 0,
        // Parse/validate/store only. Never a runtime motion target.
        CAPABILITY_TRAJECTORY_PREVIEW = 1u << 1,
        // Executes bounded node-centering primitives while a completed
        // physical-fleet edge is stopped and awaiting Server verification.
        CAPABILITY_NODE_CORRECTION = 1u << 2
    };

    enum class ClientType : uint8_t
    {
        UNKNOWN = 0,
        UNITY = 1,
        ESP32 = 2,
        TOOL = 3,
        FAKE_ROBOT = 4,
        VISION_TRACKER = 5
    };

    enum class PacketID : uint16_t
    {
        ROUTE_COMMAND = 100,
        CANCEL_ROUTE = 101,
        TRAJECTORY_COMMAND = 102,
        NODE_CORRECTION_COMMAND = 103,

        STATUS = 200,
        ARRIVED = 201,
        NODE_CORRECTION_REPORT = 202,

        PING = 300,
        PONG = 301,

        HELLO = 400,
        HELLO_ACK = 401,

        ERROR_PACKET = 500,
        EMERGENCY_STOP = 501,

        VISION_HELLO = 600,
        VISION_HELLO_ACK = 601,
        VISION_OBSERVATION = 602
    };

    enum class RobotState : uint8_t
    {
        UNKNOWN = 0,
        IDLE = 1,
        MOVING = 2,
        LOADING = 3,
        UNLOADING = 4,
        WAIT_REPLAN = 5,
        FAULT = 100,
        EMERGENCY_STOPPED = 101
    };

    enum class ErrorCode : uint16_t
    {
        NONE = 0,
        PROTOCOL_MISMATCH = 1,
        UNKNOWN_AGV = 2,
        MOTOR_FAULT = 100,
        LOW_BATTERY = 101,
        OBSTACLE_DETECTED = 102
    };

    // A WHEEL_MISMATCH keeps the legacy MOTOR_FAULT detail first so existing
    // receivers still fail safe. Five immediately following MOTOR_FAULT
    // records may carry a tagged, frozen encoder snapshot in the same uint32
    // detail field; ERROR_PACKET and ErrorPayload remain unchanged.
    constexpr uint32_t kWheelMismatchLegacyDetail = 65539u;
    constexpr uint8_t kMotorFaultDiagnosticVersion = 1;
    enum class MotorFaultDiagnosticTag : uint8_t
    {
        CONTEXT = 0xD0,
        LEFT_PROGRESS = 0xD1,
        RIGHT_PROGRESS = 0xD2,
        LEFT_TARGET = 0xD3,
        RIGHT_TARGET = 0xD4
    };

    enum class VisionHelloRejectionReason : uint16_t
    {
        NONE = 0,
        PROTOCOL_MISMATCH = 1,
        FEATURE_DISABLED = 2,
        INVALID_SOURCE = 3,
        MAP_CONTRACT_MISMATCH = 4,
        POSE_CONTRACT_MISMATCH = 5,
        DUPLICATE_SESSION = 6,
        MALFORMED_HANDSHAKE = 7
    };

    enum class VisionTrackingState : uint8_t
    {
        MEASURED = 1,
        HELD = 2,
        LOST = 3
    };

    enum class NodeCorrectionAction : uint8_t
    {
        DRIVE_FORWARD = 2,
        TURN_CW = 3,
        TURN_CCW = 4
    };

    enum class NodeCorrectionResult : uint8_t
    {
        COMPLETED = 2,
        REJECTED = 3,
        FAULT = 4
    };

    // Mirrors the locked-calibration guard states emitted by VisionTracker.
    // A pose must still be rejected by the Server unless the observation and
    // its associated session satisfy the Server's validation policy.
    enum class VisionVerificationState : uint8_t
    {
        UNKNOWN = 0,
        VERIFIED = 1,
        AWAITING_VERIFICATION = 2,
        REFERENCES_MISSING = 3,
        MISMATCH = 4,
        STALE = 5,
        INVALID = 6
    };

    enum VisionQualityField : uint16_t
    {
        VISION_QUALITY_NONE = 0,
        VISION_QUALITY_DECISION_MARGIN = 1u << 0,
        VISION_QUALITY_CALIBRATION_RMS_ERROR = 1u << 1,
        VISION_QUALITY_VERIFICATION_REFERENCE_COUNT = 1u << 2,
        VISION_QUALITY_VERIFICATION_RMS_ERROR = 1u << 3,
        VISION_QUALITY_VERIFICATION_MAX_ERROR = 1u << 4,
        VISION_QUALITY_VERIFICATION_COVERAGE = 1u << 5,
        VISION_QUALITY_VERIFICATION_AGE = 1u << 6
    };

#pragma pack(push, 1)
    struct PacketHeader
    {
        uint16_t packetSize; // Full TCP frame size: this header + payload.
        uint16_t packetID;
        uint32_t agvID;
        uint32_t sequence;
    };
#pragma pack(pop)

    struct PacketBodyHeader
    {
        uint16_t packetID;
        uint32_t agvID;
        uint32_t sequence;
    };

    struct HelloPayload
    {
        uint16_t protocolVersion = kProtocolVersion;
        ClientType clientType = ClientType::UNKNOWN;
        uint32_t requestedAgvID = 0;
        // Optional on the wire. Legacy v1 clients omit this field and are
        // treated as CAPABILITY_NONE by the server.
        uint32_t capabilities = CAPABILITY_NONE;
    };

    struct HelloAckPayload
    {
        uint16_t protocolVersion = kProtocolVersion;
        uint8_t accepted = 0;
        uint32_t assignedAgvID = 0;
        ErrorCode errorCode = ErrorCode::NONE;
    };

    struct StatusPayload
    {
        uint32_t currentNodeID = 0;
        uint32_t currentLinkID = 0;
        float progress = 0.0f;
        float x = 0.0f;
        float z = 0.0f;
        float heading = 0.0f;
        float velocity = 0.0f;
        float battery = 100.0f;
        RobotState state = RobotState::UNKNOWN; // Physical state from robot, or server AGV state when mirrored.
    };

    struct ArrivedPayload
    {
        uint32_t currentNodeID = 0;
    };

    struct NodeCorrectionCommandPayload
    {
        uint32_t routeID = 0;
        uint32_t nodeID = 0;
        uint32_t commandID = 0;
        NodeCorrectionAction action = NodeCorrectionAction::DRIVE_FORWARD;
        // Millimetres for DRIVE_FORWARD and radians for TURN_CW/TURN_CCW.
        // Direction is carried by action, so this value is always positive.
        float magnitude = 0.0f;
    };

    struct NodeCorrectionReportPayload
    {
        uint32_t routeID = 0;
        uint32_t nodeID = 0;
        uint32_t commandID = 0;
        NodeCorrectionResult result = NodeCorrectionResult::REJECTED;
        uint32_t detail = 0;
    };

    struct ErrorPayload
    {
        ErrorCode errorCode = ErrorCode::NONE;
        uint32_t detail = 0;
    };

    struct TimePayload
    {
        uint32_t timestampMs = 0;
    };

    struct VisionHelloPayload
    {
        uint16_t protocolVersion = kProtocolVersion;
        uint32_t sourceID = 0;
        uint64_t sessionID = 0;
        std::string mapContractID;
        std::string poseContractID;
    };

    struct VisionHelloAckPayload
    {
        uint16_t protocolVersion = kProtocolVersion;
        uint8_t accepted = 0;
        VisionHelloRejectionReason rejectionReason =
            VisionHelloRejectionReason::NONE;
        uint32_t sourceID = 0;
        uint64_t sessionID = 0;
    };

    struct VisionPose
    {
        float xMm = 0.0f;
        float zMm = 0.0f;
        float headingDeg = 0.0f;
    };

    struct VisionQualityMetadata
    {
        // qualityFields identifies which numeric values are available. The
        // scalar fields remain fixed-width to keep parsing deterministic.
        uint16_t qualityFields = VISION_QUALITY_NONE;
        float decisionMargin = 0.0f;
        float calibrationRmsErrorMm = 0.0f;
        uint16_t verificationReferenceCount = 0;
        float verificationRmsErrorMm = 0.0f;
        float verificationMaxErrorMm = 0.0f;
        float verificationCoverageRatio = 0.0f;
        uint32_t verificationAgeMs = 0;
    };

    struct VisionObservationPayload
    {
        // Sender-process monotonic metadata. It is not comparable to the
        // Server clock and must never drive Server freshness decisions; use
        // Server receive time for that purpose.
        uint64_t sourceTimestampUs = 0;
        uint32_t reportedAgeMs = 0;
        VisionTrackingState state = VisionTrackingState::LOST;
        // MEASURED and HELD require a pose; LOST requires no pose. The wire
        // format omits x/z/heading entirely for LOST observations.
        std::optional<VisionPose> pose;
        std::string calibrationID;
        VisionVerificationState verificationState =
            VisionVerificationState::UNKNOWN;
        VisionQualityMetadata quality;
    };

    enum TrajectoryWaypointFlag : uint8_t
    {
        TRAJECTORY_FLAG_NONE = 0,
        TRAJECTORY_FLAG_NODE_BOUNDARY = 1u << 0,
        TRAJECTORY_FLAG_STOP = 1u << 1,
        TRAJECTORY_FLAG_ROTATE_IN_PLACE = 1u << 2,
        TRAJECTORY_FLAG_FINAL = 1u << 3
    };
}
