#pragma once
#include <cstdint>

namespace RobotProtocol
{
    constexpr uint16_t kProtocolVersion = 1;
    constexpr uint16_t kMaxRouteNodes = 64;
    constexpr uint8_t kTrajectoryFormatVersion = 1;
    constexpr uint16_t kMaxTrajectoryWaypoints = 64;

    enum ClientCapability : uint32_t
    {
        CAPABILITY_NONE = 0,
        // Full execution support: follower, safety, STATUS and ARRIVED.
        CAPABILITY_TRAJECTORY_COMMAND = 1u << 0,
        // Parse/validate/store only. Never a runtime motion target.
        CAPABILITY_TRAJECTORY_PREVIEW = 1u << 1
    };

    enum class ClientType : uint8_t
    {
        UNKNOWN = 0,
        UNITY = 1,
        ESP32 = 2,
        TOOL = 3,
        FAKE_ROBOT = 4
    };

    enum class PacketID : uint16_t
    {
        ROUTE_COMMAND = 100,
        CANCEL_ROUTE = 101,
        TRAJECTORY_COMMAND = 102,

        STATUS = 200,
        ARRIVED = 201,

        PING = 300,
        PONG = 301,

        HELLO = 400,
        HELLO_ACK = 401,

        ERROR_PACKET = 500,
        EMERGENCY_STOP = 501
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

    struct ErrorPayload
    {
        ErrorCode errorCode = ErrorCode::NONE;
        uint32_t detail = 0;
    };

    struct TimePayload
    {
        uint32_t timestampMs = 0;
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
