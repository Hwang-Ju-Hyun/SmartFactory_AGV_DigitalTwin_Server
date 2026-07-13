#pragma once
#include <cstdint>

namespace RobotProtocol
{
    constexpr uint16_t kProtocolVersion = 1;
    constexpr uint16_t kMaxRouteNodes = 64;

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
}
