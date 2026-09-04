#pragma once
#include <sys/socket.h>
#include <unistd.h>
#include <cstdint>
#include <arpa/inet.h>
#include <netdb.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>

using SOCKET = unsigned int;

#define ERROR -1

// Unity/session packet IDs.
// New ESP32/FakeRobot traffic uses RobotProtocol::PacketID in Protocol.hpp.
enum UnityPacketType : uint8_t
{
    UPT_REPLICATION = 0,
    UPT_MAZE_DATA = 1,
    UPT_HELLO = 2,
    UPT_DISCONNECTED = 3,
    UPT_READY_MAP = 4,
    UPT_READY_OBJECT = 5,
    // Observation-only physical pose for a separate Unity comparison marker.
    // This never replaces the authoritative UPT_REPLICATION AGV pose.
    UPT_VISION_OBSERVATION = 6,
    // Idempotent cargo attachment state for the Unity viewer.
    UPT_CARGO_STATE = 7,
    
    // ==========================================
    // Deprecated legacy robot IDs from the old uint8_t packet scheme.
    // Kept only to avoid breaking old references. Do not use for ESP32.
    // ==========================================
    UPT_LEGACY_ROBOT_ROUTE = 10,
    UPT_LEGACY_ROBOT_CANCEL_ROUTE = 11,
    UPT_LEGACY_ROBOT_ARRIVED = 12,
    UPT_LEGACY_ROBOT_STATUS = 13,
    UPT_LEGACY_ROBOT_ERROR = 14,
    UPT_LEGACY_ROBOT_HEARTBEAT = 15
};

enum ReplicationAction : uint8_t
{
    RT_CREATE,
    RT_UPDATE,
    RT_DESTORY,
    MAX
};

enum ClassID: uint32_t
{
    OBJ_DEFAULT=1000,
    OBJ_AGV=1001    
};
