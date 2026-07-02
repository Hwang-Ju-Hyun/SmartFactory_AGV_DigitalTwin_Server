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

enum PacketType : uint8_t
{
    PT_Replication=0,
    PT_MAZE_DATA=1,
    PT_Hello=2,
    PT_Disconnected=3,
    PT_READY_MAP=4,
    PT_READY_OBJECT=5,
    
    // ==========================================
    // 2. Robot Protocol (Server <-> 실제 로봇)
    // ==========================================
    PT_ROBOT_MOVE = 10,       // 이 경로(Link들)를 따라 이동해라
    PT_ROBOT_STOP = 11,       // 긴급 정지
    PT_ROBOT_RESUME = 12,     // 다시 출발
    PT_ROBOT_STATUS = 13,     // (로봇->서버) 내 현재 위치, 속도, 배터리 상태
    PT_ROBOT_HEARTBEAT = 14,  // (로봇->서버) 나 살아있음
    PT_ROBOT_ERROR = 15       // (로봇->서버) 장애물 발견 / 에러 발생
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