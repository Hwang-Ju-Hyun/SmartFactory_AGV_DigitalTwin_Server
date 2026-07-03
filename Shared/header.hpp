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
    PT_ROUTE = 10,          // (서버->로봇) "이 노드들을 순서대로 거쳐서 가라"
    PT_CANCEL_ROUTE = 11,   // (서버->로봇) "경로 폐기! 그 자리에 정지해라"
    PT_ARRIVED = 12,        // (로봇->서버) "다음 노드에 무사히 도착했습니다"
    PT_STATUS = 13,         // (로봇->서버) "현재 X, Z, 각도, 속도, 배터리 상태 보고"
    PT_ERROR = 14,          // (로봇->서버) "모터 고장 / 충돌 감지"
    PT_HEARTBEAT = 15       // (로봇->서버) "나 아직 살아있음 (1초 주기)"
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