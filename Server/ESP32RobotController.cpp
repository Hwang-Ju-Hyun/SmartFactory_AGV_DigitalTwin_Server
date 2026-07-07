#include "ESP32RobotController.hpp"

ESP32RobotController::ESP32RobotController(TCPSessionPtr _robotSession)
    : m_RobotSession(_robotSession)
{
 
}


void ESP32RobotController::FollowRoute(const RoutePacket& _routePacket)
{
    // 관제탑이 명령을 내리면 메모리 스트림에 직렬화해서 진짜 로봇에게 쏴버림
    OutputMemoryStream outStream;
    uint8_t packetType = PacketType::PT_ROUTE;
    _routePacket.Serialize(outStream);
    m_RobotSession->SendPacket(outStream);
}

void ESP32RobotController::CancelRoute()
{
    // ESP32 로봇에게 경로 취소를 전달하는 로직 구현
    OutputMemoryStream outStream;
    outStream.Write(static_cast<uint8_t>(PT_CANCEL_ROUTE));
    m_RobotSession->SendPacket(outStream);
}

StatusPacket ESP32RobotController::GetStatus()
{
    StatusPacket p;
    return p;
}

void ESP32RobotController::Update(float dt, float serverTime)
{
    // ESP32 로봇을 업데이트하는 로직 구현
}