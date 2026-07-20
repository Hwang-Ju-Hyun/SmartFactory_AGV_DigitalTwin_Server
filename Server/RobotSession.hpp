#pragma once
#include "IRobotController.hpp"
#include "PacketSerializer.hpp"
#include "TCPSession.hpp"
#include <queue>

class RobotSession
{
private:
    TCPSessionPtr m_TCPSession;
    uint32_t m_AgvID = 0;
    uint32_t m_NextOutgoingSequence = 1;
    uint32_t m_NextRouteID = 1;
    StatusPacket m_LastStatus{};
    bool m_HasStatus = false;
    std::queue<ControllerEvent> m_EventQueue;

public:
    RobotSession(TCPSessionPtr session, uint32_t agvID);

    uint32_t GetAgvID() const { return m_AgvID; }
    TCPSessionPtr GetSession() const { return m_TCPSession; }

    void ProcessPacket(const RobotProtocol::PacketBodyHeader& header, InputMemoryStream& payloadStream);

    void SendHelloAck(const RobotProtocol::HelloAckPayload& payload);
    void SendRoute(const RoutePacket& routePacket);
    void SendCancelRoute(uint32_t agvID);
    void SendPong(uint32_t timestampMs);

    void PrimeStatus(float x, float z, float heading);
    StatusPacket GetStatus() const;
    bool HasEvent() const;
    ControllerEvent PopEvent();

private:
    uint32_t NextSequence();
    void SendRobotPacket(RobotProtocol::PacketID packetID, uint32_t agvID, OutputMemoryStream& payloadStream);
};

using RobotSessionPtr = std::shared_ptr<RobotSession>;
