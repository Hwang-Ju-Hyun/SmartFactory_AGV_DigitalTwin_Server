#include "RobotSession.hpp"
#include <iostream>

RobotSession::RobotSession(TCPSessionPtr session, uint32_t agvID)
    : m_TCPSession(session)
    , m_AgvID(agvID)
{
    m_LastStatus.agvID = agvID;
    m_LastStatus.battery = 100.0f;
}

void RobotSession::ProcessPacket(const RobotProtocol::PacketBodyHeader& header, InputMemoryStream& payloadStream)
{
    const RobotProtocol::PacketID packetID = static_cast<RobotProtocol::PacketID>(header.packetID);

    switch (packetID)
    {
    case RobotProtocol::PacketID::STATUS:
    {
        RobotProtocol::StatusPayload payload;
        if (!RobotProtocol::ReadStatusPayload(payloadStream, payload))
        {
            std::cout << "[RobotSession] Invalid STATUS payload from AGV " << header.agvID << "\n";
            return;
        }

        m_LastStatus.agvID = header.agvID;
        m_LastStatus.currentLinkID = payload.currentLinkID;
        m_LastStatus.progress = payload.progress;
        m_LastStatus.x = payload.x;
        m_LastStatus.z = payload.z;
        m_LastStatus.heading = payload.heading;
        m_LastStatus.velocity = payload.velocity;
        m_LastStatus.battery = payload.battery;
        m_HasStatus = true;
        break;
    }
    case RobotProtocol::PacketID::ARRIVED:
    {
        RobotProtocol::ArrivedPayload payload;
        if (!RobotProtocol::ReadArrivedPayload(payloadStream, payload))
        {
            std::cout << "[RobotSession] Invalid ARRIVED payload from AGV " << header.agvID << "\n";
            return;
        }

        m_EventQueue.push({ ControllerEventType::ARRIVED, payload.currentNodeID, 0 });
        break;
    }
    case RobotProtocol::PacketID::PING:
    {
        RobotProtocol::TimePayload payload;
        if (RobotProtocol::ReadTimePayload(payloadStream, payload))
        {
            SendPong(payload.timestampMs);
        }
        break;
    }
    case RobotProtocol::PacketID::ERROR_PACKET:
    {
        RobotProtocol::ErrorPayload payload;
        if (RobotProtocol::ReadErrorPayload(payloadStream, payload))
        {
            m_EventQueue.push({ ControllerEventType::ERROR_SLIP, static_cast<uint32_t>(payload.errorCode), payload.detail });
        }
        break;
    }
    case RobotProtocol::PacketID::EMERGENCY_STOP:
        m_EventQueue.push({ ControllerEventType::EMERGENCY_STOP, 0, 0 });
        break;
    case RobotProtocol::PacketID::PONG:
    case RobotProtocol::PacketID::HELLO_ACK:
    case RobotProtocol::PacketID::HELLO:
    case RobotProtocol::PacketID::ROUTE_COMMAND:
    case RobotProtocol::PacketID::CANCEL_ROUTE:
        break;
    }
}

void RobotSession::SendHelloAck(const RobotProtocol::HelloAckPayload& payload)
{
    OutputMemoryStream payloadStream;
    RobotProtocol::WriteHelloAckPayload(payloadStream, payload);
    SendRobotPacket(RobotProtocol::PacketID::HELLO_ACK, payload.assignedAgvID, payloadStream);
}

void RobotSession::SendRoute(const RoutePacket& routePacket)
{
    RobotProtocol::RouteCommandPayload payload;
    payload.routeID = m_NextRouteID++;
    payload.nodes = routePacket.nodes;

    OutputMemoryStream payloadStream;
    RobotProtocol::WriteRouteCommandPayload(payloadStream, payload);
    SendRobotPacket(RobotProtocol::PacketID::ROUTE_COMMAND, routePacket.agvID, payloadStream);
}

void RobotSession::SendCancelRoute(uint32_t agvID)
{
    OutputMemoryStream payloadStream;
    SendRobotPacket(RobotProtocol::PacketID::CANCEL_ROUTE, agvID, payloadStream);
}

void RobotSession::SendPong(uint32_t timestampMs)
{
    RobotProtocol::TimePayload payload;
    payload.timestampMs = timestampMs;

    OutputMemoryStream payloadStream;
    RobotProtocol::WriteTimePayload(payloadStream, payload);
    SendRobotPacket(RobotProtocol::PacketID::PONG, m_AgvID, payloadStream);
}

StatusPacket RobotSession::GetStatus() const
{
    return m_LastStatus;
}

bool RobotSession::HasEvent() const
{
    return !m_EventQueue.empty();
}

ControllerEvent RobotSession::PopEvent()
{
    if (m_EventQueue.empty())
        return ControllerEvent{ ControllerEventType::NONE, 0, 0 };

    ControllerEvent event = m_EventQueue.front();
    m_EventQueue.pop();
    return event;
}

uint32_t RobotSession::NextSequence()
{
    return m_NextOutgoingSequence++;
}

void RobotSession::SendRobotPacket(RobotProtocol::PacketID packetID, uint32_t agvID, OutputMemoryStream& payloadStream)
{
    if (!m_TCPSession) 
        return;

    OutputMemoryStream bodyStream;
    RobotProtocol::WritePacketBodyHeader(bodyStream, packetID, agvID, NextSequence());
    bodyStream.Write(payloadStream.GetBuffer(), payloadStream.GetLength());

    m_TCPSession->SendPacket(bodyStream);
}
