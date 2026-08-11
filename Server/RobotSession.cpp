#include "RobotSession.hpp"
#include "BezierFollower.hpp"
#include "Map.hpp"
#include <cmath>
#include <iostream>
#include <utility>

namespace
{
    constexpr float kCoordinateEpsilon = 0.001f;

    bool TryGetMapNode(uint32_t nodeID, MapNode& outNode)
    {
        if (nodeID == 0)
            return false;

        const auto nodes = MapManager::GetInstance().GetNodes();
        auto it = nodes.find(nodeID);
        if (it == nodes.end())
            return false;

        outNode = it->second;
        return true;
    }

    bool TryApplyNodeBasedPose(const RobotProtocol::StatusPayload& payload, StatusPacket& status)
    {
        MapNode currentNode;
        if (!TryGetMapNode(payload.currentNodeID, currentNode))
            return false;

        MapNode targetNode;
        if (TryGetMapNode(payload.currentLinkID, targetNode))
        {
            MapLink link = MapManager::GetInstance().FindLink(payload.currentNodeID, payload.currentLinkID);
            const float progress = std::max(0.0f, std::min(payload.progress, 1.0f));
            Vector2 position = BezierFollower::Evaluate(currentNode, targetNode, link, progress);

            status.x = position.x;
            status.z = position.z;
            status.heading = BezierFollower::Heading(currentNode, targetNode, link, progress);
            return true;
        }

        status.x = currentNode.m_PosX;
        status.z = currentNode.m_PosZ;
        status.heading = payload.heading;
        return true;
    }

    bool HasExplicitPayloadPosition(const RobotProtocol::StatusPayload& payload)
    {
        return std::abs(payload.x) > kCoordinateEpsilon || std::abs(payload.z) > kCoordinateEpsilon;
    }
}

RobotSession::RobotSession(TCPSessionPtr session, uint32_t agvID, uint32_t clientCapabilities)
    : m_TCPSession(session)
    , m_AgvID(agvID)
    , m_ClientCapabilities(clientCapabilities)
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
        m_LastStatus.velocity = payload.velocity;
        m_LastStatus.battery = payload.battery;

        const bool usedNodeBasedPose = TryApplyNodeBasedPose(payload, m_LastStatus);
        if (!usedNodeBasedPose && (!m_HasStatus || HasExplicitPayloadPosition(payload)))
        {
            m_LastStatus.x = payload.x;
            m_LastStatus.z = payload.z;
            m_LastStatus.heading = payload.heading;
        }

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
    case RobotProtocol::PacketID::TRAJECTORY_COMMAND:
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

    std::cout << "[RobotProtocol] Send ROUTE_COMMAND agvID=" << routePacket.agvID
              << " routeID=" << payload.routeID
              << " nodes=" << payload.nodes.size() << "\n";
}

bool RobotSession::SendTrajectory(RobotProtocol::TrajectoryCommandPayload payload)
{
    return SendTrajectoryForCapability(
        std::move(payload), RobotProtocol::CAPABILITY_TRAJECTORY_COMMAND, "execution");
}

bool RobotSession::SendTrajectoryPreview(RobotProtocol::TrajectoryCommandPayload payload)
{
    if (!SupportsTrajectoryPreviewOnly())
    {
        std::cout << "[RobotProtocol] TRAJECTORY_COMMAND preview not sent: AGV "
                  << m_AgvID << " is not a preview-only client\n";
        return false;
    }
    for (const RobotProtocol::TrajectoryWaypoint& waypoint : payload.waypoints)
    {
        if (std::abs(waypoint.targetSpeedMmPerSecond) > 0.001f)
        {
            std::cout << "[RobotProtocol] TRAJECTORY_COMMAND preview not sent: AGV "
                      << m_AgvID << " payload contains non-zero target speed\n";
            return false;
        }
    }
    return SendTrajectoryForCapability(
        std::move(payload), RobotProtocol::CAPABILITY_TRAJECTORY_PREVIEW, "preview");
}

bool RobotSession::SendTrajectoryForCapability(
    RobotProtocol::TrajectoryCommandPayload payload,
    uint32_t requiredCapability,
    const char* capabilityName)
{
    if ((m_ClientCapabilities & requiredCapability) == 0)
    {
        std::cout << "[RobotProtocol] TRAJECTORY_COMMAND not sent: AGV " << m_AgvID
                  << " did not advertise " << capabilityName << " capability\n";
        return false;
    }

    if (payload.routeID == 0)
        payload.routeID = m_NextRouteID++;

    OutputMemoryStream payloadStream;
    if (!RobotProtocol::WriteTrajectoryCommandPayload(payloadStream, payload))
    {
        std::cout << "[RobotProtocol] TRAJECTORY_COMMAND not sent: invalid payload for AGV "
                  << m_AgvID << "\n";
        return false;
    }

    SendRobotPacket(RobotProtocol::PacketID::TRAJECTORY_COMMAND, m_AgvID, payloadStream);
    std::cout << "[RobotProtocol] Send TRAJECTORY_COMMAND agvID=" << m_AgvID
              << " routeID=" << payload.routeID
              << " waypoints=" << payload.waypoints.size() << "\n";
    return true;
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

void RobotSession::PrimeStatus(float x, float z, float heading)
{
    m_LastStatus.x = x;
    m_LastStatus.z = z;
    m_LastStatus.heading = heading;
    m_LastStatus.agvID = m_AgvID;
    m_LastStatus.battery = 100.0f;
    m_HasStatus = true;
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
