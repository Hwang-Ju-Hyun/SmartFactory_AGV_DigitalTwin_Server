#include "FakeRobotApp.hpp"
#include "SocketAddressFactory.hpp"
#include "SocketUtil.hpp"
#include <cstdlib>
#include <iostream>

FakeRobotApp::FakeRobotApp(std::string serverAddress, uint32_t requestedAgvID)
    : m_ServerAddress(std::move(serverAddress))
    , m_AgvID(requestedAgvID)
    , m_Simulator(requestedAgvID, 0.0f, 0.0f, 0.0f)
{
}

int FakeRobotApp::Run()
{
    std::cout.setf(std::ios::unitbuf);

    if (!Connect())
        return 1;

    InstallPacketHandler();
    SendHello();
    std::cout << "[FakeRobot] Connected to " << m_ServerAddress
              << " as requested AGV " << m_AgvID << "\n";

    m_LastUpdate = std::chrono::steady_clock::now();
    m_LastStatus = m_LastUpdate;

    while (m_Running)
    {
        timeval timeoutValue;
        timeoutValue.tv_sec = 0;
        timeoutValue.tv_usec = 100000;

        int selected = SocketUtil::Select(&m_ReadSockets, &m_ReadableSockets, nullptr, nullptr, nullptr, nullptr, &timeoutValue);
        if (selected > 0)
        {
            if (!m_Session->ProcessIncomingData())
            {
                std::cout << "[FakeRobot] Server disconnected\n";
                return 0;
            }
        }

        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - m_LastUpdate).count();
        m_LastUpdate = now;

        if (!m_Accepted)
            continue;

        UpdateSimulation(dt);
        SendPeriodicStatus(now);
    }

    return m_Accepted ? 0 : 1;
}

bool FakeRobotApp::Connect()
{
    SocketAddressPtr address = SocketAddressFactory::CreateIPv4FromString(m_ServerAddress);
    if (!address)
    {
        std::cerr << "[FakeRobot] Invalid server address: " << m_ServerAddress << "\n";
        return false;
    }

    m_Socket = SocketUtil::CreateTCPSocket(AF_INET);
    if (m_Socket->Connect(*address) == ERROR)
    {
        std::cerr << "[FakeRobot] Failed to connect: " << m_ServerAddress << "\n";
        return false;
    }

    m_Session = std::make_shared<TCPSession>();
    m_Session->SetSocket(m_Socket);
    m_ReadSockets = { m_Socket };
    return true;
}

void FakeRobotApp::InstallPacketHandler()
{
    m_Session->OnPacketReceived = [this](InputMemoryStream& inStream)
    {
        HandlePacket(inStream);
    };
}

void FakeRobotApp::HandlePacket(InputMemoryStream& inStream)
{
    RobotProtocol::PacketBodyHeader header;
    if (!RobotProtocol::ReadPacketBodyHeader(inStream, header))
    {
        std::cout << "[FakeRobot] Invalid packet header\n";
        return;
    }

    const RobotProtocol::PacketID packetID = static_cast<RobotProtocol::PacketID>(header.packetID);
    switch (packetID)
    {
    case RobotProtocol::PacketID::HELLO_ACK:
        HandleHelloAck(inStream);
        break;
    case RobotProtocol::PacketID::ROUTE_COMMAND:
        HandleRouteCommand(inStream);
        break;
    case RobotProtocol::PacketID::CANCEL_ROUTE:
        HandleCancelRoute();
        break;
    case RobotProtocol::PacketID::PING:
        HandlePing(inStream);
        break;
    default:
        break;
    }
}

void FakeRobotApp::HandleHelloAck(InputMemoryStream& inStream)
{
    RobotProtocol::HelloAckPayload payload;
    if (!RobotProtocol::ReadHelloAckPayload(inStream, payload))
        return;

    m_Accepted = payload.accepted != 0;
    m_AgvID = payload.assignedAgvID != 0 ? payload.assignedAgvID : m_AgvID;
    std::cout << "[FakeRobot] HELLO_ACK accepted=" << m_Accepted
              << " assignedAgvID=" << m_AgvID
              << " error=" << static_cast<uint16_t>(payload.errorCode) << "\n";

    if (!m_Accepted)
        m_Running = false;
}

void FakeRobotApp::HandleRouteCommand(InputMemoryStream& inStream)
{
    if (!RobotProtocol::ReadRouteCommandPayload(inStream, m_CurrentRoute))
        return;

    m_RouteActive = m_CurrentRoute.nodes.size() >= 2;

    RoutePacket routePacket;
    routePacket.agvID = m_AgvID;
    routePacket.nodes = m_CurrentRoute.nodes;
    m_Simulator.LoadRoute(routePacket);

    if (!routePacket.nodes.empty())
        m_SimulatorTime = routePacket.nodes.front().departureTime;

    std::cout << "[FakeRobot] ROUTE routeID=" << m_CurrentRoute.routeID
              << " nodes=" << m_CurrentRoute.nodes.size() << "\n";
}

void FakeRobotApp::HandleCancelRoute()
{
    m_RouteActive = false;
    m_Simulator.CancelRoute();
    std::cout << "[FakeRobot] CANCEL_ROUTE\n";
}

void FakeRobotApp::HandlePing(InputMemoryStream& inStream)
{
    RobotProtocol::TimePayload payload;
    if (RobotProtocol::ReadTimePayload(inStream, payload))
        SendPong(payload.timestampMs);
}

void FakeRobotApp::UpdateSimulation(float dt)
{
    m_SimulatorTime += dt;

    if (!m_RouteActive)
        return;

    m_Simulator.Update(dt, m_SimulatorTime);
    while (m_Simulator.HasEvent())
    {
        MovementEvent event = m_Simulator.PopEvent();
        if (event.type == MovementEventType::ARRIVED)
        {
            SendArrived(event.nodeID);
            std::cout << "[FakeRobot] ARRIVED node=" << event.nodeID << "\n";
        }
    }

    m_RouteActive = !m_Simulator.IsRouteComplete();
}

void FakeRobotApp::SendPeriodicStatus(std::chrono::steady_clock::time_point now)
{
    const float statusInterval = std::chrono::duration<float>(now - m_LastStatus).count();
    if (statusInterval < 0.1f)
        return;

    StatusPacket status = m_Simulator.GetStatus();
    status.agvID = m_AgvID;
    SendStatus(status);
    m_LastStatus = now;
}

void FakeRobotApp::SendHello()
{
    RobotProtocol::HelloPayload payload;
    payload.protocolVersion = RobotProtocol::kProtocolVersion;
    payload.clientType = RobotProtocol::ClientType::FAKE_ROBOT;
    payload.requestedAgvID = m_AgvID;

    OutputMemoryStream payloadStream;
    RobotProtocol::WriteHelloPayload(payloadStream, payload);
    SendRobotPacket(RobotProtocol::PacketID::HELLO, m_AgvID, payloadStream);
}

void FakeRobotApp::SendStatus(const StatusPacket& status)
{
    RobotProtocol::StatusPayload payload;
    payload.currentNodeID = 0;
    payload.currentLinkID = status.currentLinkID;
    payload.progress = status.progress;
    payload.x = status.x;
    payload.z = status.z;
    payload.heading = status.heading;
    payload.velocity = status.velocity;
    payload.battery = status.battery;
    payload.state = status.velocity > 0.001f ? RobotProtocol::RobotState::MOVING : RobotProtocol::RobotState::IDLE;

    OutputMemoryStream payloadStream;
    RobotProtocol::WriteStatusPayload(payloadStream, payload);
    SendRobotPacket(RobotProtocol::PacketID::STATUS, status.agvID, payloadStream);
}

void FakeRobotApp::SendArrived(uint32_t currentNodeID)
{
    RobotProtocol::ArrivedPayload payload;
    payload.currentNodeID = currentNodeID;

    OutputMemoryStream payloadStream;
    RobotProtocol::WriteArrivedPayload(payloadStream, payload);
    SendRobotPacket(RobotProtocol::PacketID::ARRIVED, m_AgvID, payloadStream);
}

void FakeRobotApp::SendPong(uint32_t timestampMs)
{
    RobotProtocol::TimePayload payload;
    payload.timestampMs = timestampMs;

    OutputMemoryStream payloadStream;
    RobotProtocol::WriteTimePayload(payloadStream, payload);
    SendRobotPacket(RobotProtocol::PacketID::PONG, m_AgvID, payloadStream);
}

void FakeRobotApp::SendRobotPacket(RobotProtocol::PacketID packetID, uint32_t agvID, OutputMemoryStream& payloadStream)
{
    OutputMemoryStream bodyStream;
    RobotProtocol::WritePacketBodyHeader(bodyStream, packetID, agvID, m_NextSequence++);
    bodyStream.Write(payloadStream.GetBuffer(), payloadStream.GetLength());
    m_Session->SendPacket(bodyStream);
}
