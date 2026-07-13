#include "MemoryStream.hpp"
#include "MovementSimulator.hpp"
#include "PacketSerializer.hpp"
#include "SocketAddressFactory.hpp"
#include "SocketUtil.hpp"
#include "TCPSession.hpp"
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace
{
    uint32_t NowMs()
    {
        using namespace std::chrono;
        return static_cast<uint32_t>(duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
    }

    void SendRobotPacket(TCPSessionPtr session, RobotProtocol::PacketID packetID, uint32_t agvID, uint32_t& sequence, OutputMemoryStream& payloadStream)
    {
        OutputMemoryStream bodyStream;
        RobotProtocol::WritePacketBodyHeader(bodyStream, packetID, agvID, sequence++);
        bodyStream.Write(payloadStream.GetBuffer(), payloadStream.GetLength());
        session->SendPacket(bodyStream);
    }

    void SendHello(TCPSessionPtr session, uint32_t requestedAgvID, uint32_t& sequence)
    {
        RobotProtocol::HelloPayload payload;
        payload.protocolVersion = RobotProtocol::kProtocolVersion;
        payload.clientType = RobotProtocol::ClientType::FAKE_ROBOT;
        payload.requestedAgvID = requestedAgvID;

        OutputMemoryStream payloadStream;
        RobotProtocol::WriteHelloPayload(payloadStream, payload);
        SendRobotPacket(session, RobotProtocol::PacketID::HELLO, requestedAgvID, sequence, payloadStream);
    }

    void SendStatus(TCPSessionPtr session, const StatusPacket& status, uint32_t& sequence)
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
        SendRobotPacket(session, RobotProtocol::PacketID::STATUS, status.agvID, sequence, payloadStream);
    }

    void SendArrived(TCPSessionPtr session, uint32_t agvID, uint32_t currentNodeID, uint32_t& sequence)
    {
        RobotProtocol::ArrivedPayload payload;
        payload.currentNodeID = currentNodeID;

        OutputMemoryStream payloadStream;
        RobotProtocol::WriteArrivedPayload(payloadStream, payload);
        SendRobotPacket(session, RobotProtocol::PacketID::ARRIVED, agvID, sequence, payloadStream);
    }

    void SendPong(TCPSessionPtr session, uint32_t agvID, uint32_t timestampMs, uint32_t& sequence)
    {
        RobotProtocol::TimePayload payload;
        payload.timestampMs = timestampMs;

        OutputMemoryStream payloadStream;
        RobotProtocol::WriteTimePayload(payloadStream, payload);
        SendRobotPacket(session, RobotProtocol::PacketID::PONG, agvID, sequence, payloadStream);
    }
}

int main(int argc, char** argv)
{
    std::cout.setf(std::ios::unitbuf);

    std::string serverAddress = argc >= 2 ? argv[1] : "127.0.0.1:6666";
    uint32_t agvID = argc >= 3 ? static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 10)) : 1;

    SocketAddressPtr address = SocketAddressFactory::CreateIPv4FromString(serverAddress);
    if (!address)
    {
        std::cerr << "[FakeRobot] Invalid server address: " << serverAddress << "\n";
        return 1;
    }

    TCPSocketPtr socket = SocketUtil::CreateTCPSocket(AF_INET);
    if (socket->Connect(*address) == ERROR)
    {
        std::cerr << "[FakeRobot] Failed to connect: " << serverAddress << "\n";
        return 1;
    }

    TCPSessionPtr session = std::make_shared<TCPSession>();
    session->SetSocket(socket);

    bool accepted = false;
    bool running = true;
    bool routeActive = false;
    uint32_t sequence = 1;
    float simulatorTime = 0.0f;
    auto lastUpdate = std::chrono::steady_clock::now();
    auto lastStatus = lastUpdate;
    RobotProtocol::RouteCommandPayload currentRoute;
    MovementSimulator simulator(agvID, 0.0f, 0.0f, 0.0f);

    session->OnPacketReceived = [&](InputMemoryStream& inStream)
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
        {
            RobotProtocol::HelloAckPayload payload;
            if (RobotProtocol::ReadHelloAckPayload(inStream, payload))
            {
                accepted = payload.accepted != 0;
                agvID = payload.assignedAgvID != 0 ? payload.assignedAgvID : agvID;
                std::cout << "[FakeRobot] HELLO_ACK accepted=" << accepted
                          << " assignedAgvID=" << agvID
                          << " error=" << static_cast<uint16_t>(payload.errorCode) << "\n";
                if (!accepted)
                    running = false;
            }
            break;
        }
        case RobotProtocol::PacketID::ROUTE_COMMAND:
        {
            if (RobotProtocol::ReadRouteCommandPayload(inStream, currentRoute))
            {
                routeActive = currentRoute.nodes.size() >= 2;
                RoutePacket routePacket;
                routePacket.agvID = agvID;
                routePacket.nodes = currentRoute.nodes;
                simulator.LoadRoute(routePacket);

                if (!routePacket.nodes.empty())
                    simulatorTime = routePacket.nodes.front().departureTime;

                std::cout << "[FakeRobot] ROUTE routeID=" << currentRoute.routeID
                          << " nodes=" << currentRoute.nodes.size() << "\n";
            }
            break;
        }
        case RobotProtocol::PacketID::CANCEL_ROUTE:
            routeActive = false;
            simulator.CancelRoute();
            std::cout << "[FakeRobot] CANCEL_ROUTE\n";
            break;
        case RobotProtocol::PacketID::PING:
        {
            RobotProtocol::TimePayload payload;
            if (RobotProtocol::ReadTimePayload(inStream, payload))
                SendPong(session, agvID, payload.timestampMs, sequence);
            break;
        }
        default:
            break;
        }
    };

    SendHello(session, agvID, sequence);
    std::cout << "[FakeRobot] Connected to " << serverAddress << " as requested AGV " << agvID << "\n";

    std::vector<TCPSocketPtr> readSockets{ socket };
    std::vector<TCPSocketPtr> readableSockets;

    while (running)
    {
        timeval timeoutValue;
        timeoutValue.tv_sec = 0;
        timeoutValue.tv_usec = 100000;

        int selected = SocketUtil::Select(&readSockets, &readableSockets, nullptr, nullptr, nullptr, nullptr, &timeoutValue);
        if (selected > 0)
        {
            if (!session->ProcessIncomingData())
            {
                std::cout << "[FakeRobot] Server disconnected\n";
                return 0;
            }
        }

        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - lastUpdate).count();
        lastUpdate = now;

        if (!accepted)
            continue;

        simulatorTime += dt;

        if (routeActive)
        {
            simulator.Update(dt, simulatorTime);
            while (simulator.HasEvent())
            {
                MovementEvent event = simulator.PopEvent();
                if (event.type == MovementEventType::ARRIVED)
                {
                    SendArrived(session, agvID, event.nodeID, sequence);
                    std::cout << "[FakeRobot] ARRIVED node=" << event.nodeID << "\n";
                }
            }

            routeActive = !simulator.IsRouteComplete();
        }

        const float statusInterval = std::chrono::duration<float>(now - lastStatus).count();
        if (statusInterval >= 0.1f)
        {
            StatusPacket status = simulator.GetStatus();
            status.agvID = agvID;
            SendStatus(session, status, sequence);
            lastStatus = now;
        }
    }

    return accepted ? 0 : 1;
}
