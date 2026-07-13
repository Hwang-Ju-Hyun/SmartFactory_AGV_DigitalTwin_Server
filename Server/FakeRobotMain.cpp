#include "MemoryStream.hpp"
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

    void SendStatus(TCPSessionPtr session, uint32_t agvID, uint32_t currentNodeID, float progress, uint32_t& sequence)
    {
        RobotProtocol::StatusPayload payload;
        payload.currentNodeID = currentNodeID;
        payload.currentLinkID = 0;
        payload.progress = progress;
        payload.x = progress * 10.0f;
        payload.z = static_cast<float>(agvID);
        payload.heading = 0.0f;
        payload.velocity = progress > 0.0f && progress < 1.0f ? 1.0f : 0.0f;
        payload.battery = 99.0f;
        payload.state = RobotProtocol::RobotState::MOVING;

        OutputMemoryStream payloadStream;
        RobotProtocol::WriteStatusPayload(payloadStream, payload);
        SendRobotPacket(session, RobotProtocol::PacketID::STATUS, agvID, sequence, payloadStream);
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
    uint32_t currentNodeID = 0;
    size_t nextNodeIndex = 0;
    float progress = 0.0f;
    RobotProtocol::RouteCommandPayload currentRoute;

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
                nextNodeIndex = routeActive ? 1 : 0;
                progress = 0.0f;
                currentNodeID = currentRoute.nodes.empty() ? currentNodeID : currentRoute.nodes.front().nodeID;
                std::cout << "[FakeRobot] ROUTE routeID=" << currentRoute.routeID
                          << " nodes=" << currentRoute.nodes.size() << "\n";
            }
            break;
        }
        case RobotProtocol::PacketID::CANCEL_ROUTE:
            routeActive = false;
            progress = 0.0f;
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

    auto lastStatus = std::chrono::steady_clock::now();
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
        float dt = std::chrono::duration<float>(now - lastStatus).count();
        if (dt < 0.5f)
            continue;

        lastStatus = now;

        if (!accepted)
            continue;

        if (routeActive && nextNodeIndex < currentRoute.nodes.size())
        {
            progress += 0.25f;
            if (progress >= 1.0f)
            {
                currentNodeID = currentRoute.nodes[nextNodeIndex].nodeID;
                SendStatus(session, agvID, currentNodeID, 1.0f, sequence);
                SendArrived(session, agvID, currentNodeID, sequence);
                std::cout << "[FakeRobot] ARRIVED node=" << currentNodeID << "\n";

                ++nextNodeIndex;
                progress = 0.0f;
                if (nextNodeIndex >= currentRoute.nodes.size())
                    routeActive = false;
            }
            else
            {
                SendStatus(session, agvID, currentNodeID, progress, sequence);
            }
        }
        else
        {
            SendStatus(session, agvID, currentNodeID, 0.0f, sequence);
        }
    }

    return accepted ? 0 : 1;
}
