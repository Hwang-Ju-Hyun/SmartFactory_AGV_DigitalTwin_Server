#pragma once
#include "MemoryStream.hpp"
#include "MovementSimulator.hpp"
#include "PacketSerializer.hpp"
#include "TCPSession.hpp"
#include "TCPSocket.hpp"
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

struct FakeRobotTrajectoryPreviewExpectation
{
    bool enabled = false;
    uint32_t startNodeID = 0;
    uint32_t finalNodeID = 0;
    std::chrono::milliseconds deadline{ 0 };
};

class FakeRobotApp
{
public:
    FakeRobotApp(
        std::string serverAddress,
        uint32_t requestedAgvID,
        FakeRobotTrajectoryPreviewExpectation expectation = {});

    int Run();

private:
    bool Connect();
    void InstallPacketHandler();

    void HandlePacket(InputMemoryStream& inStream);
    void HandleHelloAck(InputMemoryStream& inStream);
    void HandleRouteCommand(InputMemoryStream& inStream);
    void HandleTrajectoryCommand(InputMemoryStream& inStream);
    void HandleCancelRoute();
    void HandlePing(InputMemoryStream& inStream);

    void UpdateSimulation(float dt);
    void SendPeriodicStatus(std::chrono::steady_clock::time_point now);

    void SendHello();
    void SendStatus(const StatusPacket& status);
    void SendArrived(uint32_t currentNodeID);
    void SendPong(uint32_t timestampMs);
    void SendRobotPacket(RobotProtocol::PacketID packetID, uint32_t agvID, OutputMemoryStream& payloadStream);

    bool IsExpectationMode() const;
    bool IsExpectationComplete() const;
    void FailExpectation(std::string reason);
    int ReportExpectationFailure() const;

private:
    std::string m_ServerAddress;
    uint32_t m_AgvID = 0;
    bool m_Accepted = false;
    bool m_Running = true;
    bool m_RouteActive = false;

    uint32_t m_NextSequence = 1;
    float m_SimulatorTime = 0.0f;

    TCPSocketPtr m_Socket;
    TCPSessionPtr m_Session;
    std::vector<TCPSocketPtr> m_ReadSockets;
    std::vector<TCPSocketPtr> m_ReadableSockets;

    MovementSimulator m_Simulator;
    RobotProtocol::RouteCommandPayload m_CurrentRoute;

    FakeRobotTrajectoryPreviewExpectation m_Expectation;
    bool m_ExpectationHelloAckReceived = false;
    bool m_ExpectationTrajectoryReceived = false;
    bool m_ExpectationFailed = false;
    std::string m_ExpectationFailure;
    std::chrono::steady_clock::time_point m_ExpectationDeadline;

    std::chrono::steady_clock::time_point m_LastUpdate;
    std::chrono::steady_clock::time_point m_LastStatus;
};
