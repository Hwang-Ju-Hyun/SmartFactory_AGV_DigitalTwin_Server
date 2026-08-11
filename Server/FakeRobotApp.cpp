#include "FakeRobotApp.hpp"
#include "SocketAddressFactory.hpp"
#include "SocketUtil.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <utility>

namespace
{
    constexpr float kTrajectoryStartToleranceMm = 1.0f;
}

FakeRobotApp::FakeRobotApp(
    std::string serverAddress,
    uint32_t requestedAgvID,
    FakeRobotTrajectoryPreviewExpectation expectation)
    : m_ServerAddress(std::move(serverAddress))
    , m_AgvID(requestedAgvID)
    , m_Simulator(requestedAgvID, 0.0f, 0.0f, 0.0f)
    , m_Expectation(std::move(expectation))
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
    if (IsExpectationMode())
    {
        m_ExpectationDeadline = m_LastUpdate + m_Expectation.deadline;
        std::cout << "[FakeRobot] Expecting trajectory preview start="
                  << m_Expectation.startNodeID
                  << " final=" << m_Expectation.finalNodeID
                  << " deadlineMs=" << m_Expectation.deadline.count() << "\n";
    }

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
                if (IsExpectationMode())
                {
                    FailExpectation("server disconnected before preview validation completed");
                    return ReportExpectationFailure();
                }
                return 0;
            }

            if (IsExpectationMode())
            {
                if (m_ExpectationFailed)
                    return ReportExpectationFailure();
                if (IsExpectationComplete())
                {
                    std::cout << "TRAJECTORY_PREVIEW_PASS"
                              << " start=" << m_Expectation.startNodeID
                              << " final=" << m_Expectation.finalNodeID << "\n";
                    return 0;
                }
            }
        }

        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - m_LastUpdate).count();
        m_LastUpdate = now;

        if (IsExpectationMode())
        {
            if (now >= m_ExpectationDeadline)
            {
                if (!m_ExpectationHelloAckReceived)
                    FailExpectation("deadline expired before accepted HELLO_ACK");
                else
                    FailExpectation("deadline expired before valid TRAJECTORY_COMMAND");
                return ReportExpectationFailure();
            }
            continue;
        }

        if (!m_Accepted)
            continue;

        UpdateSimulation(dt);
        SendPeriodicStatus(now);
    }

    if (IsExpectationMode())
    {
        if (!m_ExpectationFailed)
            FailExpectation("preview validation stopped before completion");
        return ReportExpectationFailure();
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
        if (IsExpectationMode())
            FailExpectation("invalid packet header");
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
    case RobotProtocol::PacketID::TRAJECTORY_COMMAND:
        HandleTrajectoryCommand(inStream);
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
    {
        if (IsExpectationMode())
            FailExpectation("invalid HELLO_ACK payload");
        return;
    }

    m_Accepted = payload.accepted != 0;
    m_AgvID = payload.assignedAgvID != 0 ? payload.assignedAgvID : m_AgvID;
    std::cout << "[FakeRobot] HELLO_ACK accepted=" << m_Accepted
              << " assignedAgvID=" << m_AgvID
              << " error=" << static_cast<uint16_t>(payload.errorCode) << "\n";

    if (IsExpectationMode())
    {
        if (payload.protocolVersion != RobotProtocol::kProtocolVersion)
        {
            FailExpectation("HELLO_ACK protocol version mismatch");
            return;
        }
        if (!m_Accepted)
        {
            FailExpectation("HELLO_ACK rejected the FakeRobot");
            return;
        }
        m_ExpectationHelloAckReceived = true;
    }

    if (!m_Accepted)
        m_Running = false;
}

void FakeRobotApp::HandleRouteCommand(InputMemoryStream& inStream)
{
    if (IsExpectationMode())
    {
        FailExpectation("unexpected ROUTE_COMMAND in trajectory-preview mode");
        return;
    }

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

void FakeRobotApp::HandleTrajectoryCommand(InputMemoryStream& inStream)
{
    RobotProtocol::TrajectoryCommandPayload trajectory;
    if (!RobotProtocol::ReadTrajectoryCommandPayload(inStream, trajectory))
    {
        std::cout << "[FakeRobot] Invalid TRAJECTORY_COMMAND payload\n";
        if (IsExpectationMode())
            FailExpectation("invalid TRAJECTORY_COMMAND payload");
        return;
    }

    const RobotProtocol::TrajectoryWaypoint& first = trajectory.waypoints.front();
    const RobotProtocol::TrajectoryWaypoint& last = trajectory.waypoints.back();
    const float startDistanceMm = std::hypot(first.forwardMm, first.leftMm);
    const bool startsNearOrigin = std::isfinite(startDistanceMm)
        && startDistanceMm <= kTrajectoryStartToleranceMm;
    const bool hasFinalFlag =
        (last.flags & RobotProtocol::TRAJECTORY_FLAG_FINAL) != 0;
    const bool endpointIDsMatch = !IsExpectationMode() ||
        (trajectory.startNodeID == m_Expectation.startNodeID &&
         trajectory.finalNodeID == m_Expectation.finalNodeID);
    const bool startSemanticsValid =
        first.nodeID == trajectory.startNodeID &&
        (first.flags & RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY) != 0;
    const uint8_t requiredFinalFlags =
        RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY |
        RobotProtocol::TRAJECTORY_FLAG_STOP |
        RobotProtocol::TRAJECTORY_FLAG_FINAL;
    const bool finalSemanticsValid =
        last.nodeID == trajectory.finalNodeID &&
        (last.flags & requiredFinalFlags) == requiredFinalFlags;

    bool valuesAreFinite = std::isfinite(trajectory.millimetersPerMapUnit)
        && trajectory.millimetersPerMapUnit > 0.0f;
    bool allTargetSpeedsAreZero = true;
    bool finalFlagOnlyOnLastWaypoint = true;
    for (size_t index = 0; index < trajectory.waypoints.size(); ++index)
    {
        const RobotProtocol::TrajectoryWaypoint& waypoint = trajectory.waypoints[index];
        valuesAreFinite = valuesAreFinite
            && std::isfinite(waypoint.forwardMm)
            && std::isfinite(waypoint.leftMm)
            && std::isfinite(waypoint.headingRad)
            && std::isfinite(waypoint.targetSpeedMmPerSecond)
            && waypoint.targetSpeedMmPerSecond >= 0.0f;
        const bool speedIsZero = IsExpectationMode()
            ? waypoint.targetSpeedMmPerSecond == 0.0f
            : std::abs(waypoint.targetSpeedMmPerSecond) <= 0.001f;
        allTargetSpeedsAreZero = allTargetSpeedsAreZero && speedIsZero;
        if (index + 1 != trajectory.waypoints.size() &&
            (waypoint.flags & RobotProtocol::TRAJECTORY_FLAG_FINAL) != 0)
            finalFlagOnlyOnLastWaypoint = false;
    }

    std::cout << "[FakeRobot] TRAJECTORY routeID=" << trajectory.routeID
              << " start=" << trajectory.startNodeID
              << " final=" << trajectory.finalNodeID
              << " waypoints=" << trajectory.waypoints.size()
              << " startNearOrigin=" << (startsNearOrigin ? 1 : 0)
              << " finalFlag=" << (hasFinalFlag ? 1 : 0)
              << " finiteValues=" << (valuesAreFinite ? 1 : 0)
              << " zeroSpeed=" << (allTargetSpeedsAreZero ? 1 : 0);
    if (IsExpectationMode())
    {
        std::cout << " endpointIDs=" << (endpointIDsMatch ? 1 : 0)
                  << " startSemantics=" << (startSemanticsValid ? 1 : 0)
                  << " finalSemantics=" << (finalSemanticsValid ? 1 : 0);
    }
    std::cout << "\n";

    const bool baseSemanticsValid = startsNearOrigin && hasFinalFlag &&
        valuesAreFinite && allTargetSpeedsAreZero;
    const bool expectationSemanticsValid = endpointIDsMatch &&
        startSemanticsValid && finalSemanticsValid && finalFlagOnlyOnLastWaypoint;
    if (!baseSemanticsValid ||
        (IsExpectationMode() && !expectationSemanticsValid))
    {
        std::cout << "[FakeRobot] Invalid TRAJECTORY_COMMAND semantics:"
                  << " startDistanceMm=" << startDistanceMm
                  << " finalFlag=" << (hasFinalFlag ? 1 : 0)
                  << " finiteValues=" << (valuesAreFinite ? 1 : 0)
                  << " zeroSpeed=" << (allTargetSpeedsAreZero ? 1 : 0)
                  << " endpointIDs=" << (endpointIDsMatch ? 1 : 0)
                  << " startSemantics=" << (startSemanticsValid ? 1 : 0)
                  << " finalSemantics=" << (finalSemanticsValid ? 1 : 0)
                  << " finalFlagOnlyLast=" << (finalFlagOnlyOnLastWaypoint ? 1 : 0)
                  << "\n";
        if (IsExpectationMode())
            FailExpectation("TRAJECTORY_COMMAND semantic validation failed");
        return;
    }

    if (IsExpectationMode())
    {
        if (!m_ExpectationHelloAckReceived)
        {
            FailExpectation("TRAJECTORY_COMMAND arrived before accepted HELLO_ACK");
            return;
        }
        m_ExpectationTrajectoryReceived = true;
    }

    // Protocol smoke coverage only. MovementSimulator and ARRIVED continue to
    // follow ROUTE_COMMAND so trajectory parsing cannot change legacy behavior.
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
    // FakeRobot parses only; it has no trajectory motion executor.
    payload.capabilities = RobotProtocol::CAPABILITY_TRAJECTORY_PREVIEW;

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

bool FakeRobotApp::IsExpectationMode() const
{
    return m_Expectation.enabled;
}

bool FakeRobotApp::IsExpectationComplete() const
{
    return m_ExpectationHelloAckReceived &&
        m_ExpectationTrajectoryReceived &&
        !m_ExpectationFailed;
}

void FakeRobotApp::FailExpectation(std::string reason)
{
    if (m_ExpectationFailed)
        return;
    m_ExpectationFailed = true;
    m_ExpectationFailure = std::move(reason);
}

int FakeRobotApp::ReportExpectationFailure() const
{
    std::cerr << "TRAJECTORY_PREVIEW_FAIL reason=" << m_ExpectationFailure << "\n";
    return 2;
}
