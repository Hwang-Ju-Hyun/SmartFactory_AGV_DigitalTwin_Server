#include "NetworkManagerServer.hpp"
#include "MemoryStream.hpp"
#include "ClientProxy.hpp"
#include "TCPSession.hpp"
#include "LinkingContext.cpp"
#include "ObjectRegistry.hpp"
#include "RoboServer.hpp"
#include "ReplicationManagerServer.hpp"
#include "Map.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "PathFinder.hpp"
#include <cassert>
#include "AGVManager.hpp"
#include "WarehouseManager.hpp"
#include "TaskManager.hpp"
#include "RoutePlanner.hpp"
#include "Map.hpp"
#include "Event.hpp"
#include "ReservationTable.hpp"
#include "RobotManager.hpp"
#include "UnityRobotController.hpp"
#include "ESP32RobotController.hpp"
#include "OccupancyProvider.hpp"
#include "PacketSerializer.hpp"
#include "TrajectoryBuilder.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string_view>
#include <utility>

std::unique_ptr<NetworkManagerServer> NetworkManagerServer::sInstance=nullptr;

uint32_t NetworkManagerServer::nextSessionID=1;

namespace
{
    constexpr uint32_t kPhysicalDemoAgvID = 1;
    constexpr uint32_t kPhysicalDemoStartNodeID = 1;
    constexpr uint32_t kPhysicalDemoTargetNodeID = 2;
    constexpr uint32_t kTrajectoryDemoAgvID = 1;
    constexpr uint32_t kTrajectoryDemoStartNodeID = 1;
    constexpr uint32_t kTrajectoryDemoTargetNodeID = 4;
    constexpr float kTrajectoryDemoScaleMmPerMapUnit = 60.0f;
    constexpr float kTrajectoryDemoSpacingMm = 20.0f;
    constexpr float kTrajectoryDemoAssumedStartHeadingRad = 3.14159265358979323846f;
    constexpr float kTrajectoryRaisedWheelSpeedMmPerSecond = 80.0f;
    constexpr uint32_t kPhysicalFleetAgvID = 1;
    constexpr uint32_t kPhysicalFleetStartNodeID = 1;
    constexpr float kPhysicalFleetScaleMmPerMapUnit = 50.0f;
    constexpr float kPhysicalFleetCruiseSpeedMmPerSecond = 80.0f;
    constexpr float kVisionScaleMmPerMapUnit = 50.0f;
    constexpr float kVisionLocalOriginServerX = 50.0f;
    constexpr float kVisionLocalOriginServerZ = -36.0f;
    constexpr float kVisionAllowedMapMarginMm = 100.0f;
    constexpr uint64_t kCorrectionMaximumVisionAgeMs = 200;
    constexpr uint64_t kCorrectionMeasurementWaitMs = 2500;
    constexpr uint64_t kCorrectionReportWaitMs = 10000;
    constexpr uint32_t kVisionAcceptedLogInterval = 30;
    constexpr float kDegreesToRadians =
        3.14159265358979323846f / 180.0f;

    struct ExpectedVisionMapNode
    {
        uint32_t id;
        float x;
        float z;
    };

    constexpr std::array<ExpectedVisionMapNode, 15> kExpectedVisionMapNodes{{
        {1, 50.0f, -36.0f}, {2, 57.0f, -36.0f},
        {3, 64.0f, -36.0f}, {4, 71.0f, -36.0f},
        {5, 78.0f, -36.0f}, {6, 50.0f, -29.0f},
        {7, 57.0f, -29.0f}, {8, 64.0f, -29.0f},
        {9, 71.0f, -29.0f}, {10, 78.0f, -29.0f},
        {11, 50.0f, -22.0f}, {12, 57.0f, -22.0f},
        {13, 64.0f, -22.0f}, {14, 71.0f, -22.0f},
        {15, 78.0f, -22.0f}
    }};

    bool MatchesExpectedVisionMapContract(
        const std::unordered_map<uint32_t, MapNode>& nodes)
    {
        if (nodes.size() != kExpectedVisionMapNodes.size())
            return false;

        for (const ExpectedVisionMapNode& expected : kExpectedVisionMapNodes)
        {
            const auto nodeIt = nodes.find(expected.id);
            if (nodeIt == nodes.end() ||
                std::abs(nodeIt->second.m_PosX - expected.x) > 0.001f ||
                std::abs(nodeIt->second.m_PosZ - expected.z) > 0.001f)
            {
                return false;
            }
        }
        return true;
    }

    uint64_t MonotonicMilliseconds()
    {
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
    }

    const char* VisionStoreResultName(VisionObservationStoreResult result)
    {
        switch (result)
        {
        case VisionObservationStoreResult::ACCEPTED: return "ACCEPTED";
        case VisionObservationStoreResult::AGV_REGISTRATION_CHECK_UNAVAILABLE: return "AGV_REGISTRATION_CHECK_UNAVAILABLE";
        case VisionObservationStoreResult::UNKNOWN_AGV: return "UNKNOWN_AGV";
        case VisionObservationStoreResult::INVALID_SOURCE_ID: return "INVALID_SOURCE_ID";
        case VisionObservationStoreResult::SOURCE_ID_MISMATCH: return "SOURCE_ID_MISMATCH";
        case VisionObservationStoreResult::INVALID_SESSION_ID: return "INVALID_SESSION_ID";
        case VisionObservationStoreResult::INVALID_SEQUENCE: return "INVALID_SEQUENCE";
        case VisionObservationStoreResult::INVALID_TRACKING_STATE: return "INVALID_TRACKING_STATE";
        case VisionObservationStoreResult::INVALID_VERIFICATION_STATE: return "INVALID_VERIFICATION_STATE";
        case VisionObservationStoreResult::MEASURED_NOT_VERIFIED: return "MEASURED_NOT_VERIFIED";
        case VisionObservationStoreResult::INVALID_CALIBRATION_ID: return "INVALID_CALIBRATION_ID";
        case VisionObservationStoreResult::CALIBRATION_ID_MISMATCH: return "CALIBRATION_ID_MISMATCH";
        case VisionObservationStoreResult::INVALID_MAP_CONTRACT_ID: return "INVALID_MAP_CONTRACT_ID";
        case VisionObservationStoreResult::MAP_CONTRACT_ID_MISMATCH: return "MAP_CONTRACT_ID_MISMATCH";
        case VisionObservationStoreResult::INVALID_POSE_CONTRACT_ID: return "INVALID_POSE_CONTRACT_ID";
        case VisionObservationStoreResult::POSE_CONTRACT_ID_MISMATCH: return "POSE_CONTRACT_ID_MISMATCH";
        case VisionObservationStoreResult::INVALID_STATE_POSE_COMBINATION: return "INVALID_STATE_POSE_COMBINATION";
        case VisionObservationStoreResult::NON_FINITE_POSE: return "NON_FINITE_POSE";
        case VisionObservationStoreResult::HEADING_OUT_OF_RANGE: return "HEADING_OUT_OF_RANGE";
        case VisionObservationStoreResult::INVALID_QUALITY_METADATA: return "INVALID_QUALITY_METADATA";
        case VisionObservationStoreResult::OUT_OF_MAP: return "OUT_OF_MAP";
        case VisionObservationStoreResult::STALE_REPORTED_AGE: return "STALE_REPORTED_AGE";
        case VisionObservationStoreResult::RECEIVE_TIME_IN_FUTURE: return "RECEIVE_TIME_IN_FUTURE";
        case VisionObservationStoreResult::STALE_RECEIVE_TIME: return "STALE_RECEIVE_TIME";
        case VisionObservationStoreResult::DUPLICATE_OR_OUT_OF_ORDER_SEQUENCE: return "DUPLICATE_OR_OUT_OF_ORDER_SEQUENCE";
        }
        return "UNKNOWN";
    }
}

NetworkManagerServer::NetworkManagerServer(
    ServerRunMode _runMode,
    VisionObservationServerConfig _visionConfig)
    : m_TotalElapsedServerTime(0.0f)
    , m_RunMode(_runMode)
    , m_VisionConfig(std::move(_visionConfig))
{
    m_LinkingContext = new LinkingContext;
}

void NetworkManagerServer::StaticInit(
    ServerRunMode _runMode,
    VisionObservationServerConfig _visionConfig)
{
    sInstance.reset(new NetworkManagerServer(_runMode, std::move(_visionConfig)));
    ObjectRegistry::sInstance->RegisterCreationFunction(ClassID::OBJ_AGV,RoboServer::StaticCreate);
    srand((unsigned int)time(NULL));
    sInstance->CreateSimulationWorld();
    sInstance->InitializeVisionObservationStore();
    sInstance->StartSimulation();
}

void NetworkManagerServer::ProcessPacket(ClientProxy* _session,InputMemoryStream& _inStream) 
{
    if (!_session)
        return;

    RobotProtocol::PacketID packetID;
    const bool hasRobotProtocolID =
        RobotProtocol::PeekPacketID(_inStream, packetID);
    const ClientProtocolIdentity identity = _session->GetProtocolIdentity();

    if (identity == ClientProtocolIdentity::Rejected)
        return;

    if (identity == ClientProtocolIdentity::Robot)
    {
        if (!hasRobotProtocolID ||
            !RobotProtocol::IsKnownRobotPacketID(
                static_cast<uint16_t>(packetID)))
        {
            std::cout << "[Protocol] Robot client attempted protocol switch\n";
            return;
        }
        TryProcessRobotProtocolPacket(_session, _inStream);
        return;
    }

    if (identity == ClientProtocolIdentity::Vision)
    {
        if (!hasRobotProtocolID ||
            !RobotProtocol::IsKnownVisionPacketID(
                static_cast<uint16_t>(packetID)))
        {
            std::cout << "[Protocol] Vision client attempted protocol switch\n";
            return;
        }
        TryProcessVisionProtocolPacket(_session, _inStream);
        return;
    }

    if (identity == ClientProtocolIdentity::Unity)
    {
        if (hasRobotProtocolID)
        {
            std::cout << "[Protocol] Unity client attempted protocol switch\n";
            return;
        }
        ProcessUnityPacket(_session, _inStream);
        return;
    }

    // An unclassified socket may select a protocol only with that protocol's
    // first HELLO. This prevents malformed Vision frames from falling through
    // to the legacy one-byte Unity parser.
    if (hasRobotProtocolID)
    {
        if (RobotProtocol::IsKnownVisionPacketID(
                static_cast<uint16_t>(packetID)))
        {
            if (packetID != RobotProtocol::PacketID::VISION_HELLO)
            {
                std::cout << "[Vision] Packet before VISION_HELLO rejected\n";
                _session->RejectProtocol();
                return;
            }
            TryProcessVisionProtocolPacket(_session, _inStream);
            return;
        }

        if (packetID != RobotProtocol::PacketID::HELLO)
        {
            std::cout << "[RobotProtocol] Packet before HELLO rejected\n";
            _session->RejectProtocol();
            return;
        }
        _session->TryBindProtocolIdentity(ClientProtocolIdentity::Robot);
        TryProcessRobotProtocolPacket(_session, _inStream);
        return;
    }

    if (_inStream.GetRemainDataSize() == 0 ||
        static_cast<uint8_t>(_inStream.GetBuffer()[0]) !=
            static_cast<uint8_t>(UPT_HELLO))
    {
        std::cout << "[Protocol] First packet was not a recognized HELLO\n";
        _session->RejectProtocol();
        return;
    }

    _session->TryBindProtocolIdentity(ClientProtocolIdentity::Unity);
    ProcessUnityPacket(_session, _inStream);
}

void NetworkManagerServer::ProcessUnityPacket(
    ClientProxy* _session,
    InputMemoryStream& _inStream)
{
    if (_inStream.GetRemainDataSize() == 0)
        return;

    uint8_t packet_type;
    _inStream.Read(packet_type);
    switch (packet_type)
    {
    case UPT_HELLO:
        HandleHello_Packet(_session,_inStream);
        break;
    case UPT_REPLICATION:
        break;
    case UPT_MAZE_DATA:
        /* code */
        break;
    case UPT_DISCONNECTED:
        /* code */
        break;        
    case UPT_READY_MAP:
        HandleReadyMap_Packet(_session,_inStream);
        break;
    case UPT_READY_OBJECT:
        HandleReadyObject_Packet(_session,_inStream);
        break;
    default:  
        printf("Inavalid PacketData\n\a");
        break;
    }
}

bool NetworkManagerServer::TryProcessRobotProtocolPacket(ClientProxy* _proxy, InputMemoryStream& _stream)
{
    RobotProtocol::PacketID packetID;
    if (!RobotProtocol::PeekPacketID(_stream, packetID) ||
        !RobotProtocol::IsKnownRobotPacketID(
            static_cast<uint16_t>(packetID)))
        return false;

    RobotProtocol::PacketBodyHeader header;
    if (!RobotProtocol::ReadPacketBodyHeader(_stream, header))
    {
        std::cout << "[RobotProtocol] Invalid packet header\n";
        return true;
    }

    if (packetID == RobotProtocol::PacketID::HELLO)
    {
        HandleRobotHelloPacket(_proxy, header, _stream);
        return true;
    }

    RobotSessionPtr robotSession = FindRobotSession(_proxy, header.agvID);
    if (!robotSession)
    {
        std::cout << "[RobotProtocol] Packet from unregistered robot. packetID="
                  << static_cast<uint16_t>(packetID) << " agvID=" << header.agvID << "\n";
        return true;
    }

    robotSession->ProcessPacket(header, _stream);
    return true;
}

bool NetworkManagerServer::TryProcessVisionProtocolPacket(
    ClientProxy* _proxy,
    InputMemoryStream& _stream)
{
    RobotProtocol::PacketID packetID;
    if (!RobotProtocol::PeekPacketID(_stream, packetID) ||
        !RobotProtocol::IsKnownVisionPacketID(
            static_cast<uint16_t>(packetID)))
    {
        return false;
    }

    RobotProtocol::PacketBodyHeader header;
    if (!RobotProtocol::ReadPacketBodyHeader(_stream, header))
    {
        std::cout << "[Vision] Invalid packet header\n";
        return true;
    }

    switch (packetID)
    {
    case RobotProtocol::PacketID::VISION_HELLO:
        HandleVisionHelloPacket(_proxy, header, _stream);
        break;
    case RobotProtocol::PacketID::VISION_OBSERVATION:
        HandleVisionObservationPacket(_proxy, header, _stream);
        break;
    case RobotProtocol::PacketID::VISION_HELLO_ACK:
        std::cout << "[Vision] Unexpected client VISION_HELLO_ACK rejected\n";
        break;
    default:
        break;
    }
    return true;
}

void NetworkManagerServer::HandleRobotHelloPacket(ClientProxy* _proxy, const RobotProtocol::PacketBodyHeader& _header, InputMemoryStream& _stream)
{
    RobotProtocol::HelloPayload hello;
    if (!RobotProtocol::ReadHelloPayload(_stream, hello))
    {
        std::cout << "[RobotProtocol] Invalid HELLO payload\n";
        return;
    }

    const bool isRobotClient =
        hello.clientType == RobotProtocol::ClientType::ESP32 ||
        hello.clientType == RobotProtocol::ClientType::FAKE_ROBOT;

    if (!isRobotClient)
    {
        std::cout << "[RobotProtocol] Unsupported robot-protocol client type: "
                  << static_cast<int>(hello.clientType) << "\n";
        return;
    }

    uint32_t assignedAgvID = hello.requestedAgvID != 0 ? hello.requestedAgvID : _header.agvID;
    if (assignedAgvID == 0 && !AGVManager::GetInstance().m_AGVs.empty())
    {
        assignedAgvID = AGVManager::GetInstance().m_AGVs.front()->GetNetworkID();
    }

    RobotProtocol::HelloAckPayload ack;
    ack.protocolVersion = RobotProtocol::kProtocolVersion;
    ack.assignedAgvID = assignedAgvID;

    const bool versionOK = hello.protocolVersion == RobotProtocol::kProtocolVersion;
    ObjectPtr agvObject = assignedAgvID != 0 ? m_LinkingContext->GetObject(assignedAgvID) : nullptr;
    const bool agvOK = agvObject && agvObject->GetClassID() == ClassID::OBJ_AGV;

    RobotSessionPtr robotSession = std::make_shared<RobotSession>(
        _proxy->GetSession(), assignedAgvID, hello.capabilities);
    if (Robo* agv = dynamic_cast<Robo*>(agvObject.get()))
    {
        robotSession->PrimeStatus(agv->GetPosX(), agv->GetPosZ(), agv->GetHeadingAngle());
    }

    if (!versionOK)
    {
        ack.accepted = 0;
        ack.errorCode = RobotProtocol::ErrorCode::PROTOCOL_MISMATCH;
        robotSession->SendHelloAck(ack);
        std::cout << "[RobotProtocol] Rejected robot HELLO: protocol version "
                  << hello.protocolVersion << " != " << RobotProtocol::kProtocolVersion << "\n";
        return;
    }

    if (!agvOK)
    {
        ack.accepted = 0;
        ack.errorCode = RobotProtocol::ErrorCode::UNKNOWN_AGV;
        robotSession->SendHelloAck(ack);
        std::cout << "[RobotProtocol] Rejected robot HELLO: unknown AGV " << assignedAgvID << "\n";
        return;
    }

    ack.accepted = 1;
    ack.errorCode = RobotProtocol::ErrorCode::NONE;

    m_AgvIdToRobotSessionMap[assignedAgvID] = robotSession;
    m_ProxyToRobotSessionMap[_proxy] = robotSession;

    ESP32TrajectoryExecutionConfig trajectoryConfig;
    if (m_RunMode == ServerRunMode::PhysicalFleet)
    {
        trajectoryConfig.enabled = true;
        trajectoryConfig.millimetersPerMapUnit = kPhysicalFleetScaleMmPerMapUnit;
        trajectoryConfig.cruiseSpeedMmPerSecond = kPhysicalFleetCruiseSpeedMmPerSecond;
    }
    RobotManager::GetInstance().RegisterRobot(
        assignedAgvID,
        std::make_unique<ESP32RobotController>(robotSession, trajectoryConfig)
    );

    robotSession->SendHelloAck(ack);

    std::cout << "[RobotProtocol] Robot client connected. agvID=" << assignedAgvID
              << " clientType=" << static_cast<int>(hello.clientType)
              << " capabilities=0x" << std::hex << hello.capabilities << std::dec
              << " sequence=" << _header.sequence << "\n";

    if (m_RunMode == ServerRunMode::PhysicalFleet)
    {
        TryActivatePhysicalFleet();
    }
    else if (m_RunMode == ServerRunMode::PhysicalDemo)
    {
        SendPhysicalDemoRoute(assignedAgvID);
    }
    else if (m_RunMode == ServerRunMode::TrajectoryPreview)
    {
        SendTrajectoryPreview(assignedAgvID, robotSession);
    }
    else if (m_RunMode == ServerRunMode::TrajectoryRaisedWheel)
    {
        SendTrajectoryRaisedWheel(assignedAgvID, robotSession);
    }
    else if (!RoutePlanner::GetInstance().ResendCurrentRouteToController(assignedAgvID))
    {
        std::cout << "[RoutePlanner] No active route to resend for AGV " << assignedAgvID << "\n";
    }
}

void NetworkManagerServer::HandleVisionHelloPacket(
    ClientProxy* _proxy,
    const RobotProtocol::PacketBodyHeader& _header,
    InputMemoryStream& _stream)
{
    RobotProtocol::VisionHelloPayload hello;
    RobotProtocol::VisionHelloAckPayload ack;
    ack.protocolVersion = RobotProtocol::kProtocolVersion;
    const bool alreadyAccepted = m_ProxyToVisionSessionMap.contains(_proxy);

    if (!RobotProtocol::ReadVisionHelloPayload(_stream, hello))
    {
        ack.accepted = 0;
        ack.rejectionReason =
            RobotProtocol::VisionHelloRejectionReason::MALFORMED_HANDSHAKE;
        SendVisionHelloAck(_proxy, _header.sequence, ack);
        if (!alreadyAccepted)
            _proxy->RejectProtocol();
        std::cout << "[Vision] Malformed VISION_HELLO rejected\n";
        return;
    }

    ack.sourceID = hello.sourceID;
    ack.sessionID = hello.sessionID;

    if (!m_VisionConfig.enabled)
    {
        ack.accepted = 0;
        ack.rejectionReason =
            RobotProtocol::VisionHelloRejectionReason::FEATURE_DISABLED;
    }
    else if (_header.agvID != 0 ||
             hello.protocolVersion != RobotProtocol::kProtocolVersion)
    {
        ack.accepted = 0;
        ack.rejectionReason =
            RobotProtocol::VisionHelloRejectionReason::PROTOCOL_MISMATCH;
    }
    else if (hello.sourceID == 0 || hello.sessionID == 0 ||
             hello.sourceID != m_VisionConfig.expectedSourceID)
    {
        ack.accepted = 0;
        ack.rejectionReason =
            RobotProtocol::VisionHelloRejectionReason::INVALID_SOURCE;
    }
    else if (hello.mapContractID != m_VisionConfig.expectedMapContractID)
    {
        ack.accepted = 0;
        ack.rejectionReason =
            RobotProtocol::VisionHelloRejectionReason::MAP_CONTRACT_MISMATCH;
    }
    else if (hello.poseContractID != m_VisionConfig.expectedPoseContractID)
    {
        ack.accepted = 0;
        ack.rejectionReason =
            RobotProtocol::VisionHelloRejectionReason::POSE_CONTRACT_MISMATCH;
    }
    else if (alreadyAccepted ||
             m_VisionSourceToProxyMap.contains(hello.sourceID))
    {
        ack.accepted = 0;
        ack.rejectionReason =
            RobotProtocol::VisionHelloRejectionReason::DUPLICATE_SESSION;
    }
    else
    {
        ack.accepted = 1;
        ack.rejectionReason = RobotProtocol::VisionHelloRejectionReason::NONE;
    }

    SendVisionHelloAck(_proxy, _header.sequence, ack);
    if (ack.accepted == 0)
    {
        // Re-HELLO on an already authenticated socket is rejected without
        // poisoning or orphaning the original accepted session.
        if (!alreadyAccepted)
            _proxy->RejectProtocol();
        std::cout << "[Vision] VISION_HELLO rejected reason="
                  << static_cast<uint16_t>(ack.rejectionReason) << "\n";
        return;
    }

    if (!_proxy->TryBindProtocolIdentity(ClientProtocolIdentity::Vision))
    {
        _proxy->RejectProtocol();
        std::cout << "[Vision] Protocol identity binding failed\n";
        return;
    }

    VisionClientSession session;
    session.sourceID = hello.sourceID;
    session.sessionID = hello.sessionID;
    session.mapContractID = std::move(hello.mapContractID);
    session.poseContractID = std::move(hello.poseContractID);
    m_ProxyToVisionSessionMap[_proxy] = session;
    m_VisionSourceToProxyMap[session.sourceID] = _proxy;

    std::cout << "[Vision] Observation-only source connected. sourceID="
              << session.sourceID << " sessionID=" << session.sessionID << "\n";
}

void NetworkManagerServer::HandleVisionObservationPacket(
    ClientProxy* _proxy,
    const RobotProtocol::PacketBodyHeader& _header,
    InputMemoryStream& _stream)
{
    auto sessionIt = m_ProxyToVisionSessionMap.find(_proxy);
    if (sessionIt == m_ProxyToVisionSessionMap.end() ||
        _proxy->GetProtocolIdentity() != ClientProtocolIdentity::Vision ||
        !m_VisionObservationStore)
    {
        std::cout << "[Vision] Observation before accepted VISION_HELLO rejected\n";
        return;
    }

    if (_header.sequence == 0)
    {
        std::cout << "[Vision] Observation sequence 0 rejected\n";
        return;
    }

    VisionClientSession& session = sessionIt->second;
    if (session.hasReceivedObservation &&
        _header.sequence <= session.lastObservationSequence)
    {
        std::cout << "[Vision] Duplicate/out-of-order transport sequence rejected\n";
        return;
    }
    // Consume a well-formed body header sequence even when the payload or its
    // semantic values are later rejected. A sender must never reuse it.
    session.hasReceivedObservation = true;
    session.lastObservationSequence = _header.sequence;

    RobotProtocol::VisionObservationPayload payload;
    if (!RobotProtocol::ReadVisionObservationPayload(_stream, payload))
    {
        std::cout << "[Vision] Malformed VISION_OBSERVATION rejected\n";
        return;
    }

    VisionObservationInput input;
    input.agvID = _header.agvID;
    input.sourceID = session.sourceID;
    input.sessionID = session.sessionID;
    input.sequence = _header.sequence;
    input.sourceTimestampMicroseconds = payload.sourceTimestampUs;
    input.reportedAgeMilliseconds = payload.reportedAgeMs;
    input.state = payload.state;
    if (payload.pose.has_value())
    {
        input.pose = VisionMetricPose{
            payload.pose->xMm,
            payload.pose->zMm,
            payload.pose->headingDeg
        };
    }
    input.calibrationID = std::move(payload.calibrationID);
    input.mapContractID = session.mapContractID;
    input.poseContractID = session.poseContractID;
    input.verificationState = payload.verificationState;
    input.quality = payload.quality;

    const uint64_t receivedAtMs = MonotonicMilliseconds();
    const VisionObservationStoreResult result =
        m_VisionObservationStore->TryStore(
            input,
            receivedAtMs,
            receivedAtMs,
            [this](uint32_t agvID)
            {
                const ObjectPtr object = m_LinkingContext->GetObject(agvID);
                return object && object->GetClassID() == ClassID::OBJ_AGV;
            });

    if (result != VisionObservationStoreResult::ACCEPTED)
    {
        std::cout << "[Vision] Observation rejected. agvID=" << _header.agvID
                  << " sequence=" << _header.sequence
                  << " reason=" << VisionStoreResultName(result) << "\n";
        return;
    }

    if (_header.sequence % kVisionAcceptedLogInterval == 0)
    {
        std::cout << "[Vision] Observation accepted. agvID=" << _header.agvID
                  << " sequence=" << _header.sequence
                  << " state=" << static_cast<uint16_t>(payload.state)
                  << "\n";
    }

    if (m_RunMode == ServerRunMode::PhysicalFleet &&
        !m_IsPhysicalFleetActivated)
    {
        TryActivatePhysicalFleet();
    }
}

void NetworkManagerServer::SendVisionHelloAck(
    ClientProxy* _proxy,
    uint32_t _sequence,
    const RobotProtocol::VisionHelloAckPayload& _payload)
{
    OutputMemoryStream outStream;
    RobotProtocol::WritePacketBodyHeader(
        outStream,
        RobotProtocol::PacketID::VISION_HELLO_ACK,
        0,
        _sequence);
    if (!RobotProtocol::WriteVisionHelloAckPayload(outStream, _payload))
    {
        std::cout << "[Vision] Failed to serialize VISION_HELLO_ACK\n";
        return;
    }
    _proxy->SendPacket(outStream);
}

void NetworkManagerServer::InitializeVisionObservationStore()
{
    if (!m_VisionConfig.enabled)
        return;

    const auto nodes = MapManager::GetInstance().GetNodes();
    if (!MatchesExpectedVisionMapContract(nodes))
    {
        throw std::runtime_error(
            "Vision observation active map does not match contract 67254eca75c55e5c");
    }

    VisionObservationStoreConfig storeConfig;
    storeConfig.map = VisionMapCoordinateContract::FromCanonicalMap(
        kVisionScaleMmPerMapUnit,
        kVisionLocalOriginServerX,
        kVisionLocalOriginServerZ,
        kVisionAllowedMapMarginMm);
    storeConfig.expectedCalibrationID = m_VisionConfig.expectedCalibrationID;
    storeConfig.expectedMapContractID = m_VisionConfig.expectedMapContractID;
    storeConfig.expectedPoseContractID = m_VisionConfig.expectedPoseContractID;
    storeConfig.expectedSourceID = m_VisionConfig.expectedSourceID;
    m_VisionObservationStore =
        std::make_unique<VisionObservationStore>(std::move(storeConfig));

    if (m_RunMode == ServerRunMode::PhysicalFleet)
        std::cout << "[VisionCorrection] Direct node correction enabled\n";
    else
        std::cout << "[Vision] Observation-only receiver enabled\n";
}

void NetworkManagerServer::TryActivatePhysicalFleet()
{
    if (m_RunMode != ServerRunMode::PhysicalFleet)
        return;

    const auto sessionIt = m_AgvIdToRobotSessionMap.find(kPhysicalFleetAgvID);
    if (sessionIt == m_AgvIdToRobotSessionMap.end() || !sessionIt->second)
        return;

    const RobotSessionPtr& session = sessionIt->second;
    if (!session->SupportsTrajectoryCommand())
    {
        std::cout << "[PhysicalFleet] AGV 1 lacks trajectory capability; dispatch stopped\n";
        return;
    }
    if (m_VisionConfig.enabled && !session->SupportsNodeCorrection())
    {
        std::cout << "[VisionCorrection] AGV 1 lacks node-correction capability; dispatch stopped\n";
        return;
    }

    if (m_VisionConfig.enabled)
    {
        const auto latest = m_VisionObservationStore ?
            m_VisionObservationStore->GetLatest(kPhysicalFleetAgvID) :
            std::nullopt;
        const uint64_t nowMs = MonotonicMilliseconds();
        if (!latest.has_value() || !latest->observation.pose.has_value() ||
            latest->observation.state !=
                RobotProtocol::VisionTrackingState::MEASURED ||
            latest->observation.verificationState !=
                RobotProtocol::VisionVerificationState::VERIFIED ||
            latest->receivedAtServerMilliseconds > nowMs ||
            nowMs - latest->receivedAtServerMilliseconds > 200)
        {
            std::cout << "[VisionCorrection] Waiting for fresh MEASURED pose before dispatch\n";
            return;
        }

        const MapNode startNode = MapManager::GetInstance().GetMapNode(
            kPhysicalFleetStartNodeID);
        const VisionMetricPose& actual = *latest->observation.pose;
        PhysicalFleetCorrectionInput startPose;
        startPose.actualXMm = actual.xMillimeters;
        startPose.actualZMm = actual.zMillimeters;
        startPose.actualHeadingRad =
            actual.headingDegrees * kDegreesToRadians;
        startPose.targetXMm =
            (startNode.m_PosX - kVisionLocalOriginServerX) *
            kVisionScaleMmPerMapUnit;
        startPose.targetZMm =
            (startNode.m_PosZ - kVisionLocalOriginServerZ) *
            kVisionScaleMmPerMapUnit;
        startPose.expectedArrivalHeadingRad = 0.0f;

        const PhysicalFleetCorrectionDecision startDecision =
            DecidePhysicalFleetCorrection(startPose);
        if (startDecision.action != PhysicalFleetCorrectionAction::ACCEPT)
        {
            std::cout << "[VisionCorrection] Waiting for AGV at node 1, east. "
                      << "positionErrorMm=" << startDecision.positionErrorMm
                      << " headingErrorDeg="
                      << startDecision.headingErrorRad / kDegreesToRadians
                      << "\n";
            return;
        }
    }

    if (m_IsPhysicalFleetActivated)
    {
        if (!m_PhysicalFleetCorrection.active() &&
            RoutePlanner::GetInstance().ResendCurrentRouteToController(
                kPhysicalFleetAgvID))
        {
            std::cout << "[PhysicalFleet] Active automatic route restored\n";
        }
        return;
    }

    m_IsPhysicalFleetActivated = true;
    EventManager::GetInstance().Publish(
        { RobotEventType::IDLE_READY,
          kPhysicalFleetAgvID,
          m_TotalElapsedServerTime });
    if (m_VisionConfig.enabled)
    {
        std::cout << "[PhysicalFleet] Vision and correction-capable AGV ready; "
                     "automatic dispatch enabled\n";
    }
    else
    {
        std::cout << "[PhysicalFleet] Command-capable AGV ready; "
                     "automatic dispatch enabled\n";
    }
}

bool NetworkManagerServer::HandlePhysicalFleetControllerEvent(
    uint32_t _agvID,
    const ControllerEvent& _event)
{
    if (m_RunMode != ServerRunMode::PhysicalFleet)
        return false;

    auto* controller = dynamic_cast<ESP32RobotController*>(
        RobotManager::GetInstance().GetRobotController(_agvID));
    if (!controller)
        return false;

    if (_event.type == ControllerEventType::NODE_CORRECTION_REPORT)
    {
        HandlePhysicalFleetCorrectionReport(_agvID, _event);
        return true;
    }

    if (_event.type != ControllerEventType::ARRIVED)
        return false;

    if (!controller->IsExpectedPhysicalArrival(_event.nodeID))
        return false;

    if (!m_VisionConfig.enabled)
    {
        if (!controller->ConfirmCorrectedPhysicalArrival(_event.nodeID))
        {
            RoutePlanner::GetInstance().StopActiveRouteForSafety(
                _agvID,
                m_TotalElapsedServerTime,
                "physical edge confirmation failed");
            return true;
        }
        return false;
    }

    BeginPhysicalFleetCorrection(_agvID, _event.nodeID);
    return true;
}

void NetworkManagerServer::BeginPhysicalFleetCorrection(
    uint32_t _agvID,
    uint32_t _nodeID)
{
    if (m_PhysicalFleetCorrection.active())
    {
        if (m_PhysicalFleetCorrection.agvID == _agvID &&
            m_PhysicalFleetCorrection.nodeID == _nodeID)
        {
            std::cout << "[VisionCorrection] Duplicate coarse ARRIVED ignored\n";
            return;
        }
        FailPhysicalFleetCorrection("overlapping coarse ARRIVED");
        return;
    }

    auto* controller = dynamic_cast<ESP32RobotController*>(
        RobotManager::GetInstance().GetRobotController(_agvID));
    float expectedHeadingRad = 0.0f;
    if (!controller || !controller->SupportsNodeCorrection() ||
        !controller->TryGetExpectedArrivalHeading(
            _nodeID, expectedHeadingRad))
    {
        RoutePlanner::GetInstance().StopActiveRouteForSafety(
            _agvID,
            m_TotalElapsedServerTime,
            "Vision correction unavailable");
        return;
    }

    const uint32_t routeID = controller->GetActivePhysicalRouteID();
    if (routeID == 0)
    {
        RoutePlanner::GetInstance().StopActiveRouteForSafety(
            _agvID,
            m_TotalElapsedServerTime,
            "missing physical edge route ID");
        return;
    }

    uint32_t baselineSequence = 0;
    if (m_VisionObservationStore)
    {
        const auto latest = m_VisionObservationStore->GetLatest(_agvID);
        if (latest.has_value())
            baselineSequence = latest->observation.sequence;
    }

    m_PhysicalFleetCorrection = {};
    m_PhysicalFleetCorrection.phase =
        PhysicalFleetCorrectionState::Phase::WAITING_FOR_MEASUREMENT;
    m_PhysicalFleetCorrection.agvID = _agvID;
    m_PhysicalFleetCorrection.nodeID = _nodeID;
    m_PhysicalFleetCorrection.routeID = routeID;
    m_PhysicalFleetCorrection.baselineVisionSequence = baselineSequence;
    m_PhysicalFleetCorrection.expectedHeadingRad = expectedHeadingRad;
    m_PhysicalFleetCorrection.deadlineMilliseconds =
        MonotonicMilliseconds() + kCorrectionMeasurementWaitMs;

    std::cout << "[VisionCorrection] Coarse node " << _nodeID
              << " reached; waiting for post-stop MEASURED pose\n";
}

void NetworkManagerServer::HandlePhysicalFleetCorrectionReport(
    uint32_t _agvID,
    const ControllerEvent& _event)
{
    if (!m_PhysicalFleetCorrection.active())
    {
        std::cout << "[VisionCorrection] Late correction report ignored\n";
        return;
    }

    if (m_PhysicalFleetCorrection.phase ==
            PhysicalFleetCorrectionState::Phase::WAITING_FOR_MEASUREMENT &&
        _event.commandID == m_PhysicalFleetCorrection.commandID &&
        _event.routeID == m_PhysicalFleetCorrection.routeID &&
        _event.nodeID == m_PhysicalFleetCorrection.nodeID)
    {
        std::cout << "[VisionCorrection] Duplicate completion report ignored\n";
        return;
    }

    if (m_PhysicalFleetCorrection.phase !=
            PhysicalFleetCorrectionState::Phase::WAITING_FOR_REPORT ||
        _agvID != m_PhysicalFleetCorrection.agvID ||
        _event.nodeID != m_PhysicalFleetCorrection.nodeID ||
        _event.routeID != m_PhysicalFleetCorrection.routeID ||
        _event.commandID != m_PhysicalFleetCorrection.commandID)
    {
        FailPhysicalFleetCorrection("correction report identity mismatch");
        return;
    }

    if (_event.correctionResult !=
        RobotProtocol::NodeCorrectionResult::COMPLETED)
    {
        std::cout << "[VisionCorrection] Primitive failed. result="
                  << static_cast<unsigned>(_event.correctionResult)
                  << " detail=" << _event.detail << "\n";
        FailPhysicalFleetCorrection("ESP32 correction primitive failed");
        return;
    }

    if (m_VisionObservationStore)
    {
        const auto latest = m_VisionObservationStore->GetLatest(_agvID);
        if (latest.has_value())
        {
            m_PhysicalFleetCorrection.baselineVisionSequence = std::max(
                m_PhysicalFleetCorrection.baselineVisionSequence,
                latest->observation.sequence);
        }
    }
    m_PhysicalFleetCorrection.phase =
        PhysicalFleetCorrectionState::Phase::WAITING_FOR_MEASUREMENT;
    m_PhysicalFleetCorrection.deadlineMilliseconds =
        MonotonicMilliseconds() + kCorrectionMeasurementWaitMs;
    std::cout << "[VisionCorrection] Primitive completed; remeasuring\n";
}

void NetworkManagerServer::UpdatePhysicalFleetCorrection()
{
    if (!m_PhysicalFleetCorrection.active())
        return;

    const uint64_t nowMs = MonotonicMilliseconds();
    if (nowMs > m_PhysicalFleetCorrection.deadlineMilliseconds)
    {
        FailPhysicalFleetCorrection(
            m_PhysicalFleetCorrection.phase ==
                    PhysicalFleetCorrectionState::Phase::WAITING_FOR_REPORT
                ? "correction report timeout"
                : "fresh Vision measurement timeout");
        return;
    }
    if (m_PhysicalFleetCorrection.phase !=
        PhysicalFleetCorrectionState::Phase::WAITING_FOR_MEASUREMENT)
    {
        return;
    }

    if (!m_VisionObservationStore)
    {
        FailPhysicalFleetCorrection("Vision store unavailable");
        return;
    }
    const auto latest = m_VisionObservationStore->GetLatest(
        m_PhysicalFleetCorrection.agvID);
    if (!latest.has_value() || !latest->observation.pose.has_value() ||
        latest->observation.sequence <=
            m_PhysicalFleetCorrection.baselineVisionSequence ||
        latest->observation.state !=
            RobotProtocol::VisionTrackingState::MEASURED ||
        latest->observation.verificationState !=
            RobotProtocol::VisionVerificationState::VERIFIED ||
        latest->receivedAtServerMilliseconds > nowMs ||
        nowMs - latest->receivedAtServerMilliseconds >
            kCorrectionMaximumVisionAgeMs)
    {
        return;
    }

    const MapNode target = MapManager::GetInstance().GetMapNode(
        m_PhysicalFleetCorrection.nodeID);
    const VisionMetricPose& actual = *latest->observation.pose;
    PhysicalFleetCorrectionInput input;
    input.actualXMm = actual.xMillimeters;
    input.actualZMm = actual.zMillimeters;
    input.actualHeadingRad = actual.headingDegrees * kDegreesToRadians;
    input.targetXMm =
        (target.m_PosX - kVisionLocalOriginServerX) * kVisionScaleMmPerMapUnit;
    input.targetZMm =
        (target.m_PosZ - kVisionLocalOriginServerZ) * kVisionScaleMmPerMapUnit;
    input.expectedArrivalHeadingRad =
        m_PhysicalFleetCorrection.expectedHeadingRad;

    const PhysicalFleetCorrectionDecision decision =
        DecidePhysicalFleetCorrection(input);
    std::cout << "[VisionCorrection] node="
              << m_PhysicalFleetCorrection.nodeID
              << " positionErrorMm=" << decision.positionErrorMm
              << " headingErrorDeg="
              << decision.headingErrorRad / kDegreesToRadians << "\n";

    if (decision.action == PhysicalFleetCorrectionAction::ACCEPT)
    {
        CompletePhysicalFleetCorrection();
        return;
    }
    if (decision.action == PhysicalFleetCorrectionAction::REJECT)
    {
        FailPhysicalFleetCorrection("correction pose outside safety bounds");
        return;
    }
    if (m_PhysicalFleetCorrection.primitiveCount >=
        PhysicalFleetCorrectionPolicy::kMaximumPrimitivesPerNode)
    {
        std::cout << "[VisionCorrection] Primitive limit exhausted. completed="
                  << static_cast<unsigned>(
                         m_PhysicalFleetCorrection.primitiveCount)
                  << " limit=" << static_cast<unsigned>(
                         PhysicalFleetCorrectionPolicy::
                             kMaximumPrimitivesPerNode)
                  << " remainingPositionMm=" << decision.positionErrorMm
                  << " remainingHeadingDeg="
                  << decision.headingErrorRad / kDegreesToRadians << "\n";
        FailPhysicalFleetCorrection("correction primitive limit exhausted");
        return;
    }

    RobotProtocol::NodeCorrectionAction action;
    switch (decision.action)
    {
    case PhysicalFleetCorrectionAction::DRIVE_FORWARD:
        action = RobotProtocol::NodeCorrectionAction::DRIVE_FORWARD;
        break;
    case PhysicalFleetCorrectionAction::TURN_CW:
        action = RobotProtocol::NodeCorrectionAction::TURN_CW;
        break;
    case PhysicalFleetCorrectionAction::TURN_CCW:
        action = RobotProtocol::NodeCorrectionAction::TURN_CCW;
        break;
    default:
        FailPhysicalFleetCorrection("invalid correction decision");
        return;
    }

    auto sessionIt = m_AgvIdToRobotSessionMap.find(
        m_PhysicalFleetCorrection.agvID);
    if (sessionIt == m_AgvIdToRobotSessionMap.end() || !sessionIt->second)
    {
        FailPhysicalFleetCorrection("robot session unavailable");
        return;
    }

    if (m_NextCorrectionCommandID == 0)
        ++m_NextCorrectionCommandID;
    RobotProtocol::NodeCorrectionCommandPayload command;
    command.routeID = m_PhysicalFleetCorrection.routeID;
    command.nodeID = m_PhysicalFleetCorrection.nodeID;
    command.commandID = m_NextCorrectionCommandID++;
    command.action = action;
    command.magnitude = decision.magnitude;
    if (!sessionIt->second->SendNodeCorrection(command))
    {
        FailPhysicalFleetCorrection("correction command send failed");
        return;
    }

    m_PhysicalFleetCorrection.commandID = command.commandID;
    m_PhysicalFleetCorrection.baselineVisionSequence =
        latest->observation.sequence;
    ++m_PhysicalFleetCorrection.primitiveCount;
    m_PhysicalFleetCorrection.phase =
        PhysicalFleetCorrectionState::Phase::WAITING_FOR_REPORT;
    m_PhysicalFleetCorrection.deadlineMilliseconds =
        nowMs + kCorrectionReportWaitMs;
}

void NetworkManagerServer::CompletePhysicalFleetCorrection()
{
    const uint32_t agvID = m_PhysicalFleetCorrection.agvID;
    const uint32_t nodeID = m_PhysicalFleetCorrection.nodeID;
    auto* controller = dynamic_cast<ESP32RobotController*>(
        RobotManager::GetInstance().GetRobotController(agvID));
    if (!controller || !controller->ConfirmCorrectedPhysicalArrival(nodeID))
    {
        FailPhysicalFleetCorrection("corrected arrival confirmation failed");
        return;
    }

    const uint8_t primitiveCount =
        m_PhysicalFleetCorrection.primitiveCount;
    m_PhysicalFleetCorrection = {};
    EventManager::GetInstance().Publish(
        { RobotEventType::NODE_ARRIVED,
          agvID,
          m_TotalElapsedServerTime,
          nodeID });
    std::cout << "[VisionCorrection] Node " << nodeID
              << " accepted after "
              << static_cast<unsigned>(primitiveCount)
              << " correction primitive(s)\n";
}

void NetworkManagerServer::FailPhysicalFleetCorrection(const char* _reason)
{
    const uint32_t agvID = m_PhysicalFleetCorrection.agvID;
    std::cout << "[VisionCorrection] SAFE STOP: " << _reason << "\n";
    m_PhysicalFleetCorrection = {};
    if (agvID != 0)
    {
        RoutePlanner::GetInstance().StopActiveRouteForSafety(
            agvID,
            m_TotalElapsedServerTime,
            _reason);
    }
}

void NetworkManagerServer::SendTrajectoryPreview(
    uint32_t _agvID, const RobotSessionPtr& _robotSession)
{
    if (_agvID != kTrajectoryDemoAgvID)
    {
        std::cout << "[TrajectoryPreview] Not sent: only AGV 1 is allowed\n";
        return;
    }
    if (!_robotSession || !_robotSession->SupportsTrajectoryPreviewOnly())
    {
        std::cout << "[TrajectoryPreview] SAFE: client is not preview-only"
                     " (PREVIEW set, COMMAND clear); no trajectory sent\n";
        return;
    }

    const MapLink& previewLink = MapManager::GetInstance().FindLink(
        kTrajectoryDemoStartNodeID, kTrajectoryDemoTargetNodeID);
    if (previewLink.m_Type != 1)
    {
        std::cout << "[TrajectoryPreview] SAFE: directed [1 -> 4] is not a Bezier link;"
                     " no trajectory sent\n";
        return;
    }

    TrajectoryBuildOptions options;
    options.hasTrustedStartHeading = true;
    options.startHeadingRad = kTrajectoryDemoAssumedStartHeadingRad;
    options.millimetersPerMapUnit = kTrajectoryDemoScaleMmPerMapUnit;
    options.spacingMm = kTrajectoryDemoSpacingMm;
    // This mode validates transport and parsing only. A zero speed remains
    // non-executable even if a client violates the preview capability contract.
    options.cruiseSpeedMmPerSecond = 0.0f;

    RobotProtocol::TrajectoryCommandPayload trajectory;
    std::string error;
    if (!TrajectoryBuilder::Build(
            { kTrajectoryDemoStartNodeID, kTrajectoryDemoTargetNodeID },
            0, options, trajectory, error))
    {
        std::cout << "[TrajectoryPreview] SAFE: build failed; no trajectory sent: "
                  << error << "\n";
        return;
    }

    std::cout << "[TrajectoryPreview] Sending parse/store-only [1 -> 4]"
                 " with targetSpeed=0; no RoutePlanner execution\n";
    if (!_robotSession->SendTrajectoryPreview(std::move(trajectory)))
    {
        std::cout << "[TrajectoryPreview] SAFE: trajectory send rejected\n";
        return;
    }

    std::cout << "[TrajectoryPreview] Sent [1 -> 4]; waiting only for client preview log\n";
}

void NetworkManagerServer::SendTrajectoryRaisedWheel(
    uint32_t _agvID, const RobotSessionPtr& _robotSession)
{
    if (_agvID != kTrajectoryDemoAgvID)
    {
        std::cout << "[TrajectoryRaisedWheel] Not sent: only AGV 1 is allowed\n";
        return;
    }
    if (!_robotSession || !_robotSession->SupportsTrajectoryCommand())
    {
        std::cout << "[TrajectoryRaisedWheel] SAFE: client did not advertise COMMAND capability;"
                     " no trajectory sent\n";
        return;
    }

    const MapLink& link = MapManager::GetInstance().FindLink(
        kTrajectoryDemoStartNodeID, kTrajectoryDemoTargetNodeID);
    if (link.m_Type != 1)
    {
        std::cout << "[TrajectoryRaisedWheel] SAFE: directed [1 -> 4] is not a Bezier link;"
                     " no trajectory sent\n";
        return;
    }

    TrajectoryBuildOptions options;
    options.hasTrustedStartHeading = true;
    options.startHeadingRad = kTrajectoryDemoAssumedStartHeadingRad;
    options.millimetersPerMapUnit = kTrajectoryDemoScaleMmPerMapUnit;
    options.spacingMm = kTrajectoryDemoSpacingMm;
    options.cruiseSpeedMmPerSecond = kTrajectoryRaisedWheelSpeedMmPerSecond;

    RobotProtocol::TrajectoryCommandPayload trajectory;
    std::string error;
    if (!TrajectoryBuilder::Build(
            { kTrajectoryDemoStartNodeID, kTrajectoryDemoTargetNodeID },
            0, options, trajectory, error))
    {
        std::cout << "[TrajectoryRaisedWheel] SAFE: build failed; no trajectory sent: "
                  << error << "\n";
        return;
    }

    std::cout << "[TrajectoryRaisedWheel] Sending executable [1 -> 4] at "
              << kTrajectoryRaisedWheelSpeedMmPerSecond
              << " mm/s; local BOOT approval remains required\n";
    if (!_robotSession->SendTrajectory(std::move(trajectory)))
    {
        std::cout << "[TrajectoryRaisedWheel] SAFE: trajectory send rejected\n";
        return;
    }

    std::cout << "[TrajectoryRaisedWheel] Sent [1 -> 4]; wheels must remain raised\n";
}

void NetworkManagerServer::SendPhysicalDemoRoute(uint32_t _agvID)
{
    const std::vector<uint32_t> expectedNodeIDs = {
        kPhysicalDemoStartNodeID,
        kPhysicalDemoTargetNodeID
    };

    if (_agvID != kPhysicalDemoAgvID)
    {
        std::cout << "[PhysicalDemo] Route not sent: only AGV 1 is allowed\n";
        return;
    }

    if (RoutePlanner::GetInstance().ResendCurrentRouteToController(_agvID, expectedNodeIDs))
    {
        std::cout << "[PhysicalDemo] Resent active logical route [1 -> 2]\n";
        return;
    }

    std::cout << "[PhysicalDemo] Creating motor-disabled logical route [1 -> 2]\n";
    std::cout << "[PhysicalDemo] ESP32 temporarily maps this logical leg to 30 cm; physical scale is not calibrated\n";
    if (!RoutePlanner::GetInstance().CreateRouteMatchingNodes(
            _agvID, expectedNodeIDs, m_TotalElapsedServerTime, MissionPurpose::NONE))
    {
        std::cout << "[PhysicalDemo] SAFE STOP: exact route [1 -> 2] was not sent\n";
        return;
    }

    std::cout << "[PhysicalDemo] Exact route [1 -> 2] registered and sent\n";
}

RobotSessionPtr NetworkManagerServer::FindRobotSession(ClientProxy* _proxy, uint32_t _agvID)
{
    auto proxyIt = m_ProxyToRobotSessionMap.find(_proxy);
    if (proxyIt != m_ProxyToRobotSessionMap.end() &&
        proxyIt->second->GetAgvID() == _agvID)
    {
        return proxyIt->second;
    }

    return nullptr;
}

void NetworkManagerServer::HandleHello_Packet(ClientProxy* _proxy,InputMemoryStream& _instream)
{                
    uint32_t newClientSessionID=nextSessionID;
    
    m_SessionIdToProxyMap[newClientSessionID]=_proxy;    
    _proxy->SetSessionID(newClientSessionID);
    nextSessionID++;

    std::cout<<"[서버] 새 클라이언트 접속 승인 부여된 ID : "<<newClientSessionID<<std::endl;         


    for(auto& obj : m_LinkingContext->GetAllObjects())
    {
        uint32_t existingNetworkID=obj.first;
        _proxy->GetReplicationManagerServer().ReplicateCreate(existingNetworkID);
    }       

    SendHello_Packet(_proxy);

    SendMap_Packet(_proxy);    

    std::cout << "[서버] 맵 전송 완료. UPT_READY_MAP 대기 중 \n";
}

void NetworkManagerServer::SendHello_Packet(ClientProxy* _proxy)
{
    OutputMemoryStream outStream;
    uint8_t packetType = static_cast<uint8_t>(UPT_HELLO);

    outStream.Write(packetType);    
    
    outStream.Write(_proxy->GetSessionID());

    _proxy->SendPacket(outStream);
}

void NetworkManagerServer::SendMap_Packet(ClientProxy* _proxy)
{
    const std::unordered_map<uint32_t,MapNode> nodes = MapManager::GetInstance().GetNodes();
    std::vector<MapLink> links = MapManager::GetInstance().GetLinks();

    OutputMemoryStream outStream;
    uint8_t packetType = static_cast<uint8_t>(UPT_MAZE_DATA);
    
    outStream.Write(packetType);
    outStream.Write(nodes);
    outStream.Write(links);

    _proxy->SendPacket(outStream);
}

void NetworkManagerServer::OnClientAccepted(TCPSocketPtr _tcpSocket)
{
    TCPSessionPtr newClientSession = std::make_shared<TCPSession>();
    newClientSession->SetSocket(_tcpSocket);
    ClientProxyPtr newClientProxy=std::make_shared<ClientProxy>(newClientSession,0);

    m_PendingProxies.push_back(newClientProxy); 
}

void NetworkManagerServer::OnClientDisconnected(ClientProxy* _proxy)
{
    if (!_proxy)
        return;

    auto robotProxyIt = m_ProxyToRobotSessionMap.find(_proxy);
    if (robotProxyIt != m_ProxyToRobotSessionMap.end())
    {
        const RobotSessionPtr disconnectedSession = robotProxyIt->second;
        const uint32_t agvID = disconnectedSession->GetAgvID();
        const StatusPacket lastStatus = disconnectedSession->GetStatus();
        std::cout << "[RobotProtocol] AGV " << agvID
                  << " disconnected. lastLink=" << lastStatus.currentLinkID
                  << " progress=" << lastStatus.progress
                  << " pose=(" << lastStatus.x << "," << lastStatus.z << ")"
                  << " velocity=" << lastStatus.velocity << "\n";
        auto agvSessionIt = m_AgvIdToRobotSessionMap.find(agvID);
        if (agvSessionIt != m_AgvIdToRobotSessionMap.end() &&
            agvSessionIt->second == disconnectedSession)
        {
            m_AgvIdToRobotSessionMap.erase(agvSessionIt);
        }
        m_ProxyToRobotSessionMap.erase(robotProxyIt);
    }

    auto visionProxyIt = m_ProxyToVisionSessionMap.find(_proxy);
    if (visionProxyIt != m_ProxyToVisionSessionMap.end())
    {
        const uint32_t sourceID = visionProxyIt->second.sourceID;
        auto sourceIt = m_VisionSourceToProxyMap.find(sourceID);
        if (sourceIt != m_VisionSourceToProxyMap.end() &&
            sourceIt->second == _proxy)
        {
            m_VisionSourceToProxyMap.erase(sourceIt);
        }
        m_ProxyToVisionSessionMap.erase(visionProxyIt);
        std::cout << "[Vision] Source disconnected; latest observation retained as historical data\n";
    }

    for (auto it = m_SessionIdToProxyMap.begin();
         it != m_SessionIdToProxyMap.end();)
    {
        if (it->second == _proxy)
        {
            m_LastVisionDeliveryByUnitySession.erase(it->first);
            it = m_SessionIdToProxyMap.erase(it);
        }
        else
            ++it;
    }

    m_PendingProxies.erase(
        std::remove_if(
            m_PendingProxies.begin(), m_PendingProxies.end(),
            [_proxy](const ClientProxyPtr& proxy)
            {
                return proxy.get() == _proxy;
            }),
        m_PendingProxies.end());

    std::cout << "[Server] Disconnected client session cleaned up\n";
}

void NetworkManagerServer::RegisterObject(ObjectPtr _obj)
{
    uint32_t networkID = m_LinkingContext->GenerateNewNextNeworkID();
    _obj->SetNetworkID(networkID);
    m_LinkingContext->AddObject(_obj,networkID);    
    
    if(_obj->GetClassID()==ClassID::OBJ_AGV)
    {
        AGVManager::GetInstance().m_AGVs.push_back(_obj);
    }

    for(auto iter=m_SessionIdToProxyMap.begin();iter!=m_SessionIdToProxyMap.end();iter++)
    {    
        ClientProxy* session=iter->second;        
        session->GetReplicationManagerServer().ReplicateCreate(networkID);
    }
}

void NetworkManagerServer::SendOutgoingReplicationPackets()
{
    for(auto iter = m_SessionIdToProxyMap.begin();iter!=m_SessionIdToProxyMap.end();iter++)
    {
        ClientProxy* proxy = iter->second;
        OutputMemoryStream replicateStream;
        UnityPacketType packetType = UPT_REPLICATION;
        uint8_t packetTypeByte=static_cast<uint8_t>(packetType);

        replicateStream.Write(packetTypeByte);
              
        proxy->GetReplicationManagerServer().Write(replicateStream);

        //리플리케이션 장부가 비워져있지 않을때만 보내겠다.
        if(replicateStream.GetLength()>sizeof(uint8_t))
        {
            proxy->SendPacket(replicateStream);
        }
    }

    SendOutgoingVisionObservationPackets();
}

void NetworkManagerServer::SendOutgoingVisionObservationPackets()
{
    if (!m_VisionObservationStore || m_SessionIdToProxyMap.empty())
        return;

    const uint64_t nowMilliseconds = MonotonicMilliseconds();
    std::vector<UnityVisionObservationPayload> payloads;
    for (const auto& [networkID, object] : m_LinkingContext->GetAllObjects())
    {
        if (!object || object->GetClassID() != ClassID::OBJ_AGV)
            continue;

        const auto latest = m_VisionObservationStore->GetLatest(networkID);
        if (!latest.has_value())
            continue;

        payloads.push_back(BuildUnityVisionObservationPayload(
            *latest,
            m_VisionObservationStore->GetConfig(),
            nowMilliseconds));
    }

    std::sort(
        payloads.begin(), payloads.end(),
        [](const UnityVisionObservationPayload& lhs,
           const UnityVisionObservationPayload& rhs)
        {
            return lhs.agvID < rhs.agvID;
        });

    for (const auto& [sessionID, proxy] : m_SessionIdToProxyMap)
    {
        auto& deliveryByAgv =
            m_LastVisionDeliveryByUnitySession[sessionID];
        for (const UnityVisionObservationPayload& payload : payloads)
        {
            const VisionViewerDeliveryState nextDelivery{
                payload.transportSequence,
                payload.trackingState,
                payload.poseValid
            };
            const auto previous = deliveryByAgv.find(payload.agvID);
            if (previous != deliveryByAgv.end() &&
                previous->second == nextDelivery)
            {
                continue;
            }

            OutputMemoryStream stream;
            if (!WriteUnityVisionObservationPacket(stream, payload))
            {
                std::cout << "[Vision] Invalid Unity relay payload suppressed. agvID="
                          << payload.agvID << "\n";
                continue;
            }

            proxy->SendPacket(stream);
            deliveryByAgv[payload.agvID] = nextDelivery;
        }
    }
}

void NetworkManagerServer::HandleReadyObject_Packet(ClientProxy* _proxy,InputMemoryStream& _instream)
{ 
    (void)_proxy;
    (void)_instream;
    std::cout << "[서버] Unity viewer object ready\n";
}

#define _TESTCASE0
//#define _TESTCASE1
//#define _TESTCASE2
//#define _TESTCASE3
//#define _TESTCASE4

void NetworkManagerServer::CreateSimulationWorld()
{
    if (m_IsWorldCreated)
        return;

    const bool automaticTasks = m_RunMode == ServerRunMode::AutomaticFleet ||
                                m_RunMode == ServerRunMode::PhysicalFleet;
    std::vector<uint32_t> initNodes;
    if (m_RunMode == ServerRunMode::PhysicalFleet)
    {
        initNodes = { kPhysicalFleetStartNodeID };
    }
    else if (m_RunMode != ServerRunMode::AutomaticFleet)
    {
        initNodes = { kPhysicalDemoStartNodeID };
    }
    else
    {
        #ifdef _TESTCASE0
        initNodes = {1, 2, 3, 4};
        #elifdef _TESTCASE1
        initNodes = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22};
        #elifdef _TESTCASE2
        initNodes = {75,65,76,73,66,74,68,67,69,70,71,72,20,21,22,23,24,39,38,40,37,41};
        #elifdef _TESTCASE3
        initNodes = {1, 2, 3, 4, 5};
        #elifdef _TESTCASE4
        initNodes = {1, 2};
        #endif

    }

    if (automaticTasks)
    {
        TaskManager::GetInsance();
        WarehouseManager::GetInstance().Init();
    }

    RoutePlanner::GetInstance().Init();

    std::vector<Robo*> Robos;
    for(size_t i = 0; i < initNodes.size(); i++)
    {
        ObjectPtr newRobo = ObjectRegistry::sInstance->CreateObject(ClassID::OBJ_AGV);
        RegisterObject(newRobo); 
        Robo* agv = dynamic_cast<Robo*>(newRobo.get());          
        Robos.push_back(agv);

        agv->ChangeState(AGVState::IDLE);
        agv->SetHomeNode(initNodes[i]);

        int32_t startNodeID = initNodes[i];
        MapNode startNode = MapManager::GetInstance().GetNodes().find(startNodeID)->second;
        
        // 제자리 대기이므로 0.0f에 출발해서 1.0f에 도착하는 것으로 설정.
        agv->AssignNextStep(startNode, startNode, AGVState::IDLE, 0.0f, 1.0f); 
        
        agv->SetPos(startNode.m_PosX, startNode.m_PosZ);
        const bool trajectoryMode = m_RunMode == ServerRunMode::TrajectoryPreview ||
                                    m_RunMode == ServerRunMode::TrajectoryRaisedWheel;
        const float startHeading = trajectoryMode
            ? kTrajectoryDemoAssumedStartHeadingRad
            : 0.0f;
        agv->SetHeadingAngle(startHeading);
        agv->SetCurrentNodeID(startNodeID);
                
        RobotManager::GetInstance().RegisterRobot(
            agv->GetNetworkID(),
            std::make_unique<UnityRobotController>(
                agv->GetNetworkID(), startNode.m_PosX, startNode.m_PosZ, startHeading)
        );
        OccupancyProvider::GetInstance().OccupyNode(agv->GetNetworkID(), startNodeID);
        ReservationTable::GetInstance().ReserveNode(startNodeID, 0.0f, 100.0f, agv->GetNetworkID(), ReservationType::Goal);        
    }     

    if (m_RunMode == ServerRunMode::AutomaticFleet)
    {
        for (Robo* agv : Robos)
        {
            RobotEvent startEvent;
            startEvent.agvID = agv->GetNetworkID();
            startEvent.timestamp = m_TotalElapsedServerTime;
            startEvent.type = RobotEventType::IDLE_READY; // "저 백수입니다"

            EventManager::GetInstance().Publish(startEvent);
        }
    }

    m_IsWorldCreated = true;
    std::cout << "[서버] Server-authoritative world created. AGV count=" << initNodes.size() << "\n";
    if (m_RunMode == ServerRunMode::PhysicalDemo)
    {
        std::cout << "[PhysicalDemo] Single AGV at node 1; automatic task dispatch disabled\n";
        std::cout << "[PhysicalDemo] Waiting for AGV 1 HELLO before issuing [1 -> 2]\n";
    }
    else if (m_RunMode == ServerRunMode::PhysicalFleet)
    {
        std::cout << "[PhysicalFleet] TestCase0 LINE map; physical AGV 1 starts at node 1\n";
        std::cout << "[PhysicalFleet] Place AGV heading east (+X, 0 rad); waiting for armed COMMAND HELLO\n";
        std::cout << "[PhysicalFleet] Automatic task/route planning will start after that HELLO\n";
    }
    else if (m_RunMode == ServerRunMode::TrajectoryPreview)
    {
        std::cout << "[TrajectoryPreview] Single AGV at node 1; automatic task dispatch disabled\n";
        std::cout << "[TrajectoryPreview] Waiting for preview-capable AGV 1 HELLO\n";
        std::cout << "[TrajectoryPreview] Packet will contain [1 -> 4], targetSpeed=0,"
                     " and no motor execution route\n";
    }
    else if (m_RunMode == ServerRunMode::TrajectoryRaisedWheel)
    {
        std::cout << "[TrajectoryRaisedWheel] Single AGV at node 1; automatic task dispatch disabled\n";
        std::cout << "[TrajectoryRaisedWheel] Waiting for command-capable AGV 1 HELLO\n";
        std::cout << "[TrajectoryRaisedWheel] WARNING: [1 -> 4], 80 mm/s, raised wheels only\n";
    }
}

void NetworkManagerServer::HandleReadyMap_Packet(ClientProxy* _proxy, InputMemoryStream& _instream)
{
    (void)_proxy;
    (void)_instream;
    m_IsMapReady = true;
    std::cout << "[서버] Unity viewer map ready\n";
}

static bool a=true;
int i=0;


void NetworkManagerServer::UpdateWorld(float _deltaTime)
{   
    if(!m_IsSimulationActive) return;
    m_TotalElapsedServerTime += _deltaTime;    

    // 1. INPUT 레이어: 컨트롤러 패킷 수거
    for (auto it = RobotManager::GetInstance().GetRobotControllers().begin(); it != RobotManager::GetInstance().GetRobotControllers().end(); ++it) 
    {
        while (it->second->HasEvent()) 
        {            
            ControllerEvent ev = it->second->PopEvent();
            if (HandlePhysicalFleetControllerEvent(it->first, ev))
                continue;
            if (ev.type == ControllerEventType::ARRIVED) 
            {
                EventManager::GetInstance().Publish({ RobotEventType::NODE_ARRIVED, it->first, m_TotalElapsedServerTime, ev.nodeID });
            }
            else if (ev.type == ControllerEventType::EXECUTION_BLOCKED) 
            {
                RoutePlanner::GetInstance().OnExecutionBlocked(it->first, ev.nodeID, ev.relatedNodeID, m_TotalElapsedServerTime);
            }
            else if (ev.type == ControllerEventType::ERROR_SLIP)
            {
                std::cout << "[RobotProtocol] AGV " << it->first
                          << " reported ERROR code=" << ev.nodeID
                          << " detail=" << ev.detail << "\n";
                if (m_RunMode == ServerRunMode::PhysicalFleet)
                {
                    if (m_PhysicalFleetCorrection.active() &&
                        m_PhysicalFleetCorrection.agvID == it->first)
                    {
                        FailPhysicalFleetCorrection("ESP32 reported motion fault");
                    }
                    else
                    {
                        RoutePlanner::GetInstance().StopActiveRouteForSafety(
                            it->first,
                            m_TotalElapsedServerTime,
                            "ESP32 reported motion fault");
                    }
                }
            }
            else if (ev.type == ControllerEventType::EMERGENCY_STOP)
            {
                std::cout << "[RobotProtocol] AGV " << it->first
                          << " reported EMERGENCY_STOP\n";
                if (m_RunMode == ServerRunMode::PhysicalFleet)
                {
                    if (m_PhysicalFleetCorrection.active() &&
                        m_PhysicalFleetCorrection.agvID == it->first)
                    {
                        FailPhysicalFleetCorrection("ESP32 emergency stop");
                    }
                    else
                    {
                        RoutePlanner::GetInstance().StopActiveRouteForSafety(
                            it->first,
                            m_TotalElapsedServerTime,
                            "ESP32 emergency stop");
                    }
                }
            }
        }
    }

    UpdatePhysicalFleetCorrection();

    // 2. LOGIC 레이어: 장부 갱신 및 시공간 설계
    EventManager::GetInstance().SwapAndProcessEvents(); 
    RoutePlanner::GetInstance().Update(_deltaTime, m_TotalElapsedServerTime);
    for (auto& agvObj : AGVManager::GetInstance().m_AGVs) 
    {
        if (Robo* agv = dynamic_cast<Robo*>(agvObj.get())) 
        {
            agv->UpdateWorkTimer(_deltaTime, m_TotalElapsedServerTime);
        }
    }

    // 3. EXECUTION 레이어: 모듈들이 한 프레임 지연 주기에 맞춰 무결점 주행 결정
    RobotManager::GetInstance().Update(_deltaTime, m_TotalElapsedServerTime);

    // 4. VISUALIZATION 레이어: 좌표 렌더링 동기화
    for (auto it = RobotManager::GetInstance().GetRobotControllers().begin(); it != RobotManager::GetInstance().GetRobotControllers().end(); ++it) {
        StatusPacket status = it->second->GetStatus();
        ObjectPtr obj = m_LinkingContext->GetObject(it->first);
        if (obj && obj->GetClassID() == ClassID::OBJ_AGV) {
            if (Robo* agv = dynamic_cast<Robo*>(obj.get())) {
                agv->SetPos(status.x, status.z);
                agv->SetHeadingAngle(status.heading);
            }
        }
    }
}
