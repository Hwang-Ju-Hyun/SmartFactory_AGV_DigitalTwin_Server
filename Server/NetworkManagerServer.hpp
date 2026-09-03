#pragma once
#include "Object.hpp"
#include "TCPSocket.hpp"
#include "ClientProxy.hpp"
#include "RobotSession.hpp"
#include "VisionObservationStore.hpp"
#include "VisionObservationRelay.hpp"
#include "PhysicalFleetCorrection.hpp"
#include "MotorFaultDiagnostic.hpp"
#include <optional>
#include <string>
#include <unordered_map>

class LinkingContext;

enum class ServerRunMode
{
    AutomaticFleet,
    PhysicalFleet,
    PhysicalDemo,
    TrajectoryPreview,
    TrajectoryRaisedWheel
};

struct VisionObservationServerConfig
{
    bool enabled = false;
    uint32_t expectedSourceID = 1;
    std::string expectedCalibrationID;
    std::string expectedMapContractID = "67254eca75c55e5c";
    std::string expectedPoseContractID = "fb3cad48a32b9893";
};

struct VisionClientSession
{
    uint32_t sourceID = 0;
    uint64_t sessionID = 0;
    std::string mapContractID;
    std::string poseContractID;
    bool hasReceivedObservation = false;
    uint32_t lastObservationSequence = 0;
};

class NetworkManagerServer
{
private:
    NetworkManagerServer(ServerRunMode _runMode,
                         VisionObservationServerConfig _visionConfig);
    float m_TotalElapsedServerTime;
    ServerRunMode m_RunMode;
    VisionObservationServerConfig m_VisionConfig;
    std::unique_ptr<VisionObservationStore> m_VisionObservationStore;
public:
    static std::unique_ptr<NetworkManagerServer> sInstance;    
    static void StaticInit(
        ServerRunMode _runMode = ServerRunMode::AutomaticFleet,
        VisionObservationServerConfig _visionConfig = {});
public:
    void ProcessPacket(ClientProxy* _cs,InputMemoryStream& _stream);
private:
    void ProcessUnityPacket(ClientProxy* _proxy, InputMemoryStream& _stream);
    bool TryProcessRobotProtocolPacket(ClientProxy* _proxy, InputMemoryStream& _stream);
    bool TryProcessVisionProtocolPacket(ClientProxy* _proxy, InputMemoryStream& _stream);
    void HandleRobotHelloPacket(ClientProxy* _proxy, const RobotProtocol::PacketBodyHeader& _header, InputMemoryStream& _stream);
    void HandleVisionHelloPacket(ClientProxy* _proxy,
                                 const RobotProtocol::PacketBodyHeader& _header,
                                 InputMemoryStream& _stream);
    void HandleVisionObservationPacket(
        ClientProxy* _proxy,
        const RobotProtocol::PacketBodyHeader& _header,
        InputMemoryStream& _stream);
    void SendVisionHelloAck(
        ClientProxy* _proxy,
        uint32_t _sequence,
        const RobotProtocol::VisionHelloAckPayload& _payload);
    void InitializeVisionObservationStore();
    RobotSessionPtr FindRobotSession(ClientProxy* _proxy, uint32_t _agvID);
    void HandleHello_Packet(ClientProxy* _proxy,InputMemoryStream& _instream);
    void SendHello_Packet(ClientProxy* _proxy);
    void SendMap_Packet(ClientProxy* _proxy);    
    void HandleReadyMap_Packet(ClientProxy* _proxy,InputMemoryStream& _instream);
    void HandleReadyObject_Packet(ClientProxy* _proxy,InputMemoryStream& _instream);
    void CreateSimulationWorld();
    void SendPhysicalDemoRoute(uint32_t _agvID);
    void SendTrajectoryPreview(uint32_t _agvID, const RobotSessionPtr& _robotSession);
    void SendTrajectoryRaisedWheel(uint32_t _agvID, const RobotSessionPtr& _robotSession);
    void SendOutgoingVisionObservationPackets();
    void TryActivatePhysicalFleet();
    bool HandlePhysicalFleetControllerEvent(uint32_t _agvID,
                                            const ControllerEvent& _event);
    void BeginPhysicalFleetCorrection(uint32_t _agvID, uint32_t _nodeID);
    void HandlePhysicalFleetCorrectionReport(
        uint32_t _agvID,
        const ControllerEvent& _event);
    void UpdatePhysicalFleetCorrection();
    void CompletePhysicalFleetCorrection();
    void FailPhysicalFleetCorrection(const char* _reason);
private:
    std::vector<ClientProxyPtr> m_PendingProxies;
    // 접속한 클라이언트들을 관리하는 명부 (ID -> 세션 )    
    std::unordered_map<uint32_t,ClientProxy*> m_SessionIdToProxyMap;
    std::unordered_map<uint32_t, RobotSessionPtr> m_AgvIdToRobotSessionMap;
    std::unordered_map<uint32_t, MotorFaultDiagnosticDecoder>
        m_MotorFaultDiagnosticDecoders;
    std::unordered_map<ClientProxy*, RobotSessionPtr> m_ProxyToRobotSessionMap;
    std::unordered_map<ClientProxy*, VisionClientSession> m_ProxyToVisionSessionMap;
    std::unordered_map<uint32_t, ClientProxy*> m_VisionSourceToProxyMap;
    struct VisionViewerDeliveryState
    {
        uint32_t transportSequence = 0;
        RobotProtocol::VisionTrackingState trackingState =
            RobotProtocol::VisionTrackingState::LOST;
        bool poseValid = false;

        bool operator==(const VisionViewerDeliveryState&) const = default;
    };
    std::unordered_map<
        uint32_t,
        std::unordered_map<uint32_t, VisionViewerDeliveryState>>
        m_LastVisionDeliveryByUnitySession;
    static uint32_t nextSessionID;
    struct PhysicalFleetCorrectionState
    {
        enum class Phase
        {
            IDLE,
            WAITING_FOR_MEASUREMENT,
            WAITING_FOR_REPORT
        };

        Phase phase = Phase::IDLE;
        uint32_t agvID = 0;
        uint32_t nodeID = 0;
        uint32_t routeID = 0;
        uint32_t commandID = 0;
        uint32_t baselineVisionSequence = 0;
        uint8_t primitiveCount = 0;
        bool positionToleranceReached = false;
        float expectedHeadingRad = 0.0f;
        uint64_t deadlineMilliseconds = 0;

        bool active() const { return phase != Phase::IDLE; }
    };
    PhysicalFleetCorrectionState m_PhysicalFleetCorrection;
    uint32_t m_NextCorrectionCommandID = 1;
private:
    LinkingContext* m_LinkingContext; 
public: 
    void OnClientAccepted(TCPSocketPtr _tcpSocket);
    void OnClientDisconnected(ClientProxy* _proxy);
    std::vector<ClientProxyPtr> GetPendingProxies()const{return m_PendingProxies;}
public:
    LinkingContext* GetLinkingContext()const{return m_LinkingContext;}
    std::optional<StoredVisionObservation> GetLatestVisionObservation(
        uint32_t _agvID) const
    {
        if (!m_VisionObservationStore)
            return std::nullopt;
        return m_VisionObservationStore->GetLatest(_agvID);
    }
    void RegisterObject(ObjectPtr _obj);
    void SendOutgoingReplicationPackets();
private:
    bool m_IsMapReady = false;
    bool m_IsWorldCreated = false;
    bool m_IsSimulationActive=false;
    bool m_IsPhysicalFleetActivated=false;
public:
    void StartSimulation(){m_IsSimulationActive=true;}
    void StopSimulation(){m_IsSimulationActive=false;}
    void UpdateWorld(float _deltaTime);

    //todo 이거 trafficcontrolmanager로 두는게 더 좋아보임
public:
    void ReplanPath(uint32_t _agvID);
    void RequestReplan(uint32_t _agvID);
    float GetTotalElapsedServerTime()const{return m_TotalElapsedServerTime;}
};
