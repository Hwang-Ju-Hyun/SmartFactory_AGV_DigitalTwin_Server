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

std::unique_ptr<NetworkManagerServer> NetworkManagerServer::sInstance=nullptr;

uint32_t NetworkManagerServer::nextSessionID=1;

NetworkManagerServer::NetworkManagerServer()
{
    m_LinkingContext = new LinkingContext;
}

void NetworkManagerServer::StaticInit()
{
    sInstance.reset(new NetworkManagerServer());    
    ObjectRegistry::sInstance->RegisterCreationFunction(ClassID::OBJ_AGV,RoboServer::StaticCreate);
    srand((unsigned int)time(NULL));
}

void NetworkManagerServer::ProcessPacket(ClientProxy* _session,InputMemoryStream& _inStream) 
{
    if (TryProcessRobotProtocolPacket(_session, _inStream))
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
    if (!RobotProtocol::PeekPacketID(_stream, packetID))
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

    RobotSessionPtr robotSession = std::make_shared<RobotSession>(_proxy->GetSession(), assignedAgvID);

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

    RobotManager::GetInstance().RegisterRobot(
        assignedAgvID,
        std::make_unique<ESP32RobotController>(robotSession)
    );

    robotSession->SendHelloAck(ack);

    std::cout << "[RobotProtocol] Robot client connected. agvID=" << assignedAgvID
              << " clientType=" << static_cast<int>(hello.clientType)
              << " sequence=" << _header.sequence << "\n";
}

RobotSessionPtr NetworkManagerServer::FindRobotSession(ClientProxy* _proxy, uint32_t _agvID)
{
    auto proxyIt = m_ProxyToRobotSessionMap.find(_proxy);
    if (proxyIt != m_ProxyToRobotSessionMap.end())
        return proxyIt->second;

    auto agvIt = m_AgvIdToRobotSessionMap.find(_agvID);
    if (agvIt != m_AgvIdToRobotSessionMap.end())
        return agvIt->second;

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
}

void NetworkManagerServer::HandleReadyObject_Packet(ClientProxy* _proxy,InputMemoryStream& _instream)
{ 
    StartSimulation();
}

//#define _TESTCASE0
//#define _TESTCASE1
#define _TESTCASE2
//#define _TESTCASE3
//#define _TESTCASE4
void NetworkManagerServer::HandleReadyMap_Packet(ClientProxy* _proxy, InputMemoryStream& _instream)
{
    #ifdef _TESTCASE0
    int spawnCount = 4;
    ObjectPtr mainRobo = nullptr;
    TaskManager::GetInsance();
    RoutePlanner::GetInstance().Init();
    WarehouseManager::GetInstance().Init();        
    uint32_t initNodes[4] = {1,2,3,4}; 

    #elifdef _TESTCASE1
    int spawnCount = 22;
    ObjectPtr mainRobo = nullptr;
    TaskManager::GetInsance();
    RoutePlanner::GetInstance().Init();
    WarehouseManager::GetInstance().Init();        
    uint32_t initNodes[22] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22}; 

    #elifdef _TESTCASE2    
    int spawnCount = 22;
    //int spawnCount = 12;
    ObjectPtr mainRobo = nullptr;

    TaskManager::GetInsance();
    RoutePlanner::GetInstance().Init();
    WarehouseManager::GetInstance().Init();        

    uint32_t initNodes[22] = {75,65,76,73,66,74,68,67,69,70,71,72,20,21,22,23,24,39,38,40,37,41}; 
    //uint32_t initNodes[12] = {75,65,76,73,66,74,68,67,69,70,71,72};
    #elifdef _TESTCASE3  
    
    int spawnCount = 5;

    ObjectPtr mainRobo = nullptr;

    TaskManager::GetInsance();
    RoutePlanner::GetInstance().Init();
    WarehouseManager::GetInstance().Init();        

    uint32_t initNodes[5] = {1, 2, 3, 4, 5};
    
    #elifdef _TESTCASE4
    int spawnCount = 2;
    ObjectPtr mainRobo = nullptr;

    TaskManager::GetInsance();
    RoutePlanner::GetInstance().Init();
    WarehouseManager::GetInstance().Init();        

    uint32_t initNodes[2] = {1,2};
    #endif

    std::vector<Robo*> Robos;
    for(int i = 0; i < spawnCount; i++)
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
        agv->SetHeadingAngle(90.0f);
        agv->SetCurrentNodeID(startNodeID);
                
        RobotManager::GetInstance().RegisterRobot(
            agv->GetNetworkID(),
            std::make_unique<UnityRobotController>(agv->GetNetworkID(), startNode.m_PosX, startNode.m_PosZ)
        );
        OccupancyProvider::GetInstance().OccupyNode(agv->GetNetworkID(), startNodeID);
        ReservationTable::GetInstance().ReserveNode(startNodeID, 0.0f, 100.0f, agv->GetNetworkID(), ReservationType::Goal);        
    }     

    for (Robo* agv : Robos)
    {
        RobotEvent startEvent;
        startEvent.agvID = agv->GetNetworkID();
        startEvent.timestamp = m_TotalElapsedServerTime;
        startEvent.type = RobotEventType::IDLE_READY; // "저 백수입니다"
        
        EventManager::GetInstance().Publish(startEvent); 
    }
   
}

static bool a=true;
int i=0;


void NetworkManagerServer::UpdateWorld(float _deltaTime)
{   
    if(!m_IsSimulationActive) return;
    m_TotalElapsedServerTime += _deltaTime;    

    // 1. INPUT 레이어: 컨트롤러 패킷 수거
    for (auto it = RobotManager::GetInstance().GetRobotControllers().begin(); it != RobotManager::GetInstance().GetRobotControllers().end(); ++it) {
        while (it->second->HasEvent()) {            
            ControllerEvent ev = it->second->PopEvent();
            if (ev.type == ControllerEventType::ARRIVED) {
                EventManager::GetInstance().Publish({ RobotEventType::NODE_ARRIVED, it->first, m_TotalElapsedServerTime, ev.nodeID });
            }
            else if (ev.type == ControllerEventType::EXECUTION_BLOCKED) {
                RoutePlanner::GetInstance().OnExecutionBlocked(it->first, ev.nodeID, ev.relatedNodeID, m_TotalElapsedServerTime);
            }
        }
    }

    // 2. LOGIC 레이어: 장부 갱신 및 시공간 설계
    EventManager::GetInstance().SwapAndProcessEvents(); 
    RoutePlanner::GetInstance().Update(_deltaTime, m_TotalElapsedServerTime);
    for (auto& agvObj : AGVManager::GetInstance().m_AGVs) {
        if (Robo* agv = dynamic_cast<Robo*>(agvObj.get())) {
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
