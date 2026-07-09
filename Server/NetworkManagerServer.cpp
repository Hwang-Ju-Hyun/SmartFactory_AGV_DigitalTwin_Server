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
    uint8_t packet_type;
    _inStream.Read(packet_type);
    switch (packet_type)
    {
    case PT_Hello:        
        HandleHello_Packet(_session,_inStream);
        break;
    case PT_Replication:   
        break;
    case PT_MAZE_DATA:
        /* code */
        break;
    case PT_Disconnected:
        /* code */
        break;        
    case PT_READY_MAP:
        HandleReadyMap_Packet(_session,_inStream);
        break;
    case PT_READY_OBJECT:
        HandleReadyObject_Packet(_session,_inStream);
        break;
    default:  
        printf("Inavalid PacketData\n\a");
        break;
    }
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

    std::cout << "[서버] 맵 전송 완료. PT_READY_MAP 대기 중 \n";
}

void NetworkManagerServer::SendHello_Packet(ClientProxy* _proxy)
{
    OutputMemoryStream outStream;
    uint8_t packetType=PacketType::PT_Hello;    

    outStream.Write(packetType);    
    
    outStream.Write(_proxy->GetSessionID());

    _proxy->SendPacket(outStream);
}

void NetworkManagerServer::SendMap_Packet(ClientProxy* _proxy)
{
    const std::unordered_map<uint32_t,MapNode> nodes = MapManager::GetInstance().GetNodes();
    std::vector<MapLink> links = MapManager::GetInstance().GetLinks();

    OutputMemoryStream outStream;
    uint8_t packetType=PacketType::PT_MAZE_DATA;
    
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
        PacketType packetType = PacketType::PT_Replication;
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
    ObjectPtr mainRobo = nullptr;

    TaskManager::GetInsance();
    RoutePlanner::GetInstance().Init();
    WarehouseManager::GetInstance().Init();        

    uint32_t initNodes[22] = {75,65,76,73,66,74,68,67,69,70,71,72,20,21,22,23,24,39,38,40,37,41}; 
    
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
        agv->SetCurrentNodeID(startNodeID);
                
        RobotManager::GetInstance().RegisterRobot(agv->GetNetworkID(), std::make_unique<UnityRobotController>());
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
    if(!m_IsSimulationActive)
    {
        return;
    }    
    m_TotalElapsedServerTime+=_deltaTime;    

    // ========================================================
    // [동적 장애물 테스트 코드] 15초마다 39번<->44번 도로 토글
    // ========================================================
    static float lastToggleTime = 0.0f;
    static bool isBlockedNow = false;

    // 15초마다 한 번씩 토글 발동
    if (m_TotalElapsedServerTime - lastToggleTime > 1500000.0f)
    {
        lastToggleTime = m_TotalElapsedServerTime;
        isBlockedNow = !isBlockedNow; // 상태 반전 (막힘 <-> 뚫림)
        
        uint32_t blockFrom = 7; // 테스트할 노드 ID 1
        uint32_t blockTo = 8;   // 테스트할 노드 ID 2
        std::cout << "\n=================================================" << std::endl;

        if (isBlockedNow)
            std::cout << " [신의 손] " << blockFrom << "번 <-> " << blockTo << "번 도로 [차단] 발동!" << std::endl;
        else
            std::cout << " [신의 손] " << blockFrom << "번 <-> " << blockTo << "번 도로 [해제] 발동!" << std::endl;
        std::cout << "=================================================\n" << std::endl;

        // 맵 매니저에서 실제 링크 속성 변경
        std::vector<MapLink>& links = MapManager::GetInstance().GetLinks();
        for (auto& link : links) 
        {
            // 양방향 모두 처리
            if ((link.m_FromNodeID == blockFrom && link.m_ToNodeID == blockTo) ||
                (link.m_FromNodeID == blockTo && link.m_ToNodeID == blockFrom)) 
            {
                link.m_IsBlocked = isBlockedNow;                            
            }
        }
        RoutePlanner::GetInstance().ClearRRAEngines();
    }
    // ========================================================


    for (auto it = RobotManager::GetInstance().GetRobotControllers().begin(); it != RobotManager::GetInstance().GetRobotControllers().end(); ++it)
    {
        uint32_t agvID = it->first;
        IRobotController* controller = it->second.get();

        while (controller->HasEvent())
        {            
            ControllerEvent ev = controller->PopEvent();
            if (ev.type == ControllerEventType::ARRIVED)
            {
                RobotEvent arrivedEvent;
                arrivedEvent.agvID = agvID;
                arrivedEvent.timestamp = m_TotalElapsedServerTime;
                arrivedEvent.currentNodeID = ev.nodeID;
                arrivedEvent.type = RobotEventType::NODE_ARRIVED;
                std::cout<<"ARRIVED"<<std::endl;
                // 이벤트 큐에 차곡차곡 쌓아둡니다.
                EventManager::GetInstance().Publish(arrivedEvent);
            }
        }
    }

    // ========================================================
    //[STEP 2: 장부 갱신] 관제탑이 이벤트를 읽고, 노드 점유(Occupy) 및 다음 길을 계산합니다.
    // ========================================================
    EventManager::GetInstance().SwapAndProcessEvents(); // 여기서 OccupyNode가 호출됨!
    RoutePlanner::GetInstance().Update(_deltaTime, m_TotalElapsedServerTime);

    // ========================================================
    //[STEP 3: 로봇 구동] 완벽하게 갱신된 장부를 바탕으로 로봇들이 출발(Edge 점유)을 결정합니다!
    // ========================================================
    RobotManager::GetInstance().Update(_deltaTime, m_TotalElapsedServerTime);

    
    // ========================================================
    // [STEP 4: 좌표 동기화] (기존 코드 유지)
    // ========================================================
    for (auto it = RobotManager::GetInstance().GetRobotControllers().begin(); it != RobotManager::GetInstance().GetRobotControllers().end(); ++it)
    {
        uint32_t agvID = it->first;
        StatusPacket status = it->second->GetStatus();

        ObjectPtr obj = m_LinkingContext->GetObject(agvID);
        if (obj && obj->GetClassID() == ClassID::OBJ_AGV) {
            if (Robo* agv = dynamic_cast<Robo*>(obj.get())) {
                agv->SetPos(status.x, status.z);
            }
        }
    }

    // 타이머 및 Replication (기존 코드 유지)
    for(auto obj : AGVManager::GetInstance().m_AGVs)
    {        
        Robo* agv = dynamic_cast<Robo*>(obj.get());
        agv->UpdateWorkTimer(_deltaTime, m_TotalElapsedServerTime); 

        for(auto iter = m_SessionIdToProxyMap.begin(); iter != m_SessionIdToProxyMap.end(); iter++) {
            iter->second->GetReplicationManagerServer().SetStateDirty(agv->GetNetworkID());
        }
    }
}

