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
#include "TaskScheduler.hpp"
#include "WarehouseManager.hpp"
#include "TaskManager.hpp"
#include "RoutePlanner.hpp"
#include "Map.hpp"
#include "Event.hpp"


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
//#define _TESTCASE2
#define _TESTCASE3
void NetworkManagerServer::HandleReadyMap_Packet(ClientProxy* _proxy,InputMemoryStream& _instream)
{
    #ifdef _TESTCASE2    
    int spawnCount=8;
    ObjectPtr mainRobo=nullptr;

    TaskManager::GetInsance();
    RoutePlanner::GetInstance().Init();
    WarehouseManager::GetInstance().Init();        

    uint32_t initNodes[8]      = {65, 66, 67, 68, 69, 70, 71, 72}; 
    
    #elifdef _TESTCASE3  
    
    int spawnCount=3;

    ObjectPtr mainRobo=nullptr;

    TaskManager::GetInsance();
    RoutePlanner::GetInstance().Init();
    WarehouseManager::GetInstance().Init();        

    uint32_t initNodes[3]      = {1,2,4}; 
  
    
    std::vector<Robo*> Robos;
    for(int i=0;i<spawnCount;i++)
    {
        ObjectPtr newRobo = ObjectRegistry::sInstance->CreateObject(ClassID::OBJ_AGV);
        RegisterObject(newRobo); 
        Robo* agv = dynamic_cast<Robo*>(newRobo.get());          
        Robos.push_back(agv);

        agv->ChangeState(AGVState::IDLE);
        agv->SetHomeNode(initNodes[i]);

        int32_t startNodeID =initNodes[i];
        MapNode startNode = MapManager::GetInstance().GetNodes().find(startNodeID)->second;
        agv->AssignNextStep(startNode, startNode, AGVState::IDLE, 0.0f); 
        agv->SetPos(startNode.m_PosX, startNode.m_PosZ);
        agv->SetCurrentNodeID(startNodeID);
        
        TrafficManager::GetInstance().ReserveNode(startNodeID, 0.0f, 100.0f, agv->GetNetworkID());        
    }        
    for (Robo* agv : Robos)
    {
        RobotEvent startEvent;
        startEvent.agvID = agv->GetNetworkID();
        startEvent.timestamp = m_TotalElapsedServerTime;
        startEvent.type = RobotEventType::IDLE_READY; // "저 백수입니다"
        
        EventManager::GetInstance().Publish(startEvent); 
    }
    #endif
}
static bool a=true;
int i=0;
#include "RoutePlanner.hpp"
void NetworkManagerServer::UpdateWorld(float _deltaTime)
{    
    if(!m_IsSimulationActive)
    {
        return;
    }    
    m_TotalElapsedServerTime+=_deltaTime;
    {
        if (m_TotalElapsedServerTime > 15.0f&&a==false)
        {
            a=true;
            
            uint32_t blockFrom = 39; // 맵에 맞게 수정
            uint32_t blockTo = 44;   // 맵에 맞게 수정

            std::cout << "\n=================================================" << std::endl;
            std::cout << blockFrom << "번 -> " << blockTo << "번 도로 동적 차단 발생 " << std::endl;
            std::cout << "=================================================\n" << std::endl;

            // 1. 맵 매니저에서 실제 링크를 막아버림 (A*가 이제 이 길을 피해서 탐색함)
            std::vector<MapLink>& links = MapManager::GetInstance().GetLinks();
            for (auto& link : links) 
            {
                if (link.m_FromNodeID == blockFrom && link.m_ToNodeID == blockTo) 
                {
                    link.m_IsBlocked = true; // 네 코드의 변수명 그대로 사용
                }
                if (link.m_ToNodeID == blockFrom && link.m_FromNodeID == blockTo) 
                {
                    link.m_IsBlocked = true; // 네 코드의 변수명 그대로 사용
                }
            }
        }   
    }
    

    for(auto obj:AGVManager::GetInstance().m_AGVs)
    {        
        ObjectPtr robo = obj;
        Robo* agv = dynamic_cast<Robo*>(robo.get());
        agv->UpdateNavigation(_deltaTime,m_TotalElapsedServerTime);             

        for(auto iter = m_SessionIdToProxyMap.begin();iter!=m_SessionIdToProxyMap.end();iter++)
        {
            ClientProxy* proxy = iter->second;
            proxy->GetReplicationManagerServer().SetStateDirty(robo->GetNetworkID());
        }        
    }          
    RoutePlanner::GetInstance().Update(_deltaTime,m_TotalElapsedServerTime);

    EventManager::GetInstance().ProcessEvents();

   
}

