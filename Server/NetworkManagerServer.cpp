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

    std::cout<<"[서버] 새 클라이언트 접속 승인! 부여된 ID : "<<newClientSessionID<<std::endl;         


    for(auto& obj : m_LinkingContext->GetAllObjects())
    {
        uint32_t existingNetworkID=obj.first;
        _proxy->GetReplicationManagerServer().ReplicateCreate(existingNetworkID);
    }       

    SendHello_Packet(_proxy);

    SendMap_Packet(_proxy);    

    std::cout << "[서버] 맵 전송 완료. 클라이언트의 PT_READY_MAP을 기다립니다.\n";
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


void NetworkManagerServer::HandleReadyMap_Packet(ClientProxy* _proxy,InputMemoryStream& _instream)
{
    int spawnCount=3;
    ObjectPtr mainRobo=nullptr;

    int startNodes[3]  = { 11,5,2 };
    int targetNodes[3] = { 7, 1,3 };

    std::vector<Robo*> Robos;
    for(int i=0;i<spawnCount;i++)
    {
        ObjectPtr newRobo = ObjectRegistry::sInstance->CreateObject(ClassID::OBJ_AGV);
        RegisterObject(newRobo); 
        Robo* agv = dynamic_cast<Robo*>(newRobo.get());  
        agv->ResgistGoalNode(targetNodes[i]);
        Robos.push_back(agv);
    }    
    
    for(int i=0;i<Robos.size();i++)
    {      
        Robo* agv =Robos[i];
        if(agv != nullptr)
        {            
            AstarPathFinder pathFinder;
                              
            const std::vector<uint32_t> path = pathFinder.FindPath(startNodes[i], targetNodes[i], MapManager::GetInstance().GetNodes(), MapManager::GetInstance().GetLinks(),agv->GetNetworkID());
            agv->SetNewTargetRoute(path);
            agv->ReserveTimeLine(path);              
            if (!path.empty())
            {
                uint32_t startNodeID = path[0];
                MapNode startNode = MapManager::GetInstance().GetNodes().find(startNodeID)->second;
                agv->SetPos(startNode.m_PosX, startNode.m_PosZ);
            }
        }         
    }
    
}

void NetworkManagerServer::UpdateWorld(float _deltaTime)
{
    if(!m_IsSimulationActive)
    {                              
        return;
    }           
    for(auto obj:m_LinkingContext->GetAllObjects())
    {        
        ObjectPtr robo = obj.second;              
        Robo* agv = dynamic_cast<Robo*>(robo.get());                 
        agv->UpdateNavigation(_deltaTime,MapManager::GetInstance().GetNodes());    
        for(auto iter = m_SessionIdToProxyMap.begin();iter!=m_SessionIdToProxyMap.end();iter++)
        {
           ClientProxy* proxy = iter->second;
            proxy->GetReplicationManagerServer().SetStateDirty(robo->GetNetworkID());
        }
    }
}