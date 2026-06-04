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
    case PT_INPUT:            
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
    
    int spawnCount=1;
    ObjectPtr mainRobo=nullptr;
    for(int i=0;i<spawnCount;i++)
    {
        ObjectPtr newRobo = ObjectRegistry::sInstance->CreateObject(ClassID::OBJ_AGV);
        RegisterObject(newRobo);        

        float startX=(i%5)*2.f;
        float startY=(i/5)*2.f;
        


        newRobo->SetPos(startX,startY);        
    }

    SendHello_Packet(_proxy);

    SendMap_Packet(_proxy);
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


void NetworkManagerServer::UpdateWorld(float _deltaTime)
{
    static bool a=false;
    for(auto obj:m_LinkingContext->GetAllObjects())
    {        
        ObjectPtr robo = obj.second;              
        Robo* agv = dynamic_cast<Robo*>(robo.get());        
        if(agv&&a==false)
        {
            agv->m_FinalPathNodeIDs= agv->pathFinder.FindPath(1,4,MapManager::GetInstance().GetNodes(),MapManager::GetInstance().GetLinks());    
        }
        agv->UpdateNavigation(_deltaTime,MapManager::GetInstance().GetNodes());

        for(auto iter = m_SessionIdToProxyMap.begin();iter!=m_SessionIdToProxyMap.end();iter++)
        {
           ClientProxy* proxy = iter->second;      
            proxy->GetReplicationManagerServer().SetStateDirty(robo->GetNetworkID());   
        }                
    }    
    a=true;
}