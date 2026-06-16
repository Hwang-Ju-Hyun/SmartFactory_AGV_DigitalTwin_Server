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

void NetworkManagerServer::HandleReadyMap_Packet(ClientProxy* _proxy,InputMemoryStream& _instream)
{
    int spawnCount=5;
    ObjectPtr mainRobo=nullptr;

    TaskManager::GetInsance();

    uint32_t initNodes[5]      = { 1,   3,   5,   7,    9};
    uint32_t storeNodes[5]     = { 41,  42,  43,  44,   45};
    uint32_t dispatchNodes[5]  = { 1,  3,  5,  6,  8};


    std::vector<Robo*> Robos;
    for(int i=0;i<spawnCount;i++)
    {
        ObjectPtr newRobo = ObjectRegistry::sInstance->CreateObject(ClassID::OBJ_AGV);
        RegisterObject(newRobo); 
        Robo* agv = dynamic_cast<Robo*>(newRobo.get());          
        Robos.push_back(agv);
    }
    
    for(int i=0;i<Robos.size();i++)
    {      
        Robo* agv =Robos[i];
        if(agv != nullptr)
        {            
            TrafficControlManager::GetInstance().ClearAgvReservations(agv->GetNetworkID());
            TrafficControlManager::GetInstance().ClearLinkReservations(agv->GetNetworkID());
            AstarPathFinder pathFinder;        

            const std::vector<uint32_t> path = pathFinder.FindPath(initNodes[i], storeNodes[i], MapManager::GetInstance().GetNodes(), MapManager::GetInstance().GetLinks(),agv->GetNetworkID(),m_TotalElapsedServerTime);
            
            agv->SetGoalNode(storeNodes[i]);
            agv->SetNewTargetRoute(path);
            agv->ReserveTimeLine(path,m_TotalElapsedServerTime);
            agv->ChangeState(AGVState::MOVE_TO_PICKUP);      
            agv->SetHomeNode(initNodes[i]);

            //auto path = agv->GetFinalPathNodeIDs();
            if (!path.empty())
            {
                uint32_t startNodeID = path[0];
                MapNode startNode = MapManager::GetInstance().GetNodes().find(startNodeID)->second;
                agv->SetPos(startNode.m_PosX, startNode.m_PosZ);
            }
        }
    }

    TrafficControlManager::GetInstance().ValidateReservation();
    
    std::vector<Conflict> conflicts = TrafficControlManager::GetInstance().GetConflicts();

    for(int i=0;i<conflicts.size();i++)
    {
        uint32_t loserAGV=TrafficControlManager::GetInstance().GetLoserAGVofConflict(conflicts[i]);
        ReplanPath(loserAGV);
    }
}
static bool a=false;
int i=0;

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
        agv->UpdateNavigation(_deltaTime,MapManager::GetInstance().GetNodes(),m_TotalElapsedServerTime);

        if(agv->m_NeedReplan&&agv->GetNetworkID()==2)
        {                   
            AstarPathFinder apf;
            agv->ChangeState(AGVState::RETURNING); 
            if(agv->isComeBackDone) 
            {
                RequestReplan(agv->GetNetworkID());
                agv->m_NeedReplan=false;
                agv->isComeBackDone=false;
                agv->ChangeState(AGVState::MOVING);
            }                            
        }             

        for(auto iter = m_SessionIdToProxyMap.begin();iter!=m_SessionIdToProxyMap.end();iter++)
        {
            ClientProxy* proxy = iter->second;
            proxy->GetReplicationManagerServer().SetStateDirty(robo->GetNetworkID());
        }        
    }   

    //WarehouseManager::GetInstance().Update();

    //TaskScheduler::GetInstance().UpdateSchedule(m_TotalElapsedServerTime);

    TrafficControlManager::GetInstance().ValidateReservation();
    
    std::vector<Conflict> conflicts = TrafficControlManager::GetInstance().GetConflicts();

    for(int i=0;i<conflicts.size();i++)
    {
        std::cout<<"충돌 발생"<<std::endl;
        uint32_t loserAGV=TrafficControlManager::GetInstance().GetLoserAGVofConflict(conflicts[i]);
        Robo* agv = AGVManager::GetInstance().FindAGV(loserAGV);
        TaskScheduler::GetInstance().ReplanPath(loserAGV,m_TotalElapsedServerTime,agv->GetState());
    }


    TaskScheduler::GetInstance().Update(_deltaTime,m_TotalElapsedServerTime);
    m_TotalElapsedServerTime+=_deltaTime;
}

void NetworkManagerServer::ReplanPath(uint32_t _agvID)
{
    TrafficControlManager::GetInstance().ClearAgvReservations(_agvID);
    ObjectPtr obj=m_LinkingContext->GetObject(_agvID);
    Robo* agv=dynamic_cast<Robo*>(obj.get());

    MapNode curNode = agv->GetCurrentNode();
    MapNode goalNode =agv->GetGoalNode();

    AstarPathFinder apf;
    
    std::vector<uint32_t> path = apf.FindPath(curNode.m_Id,goalNode.m_Id,MapManager::GetInstance().GetNodes(),MapManager::GetInstance().GetLinks(),_agvID,m_TotalElapsedServerTime);    
    
    if(!path.empty())
    {
        agv->SetNewTargetRoute(path);

        agv->ReserveTimeLine(path,m_TotalElapsedServerTime);
    }
    else
    {
        std::cout << "[스케줄러] 트래픽 체증 감지! AGV " << _agvID 
                  << "번 경로 탐색 실패. 1.5초 뒤 재시도 큐에 등록합니다." << std::endl;
                
        // (현재 시간부터 넉넉하게 5초 동안 점유)
        TrafficControlManager::GetInstance().ReserveNode(curNode.m_Id, m_TotalElapsedServerTime, m_TotalElapsedServerTime+ 5.0f, _agvID);                       

        // 로봇 상태는 WAITING(대기)으로 묶어둠
        agv->ChangeState(AGVState::WAITING);
    }
    
}

void NetworkManagerServer::RequestReplan(uint32_t _agvID)
{    
    ObjectPtr obj = m_LinkingContext->GetObject(_agvID);

    Robo* agv = dynamic_cast<Robo*>(obj.get());
    if (!agv) 
        return;
    
    size_t currentPathIndex= agv->GetCurrentPathIndex();
    std::vector<uint32_t> oldPath = agv->GetFinalPathNodeIDs();

    uint32_t fromNodeID = oldPath[currentPathIndex];
    uint32_t nextNodeID   = oldPath[currentPathIndex+1];
    uint32_t goalNodeID = oldPath[oldPath.size()-1];    

    // 기존 장부 반납
    TrafficControlManager::GetInstance().ClearAgvReservations(_agvID);
    
    MapLink currentLink = MapManager::GetInstance().FindLink(fromNodeID,nextNodeID);
    
    uint32_t newStartNodeID=nextNodeID;
        
    bool IsCurrentLinkBlocked=currentLink.m_IsBloacked;
    
    //담 노드까지 가는데 걸리는 시간
    float remainingTime=agv->GetTimeSpendOnCurrentLink_ToNode();


    if(IsCurrentLinkBlocked)
    {
        newStartNodeID=fromNodeID;

        //방금 전 노드로 돌아가는 시간
        remainingTime = agv->GetTimeSpentOnCurrentLink_FromNode();
    }

    float newStartTime=m_TotalElapsedServerTime+remainingTime;

    AstarPathFinder apf;
    std::vector<uint32_t> newPath = apf.FindPath(newStartNodeID,goalNodeID,MapManager::GetInstance().GetNodes(),MapManager::GetInstance().GetLinks(),_agvID,newStartTime);

    if(IsCurrentLinkBlocked)
    {
        agv->SetNewTargetRoute(newPath);
        agv->SetCurrentIndex(0);
    }
    else
    {
        //먼 전방이 막혔을때 일단 agv가 가던 노드는 마저가야함
        newPath.insert(newPath.begin(),fromNodeID);
        agv->SetNewTargetRoute(newPath);
        agv->SetCurrentIndex(agv->GetCurrentPathIndex());
    }

    agv->ReserveTimeLine(newPath,newStartTime);
}