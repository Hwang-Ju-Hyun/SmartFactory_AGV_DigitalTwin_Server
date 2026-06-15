#include "Robo.hpp"
#include "header.hpp"
#include <iostream>
const float WaitTime=3.f;
Robo::Robo()
{
    SetName("Robo");
    m_ClassID=ClassID::OBJ_AGV;    
    m_Speed=3.8f;
    m_Progress=0.f;
}

static float acc=0;
static bool a=false;

void Robo::UpdateNavigation(float _deltaTime,const std::unordered_map<uint32_t,MapNode>& _nodes,float _serverTime)
{    

    if(m_State == AGVState::ARRIVED)
    {        
        m_TaskState=TaskState::COMPLETED;
        return;
    }        
    // acc+=_deltaTime;
    
    // if(acc>60.f&&a==false&&GetNetworkID()==2)
    // {
    //     MapLink& l1 = MapManager::GetInstance().FindLink(1,4);        
    //     MapLink& l2 = MapManager::GetInstance().FindLink(4,1);        
    //     l1.m_IsBloacked=true;
    //     l2.m_IsBloacked=true;
    //     m_NeedReplan=true;
    //     a=true;
    //     std::cout<<"링크 14 차단"<<std::endl;
    //     return;
    // }

    if (m_FinalPathNodeIDs.empty())
    {
        m_State=AGVState::IDLE;
        return;
    }
    
    uint32_t fromNodeID = m_FinalPathNodeIDs[m_CurrentPathIndex];
    uint32_t toNodeID   = m_FinalPathNodeIDs[m_CurrentPathIndex+1];
    
    MapNode fromNode=_nodes.find(fromNodeID)->second;
    MapNode toNode=_nodes.find(toNodeID)->second;               
        
    m_FromNode=fromNode;
    m_ToNode=toNode;
    m_FromNode.m_PosX=fromNode.m_PosX;
    m_FromNode.m_PosZ=fromNode.m_PosZ;    
    m_ToNode.m_PosX=toNode.m_PosX;
    m_ToNode.m_PosZ=toNode.m_PosZ;
    

    if(fromNodeID == toNodeID&&m_State==AGVState::MOVING)
    {
        m_State=AGVState::WAITING;              
        return;
    }
    
    switch (m_State)
    {        
    case AGVState::IDLE:        
        break;
    case AGVState::MOVING:    
    case AGVState::MOVE_TO_PICKUP:    
    {   
        m_TaskState =TaskState::IN_PROGRESS;

        float dist = std::sqrt(std::pow(fromNode.m_PosX - toNode.m_PosX, 2) + std::pow(fromNode.m_PosZ - toNode.m_PosZ, 2));
        m_Progress += (m_Speed / dist) * _deltaTime;

        m_posX = fromNode.m_PosX + (toNode.m_PosX - fromNode.m_PosX) * m_Progress;
        m_posZ = fromNode.m_PosZ + (toNode.m_PosZ - fromNode.m_PosZ) * m_Progress;

        float directionX = toNode.m_PosX - fromNode.m_PosX;
        float directionZ = toNode.m_PosZ - fromNode.m_PosZ;
        float radians = std::atan2(directionX, directionZ);
    
        this->SetHeadingAngle(radians * (180.f / 3.141592f));
        this->SetRotation(glm::angleAxis(radians, glm::vec3(0.f, 1.f, 0.f)));

        if (m_Progress >= 1.f)
        {
            m_TaskState=TaskState::COMPLETED;
            m_Progress = 1.f;
            TrafficControlManager::GetInstance().ReleaseNodeReservation(fromNodeID, GetNetworkID());
            
            if (m_CurrentPathIndex >= m_FinalPathNodeIDs.size() - 2)
            {
                m_State=AGVState::LOADING;
                m_CurrentPathIndex=0;
                m_AccStayTime = 0.f;
            }
            else
            {
                // 아직 중간 경로를 지나가는 중이라면 다음 링크를 향해 계속 전진!
                m_Progress = 0.f;
                m_CurrentPathIndex++;
            }
            
        }
    }
    break;    
    case AGVState::MOVE_TO_DROP:
    {
        if(GetNetworkID()==4)
            int a=0;
        // 주행 물리 연산은 완벽하게 동일합니다.
        float dist = std::sqrt(std::pow(fromNode.m_PosX - toNode.m_PosX, 2) + std::pow(fromNode.m_PosZ - toNode.m_PosZ, 2));
        m_Progress += (m_Speed / dist) * _deltaTime;
            
        m_posX = fromNode.m_PosX + (toNode.m_PosX - fromNode.m_PosX) * m_Progress;
        m_posZ = fromNode.m_PosZ + (toNode.m_PosZ - fromNode.m_PosZ) * m_Progress;

        float directionX = toNode.m_PosX - fromNode.m_PosX;
        float directionZ = toNode.m_PosZ - fromNode.m_PosZ;
        float radians = std::atan2(directionX, directionZ);
    
        this->SetHeadingAngle(radians * (180.f / 3.141592f));
        this->SetRotation(glm::angleAxis(radians, glm::vec3(0.f, 1.f, 0.f)));
        
        if (m_Progress >= 1.f)
        {
            m_Progress = 1.f;
            // 내가 지나온 옛날 노드 장부 반납
            TrafficControlManager::GetInstance().ReleaseNodeReservation(fromNodeID, GetNetworkID());

            // 현재 도달한 노드가 최종 경로의 맨 마지막 노드(즉, 하역장 정중앙)인지 체크합니다.
            if (m_CurrentPathIndex >= m_FinalPathNodeIDs.size() - 1)
            {                
                // 하차(물건 내리기) 작업을 위해 STAYING 상태로 전환하고 타이머 리셋
                m_State = AGVState::UNLOADING;
                m_CurrentPathIndex=0;
                m_AccStayTime = 0.f;
            }
            else
            {
                // 아직 중간 경로를 지나가는 중이라면 다음 링크를 향해 계속 전진!
                m_Progress = 0.f;
                m_CurrentPathIndex++;
            }
        }
    }
    break;
    case AGVState::MOVE_TO_HOME:
    {
        // 주행 물리 연산은 완벽하게 동일합니다.
        float dist = std::sqrt(std::pow(fromNode.m_PosX - toNode.m_PosX, 2) + std::pow(fromNode.m_PosZ - toNode.m_PosZ, 2));
        m_Progress += (m_Speed / dist) * _deltaTime;
            
        m_posX = fromNode.m_PosX + (toNode.m_PosX - fromNode.m_PosX) * m_Progress;
        m_posZ = fromNode.m_PosZ + (toNode.m_PosZ - fromNode.m_PosZ) * m_Progress;

        float directionX = toNode.m_PosX - fromNode.m_PosX;
        float directionZ = toNode.m_PosZ - fromNode.m_PosZ;
        float radians = std::atan2(directionX, directionZ);
    
        this->SetHeadingAngle(radians * (180.f / 3.141592f));
        this->SetRotation(glm::angleAxis(radians, glm::vec3(0.f, 1.f, 0.f)));
        
        if (m_Progress >= 1.f)
        {
            m_Progress = 1.f;
            // 내가 지나온 옛날 노드 장부 반납
            TrafficControlManager::GetInstance().ReleaseNodeReservation(fromNodeID, GetNetworkID());

            // 현재 도달한 노드가 최종 경로의 맨 마지막 노드(즉, 하역장 정중앙)인지 체크합니다.
            if (m_CurrentPathIndex >= m_FinalPathNodeIDs.size() - 1)
            {                
                // 하차(물건 내리기) 작업을 위해 STAYING 상태로 전환하고 타이머 리셋
                m_State = AGVState::STAYING;
                m_AccStayTime = 0.f;
            }
            else
            {
                // 아직 중간 경로를 지나가는 중이라면 다음 링크를 향해 계속 전진!
                m_Progress = 0.f;
                m_CurrentPathIndex++;
            }
        }
    }
    break;
    case AGVState::RETURNING:
    {        
        m_TaskState=TaskState::IN_PROGRESS;
        float totalLinkDist = std::sqrt(std::pow(toNode.m_PosX - fromNode.m_PosX, 2) + 
                                        std::pow(toNode.m_PosZ - fromNode.m_PosZ, 2));
        std::cout<<"리터닝 중중 중 "<<std::endl;
        if (totalLinkDist < 0.001f) 
        {
            m_Progress = 0.0f;
        } 
        else 
        {
            m_Progress -= (m_Speed / totalLinkDist) * _deltaTime;
        }
        
        m_posX = fromNode.m_PosX + (toNode.m_PosX - fromNode.m_PosX) * m_Progress;
        m_posZ = fromNode.m_PosZ + (toNode.m_PosZ - fromNode.m_PosZ) * m_Progress;
                
        float directionX = fromNode.m_PosX - m_posX;
        float directionZ = fromNode.m_PosZ - m_posZ;
        if (std::abs(directionX) > 0.001f || std::abs(directionZ) > 0.001f)
        {
            float radians = std::atan2(directionX, directionZ);
            this->SetHeadingAngle(radians * (180.f / 3.141592f));
            this->SetRotation(glm::angleAxis(radians, glm::vec3(0.f, 1.f, 0.f)));
        }

        
        if (m_Progress <= 0.0f)
        {
            m_Progress = 0.0f;                                 
                        
            m_State = AGVState::WAITING; 
            m_AccWaitTime = 0.f; 
            isComeBackDone=true;
        }        
    }
    break;
    case AGVState::WAITING:
    {
        m_TaskState=TaskState::IN_PROGRESS;        
        m_posX = fromNode.m_PosX;
        m_posZ = fromNode.m_PosZ;

        m_AccWaitTime += _deltaTime;
        
        if(m_AccWaitTime >= WaitTime)
        {
            m_AccWaitTime = 0.f;
            m_Progress=0.f;
            m_CurrentPathIndex++;

            if(m_CurrentPathIndex>=m_FinalPathNodeIDs.size()-1)
            {
                TrafficControlManager::GetInstance().ReleaseNodeReservation(fromNodeID,GetNetworkID());
                m_State=AGVState::ARRIVED;
            }
            else
            {                
                m_State=AGVState::MOVING;
            }
        }
    }
    break;    
    case AGVState::LOADING:
    {
        m_TaskState=TaskState::IN_PROGRESS;
        m_AccStayTime += _deltaTime;        
        auto a= m_posX;
        auto b = m_posZ;
        m_Progress = 1.00001f; 

        if(m_AccStayTime > m_StayTime)
        {
            m_Progress = 0.f;
            m_AccStayTime = 0.f;
            
            std::cout << "[AGV " << GetNetworkID() << "] 물건 가져옴 이제 갔다 버리러 가자." << std::endl;

            TrafficControlManager::GetInstance().ClearAgvReservations(GetNetworkID());
         
            m_State = AGVState::MOVE_TO_DROP;

            TaskScheduler::GetInstance().AssignUnLoadRoute(_serverTime,this);
            m_CurrentPathIndex = 0;
        }
    }
    break;
    case AGVState::UNLOADING:
    {
        m_AccStayTime += _deltaTime;        
        m_posX = toNode.m_PosX;
        m_posZ = toNode.m_PosZ;
        m_Progress = 1.00001f; 

        if(m_AccStayTime > m_StayTime)
        {
            m_Progress = 0.f;
            m_AccStayTime = 0.f;

            std::cout << "[AGV " << GetNetworkID() << "] 부품 상차 완료 ➔ 하역장(DROP)으로 후반전 출발!" << std::endl;                                    
                
            m_CurrentPathIndex = 0;

            m_State = AGVState::ARRIVED;
        }
    }
    break;
    case AGVState::ARRIVED:
    {
        m_State=AGVState::MOVING;                
        m_TaskState=TaskState::COMPLETED;
    }        
        break;
    default:
        break;
    }
            
}


void Robo::ReserveTimeLine(const std::vector<uint32_t>& _pathNode,float _serverTime)
{    
    TrafficControlManager::GetInstance().ClearAgvReservations(this-> GetNetworkID());

    float accTime=_serverTime;
    float speed=m_Speed;
    float stayTime=m_StayTime; 

    // 출발 노드 예약
    uint32_t startNodeID = _pathNode[0];
    TrafficControlManager::GetInstance().ReserveNode(startNodeID, accTime, stayTime, this->GetNetworkID());
    
    accTime += stayTime;

    for(int i=0;i<_pathNode.size()-1;i++)
    {
        MapNode fromNode = MapManager::GetInstance().GetNodes()[_pathNode[i]];
        MapNode toNode = MapManager::GetInstance().GetNodes()[_pathNode[i+1]];

        //wait
        if(fromNode.m_Id == toNode.m_Id)
        {
            float enterTime = accTime;
            float leaveTime = accTime + WaitTime;

            TrafficControlManager::GetInstance().ReserveNode(toNode.m_Id,enterTime,leaveTime,GetNetworkID());
            accTime = leaveTime;
            continue;
        }
        
        float dist=std::sqrt(std::pow(fromNode.m_PosX-toNode.m_PosX,2)+std::pow(fromNode.m_PosZ-toNode.m_PosZ,2));

        //시간=거리/속도
        float travelTime = dist/speed;
       
        float linkEnterTime = accTime;               
        float linkLeaveTime = accTime + travelTime;  
        
        TrafficControlManager::GetInstance().ReserveLink(fromNode.m_Id,toNode.m_Id,linkEnterTime,linkLeaveTime,GetNetworkID());
        
        float nodeEnterTime = linkLeaveTime;
        float nodeLeaveTime = nodeEnterTime+stayTime;
 
        if(i+1==_pathNode.size()-1)        
            TrafficControlManager::GetInstance().ReserveNode(toNode.m_Id, nodeEnterTime, 99999.f, GetNetworkID());            
        else
            TrafficControlManager::GetInstance().ReserveNode(toNode.m_Id, nodeEnterTime, nodeLeaveTime, GetNetworkID());

        accTime=nodeLeaveTime;
    }    
}

MapNode Robo::GetCurrentNode()
{    
    MapNode curNode = FindNearestNode(m_posX,m_posZ);
    return curNode;
}

#include <cmath>
MapNode Robo::FindNearestNode(float _x,float _z)
{
    std::unordered_map<uint32_t,MapNode> nodes = MapManager::GetInstance().GetNodes();

    float min_len=1e9;

    MapNode currentNode;
    
    for(auto mn:nodes)
    {                        
        float length = std::sqrt(std::pow(mn.second.m_PosX-_x,2)+std::pow(mn.second.m_PosZ-_z,2));
        if(min_len>length)
        {
            currentNode=mn.second;
            min_len=length;
        }
    }
    return currentNode;
}

float Robo::GetTimeSpentOnCurrentLink_FromNode()
{
    auto x = GetPosX();
    auto z =GetPosZ();
    float dist=std::sqrt(std::pow(m_FromNode.m_PosX-x,2)+std::pow(m_FromNode.m_PosZ-z,2));         
    float travelTime = dist/m_Speed;
    return travelTime;
}

float Robo::GetTimeSpendOnCurrentLink_ToNode()
{
    auto x = GetPosX();
    auto z =GetPosZ();
    float dist=std::sqrt(std::pow(m_ToNode.m_PosX-x,2)+std::pow(m_ToNode.m_PosZ-z,2));         
    float travelTime = dist/m_Speed;
    return travelTime;
}
