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

void Robo::UpdateNavigation(float _deltaTime,const std::unordered_map<uint32_t,MapNode>& _nodes)
{    
    if (m_FinalPathNodeIDs.empty() || m_CurrentPathIndex >= m_FinalPathNodeIDs.size() - 1)
        return;
    
    uint32_t fromNodeID = m_FinalPathNodeIDs[m_CurrentPathIndex];
    uint32_t toNodeID = m_FinalPathNodeIDs[m_CurrentPathIndex+1];
    
    MapNode fromNode=_nodes.find(fromNodeID)->second;
    MapNode toNode=_nodes.find(toNodeID)->second;               
        
    if(m_Progress>=1.0f)
    {        
        m_AccStayTime+=_deltaTime;
        
        if(m_AccStayTime<m_StayTime)
        {
            m_posX = toNode.m_PosX;
            m_posZ = toNode.m_PosZ;
            m_Progress = 1.00001f; 
            return;
        }

        m_Progress=0.f;
        m_AccStayTime=0.f;
        m_CurrentPathIndex++;

        if(m_CurrentPathIndex>=m_FinalPathNodeIDs.size()-1)
        {                        
            return;
        }           

        
        fromNodeID = m_FinalPathNodeIDs[m_CurrentPathIndex];
        toNodeID   = m_FinalPathNodeIDs[m_CurrentPathIndex + 1];

        fromNode   = _nodes.find(fromNodeID)->second;
        toNode     = _nodes.find(toNodeID)->second;  
    }

    float dist = std::sqrt(std::pow(fromNode.m_PosX-toNode.m_PosX,2)+std::pow(fromNode.m_PosZ-toNode.m_PosZ,2));
    m_Progress+=(m_Speed/dist)*_deltaTime;
    
    m_posX = fromNode.m_PosX + (toNode.m_PosX - fromNode.m_PosX) * m_Progress;
    m_posZ = fromNode.m_PosZ + (toNode.m_PosZ - fromNode.m_PosZ) * m_Progress;    


    if(fromNodeID == toNodeID)
    {
        m_AccWaitTime += _deltaTime;

        if(m_AccWaitTime >= WaitTime)
        {
            m_AccWaitTime = 0.f;
            m_CurrentPathIndex++;
        }
        return;
    }


    float directionX = toNode.m_PosX - fromNode.m_PosX;
    float directionZ = toNode.m_PosZ - fromNode.m_PosZ;

    float radians = std::atan2(directionX,directionZ);
    
    this->SetHeadingAngle(radians * (180.f / 3.141592f));
    this->SetRotation(glm::angleAxis(radians, glm::vec3(0.f, 1.f, 0.f)));
}


void Robo::ReserveTimeLine(const std::vector<uint32_t>& _pathNode)
{    
    TrafficControlManager::GetInstance().ClearAgvReservations(this-> GetNetworkID());

    float accTime=0.f;
    float speed=m_Speed;
    float stayTime=m_StayTime; 

    // 출발 노드 예약
    uint32_t startNodeID = _pathNode[0];
    TrafficControlManager::GetInstance().ReserveNode(startNodeID, 0.0f, stayTime, this->GetNetworkID());
    
    accTime += stayTime;

    for(int i=0;i<_pathNode.size()-1;i++)
    {
        MapNode fromNode = MapManager::GetInstance().GetNodes()[_pathNode[i]];
        MapNode toNode = MapManager::GetInstance().GetNodes()[_pathNode[i+1]];

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

        float enterTime = travelTime+accTime;

        float leaveTime = enterTime+stayTime;
 
        if(i+1==_pathNode.size()-1)        
            TrafficControlManager::GetInstance().ReserveNode(toNode.m_Id, enterTime, 99999.f, GetNetworkID());            
        else
            TrafficControlManager::GetInstance().ReserveNode(toNode.m_Id, enterTime, leaveTime, GetNetworkID());

        accTime=leaveTime;
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