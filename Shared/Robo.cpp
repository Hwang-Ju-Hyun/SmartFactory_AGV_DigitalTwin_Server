#include "Robo.hpp"
#include "header.hpp"
#include <iostream>

Robo::Robo()
{
    SetName("Robo");
    m_ClassID=ClassID::OBJ_AGV;    
    m_Speed=0.8f;
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

    m_Progress+=m_Speed*_deltaTime;


    if(m_Progress>=1.f)
    {
        m_Progress=0.f;
        
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
    
    m_posX = fromNode.m_PosX + (toNode.m_PosX - fromNode.m_PosX) * m_Progress;
    m_posZ = fromNode.m_PosZ + (toNode.m_PosZ - fromNode.m_PosZ) * m_Progress;    

    float directionX = toNode.m_PosX - fromNode.m_PosX;
    float directionZ = toNode.m_PosZ - fromNode.m_PosZ;

    float radians = std::atan2(directionX,directionZ);
    
    this->SetHeadingAngle(radians * (180.f / 3.141592f));
    this->SetRotation(glm::angleAxis(radians, glm::vec3(0.f, 1.f, 0.f)));
}