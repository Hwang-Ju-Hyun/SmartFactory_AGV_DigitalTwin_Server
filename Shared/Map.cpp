#include "Map.hpp"

void MapManager::Init()
{
    MapNode node1={1,-7.f,-7.f,0};
    MapNode node2={2, 7.f,-7.f,0};
    MapNode node3={3, 7.f, 7.f,0};
    MapNode node4={4,-7.f,7.f,0};

    m_Nodes.insert({node1.m_Id,node1});
    m_Nodes.insert({node2.m_Id,node2});
    m_Nodes.insert({node3.m_Id,node3});
    m_Nodes.insert({node4.m_Id,node4});
    
    MapLink link1={1,node1.m_Id,node2.m_Id};
    MapLink link2={2,node2.m_Id,node3.m_Id};
    MapLink link3={3,node3.m_Id,node4.m_Id};        
    MapLink link4={4,node4.m_Id,node1.m_Id};            
    
    m_Links.push_back(link1);
    m_Links.push_back(link2);
    m_Links.push_back(link3);
    m_Links.push_back(link4);
}