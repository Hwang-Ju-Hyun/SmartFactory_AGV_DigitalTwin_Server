#include "Map.hpp"

void MapManager::Init()
{
    Node node1={1,-7.f,-7.f,0};
    Node node2={2, 7.f,-7.f,0};
    Node node3={3, 7.f, 7.f,0};
    Node node4={4,-7.f,7.f,0};

    Link link1={1,node1.m_Id,node2.m_Id};
    Link link2={1,node2.m_Id,node3.m_Id};
    Link link3={1,node3.m_Id,node4.m_Id};
        
    m_Nodes.push_back(node1);
    m_Nodes.push_back(node2);
    m_Nodes.push_back(node3);
    m_Nodes.push_back(node4);
    
    m_Links.push_back(link1);
    m_Links.push_back(link2);
    m_Links.push_back(link3);
}