#pragma once
#include <cstdint>
#include <vector>
#include <memory>
#include <unordered_map>

struct MapNode
{
public:
    uint32_t m_Id;

    float m_PosX;
    float m_PosZ;

    uint8_t type;
};

struct MapLink
{
public:
    uint32_t m_Id;
    uint32_t m_FromNodeID;
    uint32_t m_ToNodeID;
    bool m_IsBloacked=false;
};

class MapManager
{
private:
    MapManager(){Init();}
public:
    static MapManager& GetInstance()
    {
        static MapManager s;        
        return s;
    }
private:    
    std::unordered_map<uint32_t,MapNode> m_Nodes;
    std::vector<MapLink> m_Links;
public:
    void Init(); 
public:
    std::unordered_map<uint32_t,MapNode>  GetNodes()const{return m_Nodes;}
    std::vector<MapLink> GetLinks()const{return m_Links;}
public:
    MapNode GetMapNode(uint32_t _nodeID){return m_Nodes.find(_nodeID)->second;}
    MapLink FindLink(uint32_t _linkID);
    MapLink& FindLink(uint32_t _fromNodeID,uint32_t _toNodeID);
};