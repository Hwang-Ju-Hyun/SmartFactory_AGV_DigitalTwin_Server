#pragma once
#include <cstdint>
#include <vector>
#include <memory>

struct Node
{
public:
    uint32_t m_Id;

    float m_PosX;
    float m_PosY;

    uint8_t type;
};

struct Link
{
public:
    uint32_t m_Id;
    uint32_t m_FromNodeID;
    uint32_t m_ToNodeID;
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
    std::vector<Node> m_Nodes;
    std::vector<Link> m_Links;
public:
    void Init(); 
public:
    std::vector<Node> GetNodes()const{return m_Nodes;}
    std::vector<Link> GetLinks()const{return m_Links;}
};