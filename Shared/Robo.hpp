#pragma once
#include "Object.hpp"
#include <vector>
#include "Map.hpp"
#include "PathFinder.hpp"

class Robo:public Object
{       
public:
    Robo();
    virtual ~Robo()override{}    
    virtual uint32_t GetClassID()override{return m_ClassID;}
private:
    MapNode m_FromNode;
    MapNode m_ToNode;    
    float m_Speed;
    float m_Progress;
public:
    AstarPathFinder pathFinder;
    std::vector<uint32_t> m_FinalPathNodeIDs;     
    size_t m_CurrentPathIndex = 0;       // 현재 티켓의 몇 번째 정거장에 와있는지 (인덱스)   
    bool go=false;
    std::vector<uint32_t> GetFinalPathNodeIDs(){return m_FinalPathNodeIDs;}
public:
    void SetNewTargetRoute(const std::vector<uint32_t>& _newPath) 
    {
        m_FinalPathNodeIDs = _newPath;
        m_CurrentPathIndex = 0; // 새 경로를 받았으니 첫 정거장부터 출발!
        m_Progress = 0.0f;        
    }
    void UpdateNavigation(float _deltaTime,const std::unordered_map<uint32_t,MapNode>& _nodes);
};