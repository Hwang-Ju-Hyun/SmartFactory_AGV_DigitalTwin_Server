#pragma once
#include "Object.hpp"
#include <vector>
#include "Map.hpp"
#include "PathFinder.hpp"
#include "TrafficControlManager.hpp"

enum class AGVState
{
    IDLE,
    MOVING,
    STAYING,
    WAITING,
    ARRIVED
};

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
    float m_StayTime=1.f;
    float m_AccStayTime=0.f;
    float m_AccWaitTime=0.f;
    bool IsStayTime=false;

    AGVState m_State;
public:
    MapNode m_GoalNode;
    AstarPathFinder pathFinder;
    std::vector<uint32_t> m_FinalPathNodeIDs;     
    size_t m_CurrentPathIndex = 0;       // 현재 티켓의 몇 번째 정거장에 와있는지 (인덱스)       
    std::vector<uint32_t> GetFinalPathNodeIDs(){return m_FinalPathNodeIDs;}
public:
    void ReserveTimeLine(const std::vector<uint32_t>& _pathNode);
    void SetNewTargetRoute(const std::vector<uint32_t>& _newPath) 
    {
        m_FinalPathNodeIDs = _newPath;
        m_CurrentPathIndex = 0;
        m_Progress = 0.0f;             
    }
    void UpdateNavigation(float _deltaTime,const std::unordered_map<uint32_t,MapNode>& _nodes);    

    MapNode GetCurrentNode();
    MapNode FindNearestNode(float _x,float _z);    
    void SetGoalNode(uint32_t _nodeID){m_GoalNode=MapManager::GetInstance().GetMapNode(_nodeID);}
    MapNode GetGoalNode(){return m_GoalNode;}

    
};