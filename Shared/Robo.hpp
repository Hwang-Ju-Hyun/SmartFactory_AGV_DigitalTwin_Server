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
    RETURNING,
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
    float m_BackProgress;
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
    void ReserveTimeLine(const std::vector<uint32_t>& _pathNode,float _serverTime=0.f);
    void SetNewTargetRoute(const std::vector<uint32_t>& _newPath) 
    {
        m_FinalPathNodeIDs = _newPath;
        m_CurrentPathIndex = 0;
        m_Progress = 0.0f;             
    }
    void UpdateNavigation(float _deltaTime,const std::unordered_map<uint32_t,MapNode>& _nodes);    

    MapNode GetCurrentNode();
    void SetCurrentIndex(uint32_t _idx){m_CurrentPathIndex=_idx;}
    MapNode FindNearestNode(float _x,float _z);    
    size_t GetCurrentPathIndex(){return m_CurrentPathIndex;}
    void SetGoalNode(uint32_t _nodeID){m_GoalNode=MapManager::GetInstance().GetMapNode(_nodeID);}    
    MapNode GetGoalNode(){return m_GoalNode;}
    void ChangeState(AGVState _state){m_State=_state;}
    float GetTimeSpentOnCurrentLink_FromNode();
    float GetTimeSpendOnCurrentLink_ToNode();

    bool m_NeedReplan=false;    

    bool ComeBack2FromNodeInLink(float _deltaTime);
    bool isComeBackDone=false;
};