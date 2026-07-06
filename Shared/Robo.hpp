#pragma once
#include "Object.hpp"
#include <vector>
#include "Map.hpp"
#include "PathFinder.hpp"

enum class AGVState
{
    IDLE,       
    MOVING,     
    LOADING,    
    UNLOADING,
    WAIT_REPLAN
};

enum class MissionPurpose
{
    NONE,
    HOME,
    PICKUP,
    DROP
};

const float WaitTime = 1.f;

class Robo : public Object
{       
public:
    Robo();
    virtual ~Robo() override {}    
    virtual uint32_t GetClassID() override { return m_ClassID; }

    float m_MoveStartTime = -1.0f;
    float m_PlannedTravelTime = 0.0f;
private:
    uint32_t m_CurrentNodeID;
    MapNode m_HomeNode;
    MapNode m_FromNode;
    MapNode m_ToNode;    
    float m_Speed;
    float m_Progress;        
    float m_StayTime = 1.f;
    float m_AccStayTime = 0.f;
    float m_AccWaitTime = 0.f;
    bool IsStayTime = false;

    float m_WorkTimer = 0.0f; // 상하차 작업 남은 시간
    AGVState m_State;
    MissionPurpose m_Purpose; 
public:
    void SetCurrentNodeID(uint32_t _nodeID) { m_CurrentNodeID = _nodeID; }
    uint32_t GetCurrentNodeID() const { return m_CurrentNodeID; }
    
    float GetSpeed() const { return m_Speed; }
    float GetWaitTime() const { return WaitTime; }

    void SetHomeNode(uint32_t _node) { m_HomeNode = MapManager::GetInstance().GetMapNode(_node); }
    uint32_t GetHomeNode() const { return m_HomeNode.m_Id; }

    void StartWorkTimer(float time) { m_WorkTimer = time; }
    void UpdateWorkTimer(float dt, float currentServerTime);
    MapNode m_GoalNode;
    std::vector<uint32_t> m_FinalPathNodeIDs;     
    size_t m_CurrentPathIndex = 0;       
    std::vector<uint32_t> GetFinalPathNodeIDs() { return m_FinalPathNodeIDs; }
    bool m_bIdleEventSent = false;

public:    
    void SetNewTargetRoute(const std::vector<uint32_t>& _newPath) 
    {
        m_FinalPathNodeIDs = _newPath;
        m_CurrentPathIndex = 0;
        m_Progress = 0.0f;             
    }        
    void SetCurrentIndex(uint32_t _idx) { m_CurrentPathIndex = _idx; }
    size_t GetCurrentPathIndex() { return m_CurrentPathIndex; }
    
    void SetGoalNode(uint32_t _nodeID) { m_GoalNode = MapManager::GetInstance().GetMapNode(_nodeID); }    
    MapNode GetGoalNode() { return m_GoalNode; }
    
    void ChangeState(AGVState _state)
    {
        m_State = _state;
        if (_state == AGVState::IDLE) 
            m_bIdleEventSent = false;
    }

    float GetTimeSpentOnCurrentLink_FromNode();
    float GetTimeSpendOnCurrentLink_ToNode();    
    
    bool isComeBackDone = false;
    MapLink m_CurrentLink;

public:
    void AssignNextStep(const MapNode& _from, const MapNode& _to, AGVState _newState, float _startTime, float _arrivalTime);
    
    void SetMissionPurpose(MissionPurpose _purpose) { m_Purpose = _purpose; }
    void UpdateNavigation(float _deltaTime, float _serverTime);

    uint32_t GetFromNodeID() const { return m_FromNode.m_Id; }
    uint32_t GetToNodeID() const { return m_ToNode.m_Id; }
    AGVState GetState() const { return m_State; }        
    MissionPurpose GetMissionPurpose() const { return m_Purpose; }
};