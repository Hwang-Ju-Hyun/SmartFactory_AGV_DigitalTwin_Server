#pragma once
#include <cstdint>
#include <queue>
#include <vector>

class Robo;
enum class AGVState;

struct Task
{
    uint32_t   m_TaskID;       // 미션 고유 번호    
    uint32_t   m_LoadNodeID;   // 자재를 싣는 노드 (출발지)
    uint32_t   m_backHomeNodeID;//자제를 내리고 할일없어 init으로 가는 노드
    uint32_t   m_UnloadNodeID; // 자재를 내리는 노드 (목적지)
};

struct PendingReplan 
{
    uint32_t agvID;
    float retryTimer;
    AGVState state;
};

class TaskScheduler
{
private:
    TaskScheduler();
    std::vector<PendingReplan> m_PendingReplans;
public:
    static TaskScheduler& GetInstance()
    {
        static TaskScheduler instance;
        return instance;
    }
private:
    std::queue<Task> m_TaskQueue;            
public:
    void Update(float _deltaTime, float _serverTime);
    void AssignRoute(uint32_t _agvID,uint32_t _targetNodeID,uint32_t _serverTime,AGVState _nextState);
    void ReplanPath(uint32_t _agvID, float _serverTime,AGVState _nextState);
};