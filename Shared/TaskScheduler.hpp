#pragma once
#include <cstdint>
#include <queue>

class Robo;
enum class AGVState;

struct Task
{
    uint32_t   m_TaskID;       // 미션 고유 번호    
    uint32_t   m_LoadNodeID;   // 자재를 싣는 노드 (출발지)
    uint32_t   m_backHomeNodeID;//자제를 내리고 할일없어 init으로 가는 노드
    uint32_t   m_UnloadNodeID; // 자재를 내리는 노드 (목적지)
};

class TaskScheduler
{
private:
    TaskScheduler();
public:
    static TaskScheduler& GetInstance()
    {
        static TaskScheduler instance;
        return instance;
    }
private:
    std::queue<Task> m_TaskQueue;    
    Robo* FindBestAGVforTask(Task _currentTask,AGVState _agvState);
public:
    void AssignTask(Robo* _robo);
    void UpdateSchedule(float _serverTime);
    void UpdateLoadSchedule(float _serverTime);
    void UpdateUnLoadSchedule(float _serverTime);
    void PushTaks(Task _task){m_TaskQueue.push(_task);}      
    uint32_t FindBestNode(Task _task,AGVState _agvCurState,float _serverTime,uint32_t _agvID);


public:
    void AssignUnLoadRoute(float _serverTime,Robo* _agv);
    uint32_t FindBestDispatchNode(float _serverTime,uint32_t _agvID);
};