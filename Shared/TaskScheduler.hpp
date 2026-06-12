#pragma once
#include <cstdint>
#include <queue>

class Robo;
enum class TaskType
{
    DELIVERY,  // 자재 이송 
    GO_DEPOT,  // 대기소 복귀 (일 없으면 쉬러 가기)
};

struct Task
{
    uint32_t   m_TaskID;       // 미션 고유 번호
    TaskType   m_Type;         // 미션 타입
    uint32_t   m_LoadNodeID;   // 자재를 싣는 노드 (출발지)
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
    void UpdateSchedule(float _serverTime);
    void AssignTask();
    Robo FindBestAGVforTask();
public:
    void PushTaks(Task _task){m_TaskQueue.push(_task);}      
};