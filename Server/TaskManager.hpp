#pragma once
#include "Event.hpp"
#include <queue>

class TaskManager
{
public:
    static TaskManager GetInsance(){static TaskManager sIstance;return sIstance;}    
private: 
    TaskManager(){Init();}
    void Init();
    std::queue<RobotEvent> m_PendingEvents;
public:
    void ProcessNextDispatch();
    //1. 로봇이 백수가 됨 -> [새 작업 부여 후 창고로 보냄]
    void OnRobotIdle(const RobotEvent& _e);
    void OnRobotLoadCompleted(const RobotEvent& _e);
    void OnRobotUnloadCompleted(const RobotEvent& _e);    
};