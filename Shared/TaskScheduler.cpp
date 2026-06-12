#include "TaskScheduler.hpp"
#include "PathFinder.hpp"
#include "NetworkManagerServer.hpp"
#include "Robo.hpp"
#include "LinkingContext.hpp"
#include "AGVManager.hpp"
#include "Object.hpp"

TaskScheduler::TaskScheduler()
{

}

void TaskScheduler::UpdateSchedule(float _serverTime)
{
    if(m_TaskQueue.empty())
        return;
    while(!m_TaskQueue.empty())
    {
        Task currentTask = m_TaskQueue.front();        
        Robo* agv = FindBestAGVforTask(currentTask);
        if(agv==nullptr)    
            break;
        AstarPathFinder apf;    
        uint32_t curNodeIDAgv= agv->GetCurrentNode().m_Id;
        std::vector<uint32_t> path=apf.FindPath(
                                        curNodeIDAgv,
                                        currentTask.m_LoadNodeID,
                                        MapManager::GetInstance().GetNodes(),
                                        MapManager::GetInstance().GetLinks(),
                                        agv->GetNetworkID(),
                                        _serverTime);
        
        agv->AssignTask(currentTask);
        agv->SetTaskProgressState(TaskProgressState::GOTO_LOAD);
        agv->SetNewTargetRoute(path);
        agv->SetGoalNode(currentTask.m_LoadNodeID);
        agv->ReserveTimeLine(path,_serverTime);
        agv->ChangeState(AGVState::MOVING);

        m_TaskQueue.pop();
    }    
    
}

void TaskScheduler::AssignTask()
{

}

Robo* TaskScheduler::FindBestAGVforTask(Task _currentTask)
{            
    std::vector<ObjectPtr> agvs=AGVManager::GetInstance().m_AGVs;    
    MapNode loadNode = MapManager::GetInstance().GetMapNode(_currentTask.m_LoadNodeID);
    Robo* result=nullptr;
    for(int i=0;i<agvs.size();i++)
    {
        Robo* agv =dynamic_cast<Robo*>(agvs[i].get());
        if(agv->GetTaskProgressState()!=TaskProgressState::NONE)
        {
            continue;
        }
        float dist = std::sqrt(std::pow(agv->GetPosX()- loadNode.m_PosX,2)+std::pow(agv->GetPosZ()-loadNode.m_PosZ,2));

        float mini_cost=1e9;
        if(dist<mini_cost)
        {
            mini_cost=dist;
            result=agv;
        }
    }

    return result;

}