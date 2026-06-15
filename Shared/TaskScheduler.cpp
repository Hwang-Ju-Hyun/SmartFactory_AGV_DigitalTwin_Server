#include "TaskScheduler.hpp"
#include "PathFinder.hpp"
#include "Robo.hpp"
#include "LinkingContext.hpp"
#include "AGVManager.hpp"
#include "Object.hpp"

TaskScheduler::TaskScheduler()
{

}

void TaskScheduler::UpdateSchedule(float _serverTime)
{    
    //UpdateLoadSchedule(_serverTime);
    UpdateUnLoadSchedule(_serverTime);
}

void TaskScheduler::UpdateUnLoadSchedule(float _serverTime)
{
    return;
    while(!m_TaskQueue.empty())
    {
        Task currentTask = m_TaskQueue.front();        
        Robo* agv = FindBestAGVforTask(currentTask,AGVState::MOVE_TO_DROP);                

        if(agv==nullptr)
            break;
        
        AstarPathFinder apf;    
        
        uint32_t curNodeIDAgv= agv->GetCurrentNode().m_Id;
        
        std::vector<uint32_t> path=apf.FindPath(
                                        curNodeIDAgv,
                                        currentTask.m_UnloadNodeID,
                                        MapManager::GetInstance().GetNodes(),
                                        MapManager::GetInstance().GetLinks(),
                                        agv->GetNetworkID(),
                                        _serverTime);
        if(agv->GetNetworkID()!=4)
            int a=0;
        agv->AssignTask(currentTask);
        agv->SetTaskState(TaskState::ASSIGNED);
        agv->SetNewTargetRoute(path);
        agv->SetGoalNode(currentTask.m_UnloadNodeID);
        agv->ReserveTimeLine(path,_serverTime);        

        m_TaskQueue.pop();
    }        
}

void TaskScheduler::UpdateLoadSchedule(float _serverTime)
{
    while(!m_TaskQueue.empty())
    {
        Task currentTask = m_TaskQueue.front();        
        Robo* agv = FindBestAGVforTask(currentTask,AGVState::MOVE_TO_PICKUP);
        
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
        agv->SetTaskState(TaskState::ASSIGNED);
        agv->SetNewTargetRoute(path);
        agv->SetGoalNode(currentTask.m_LoadNodeID);
        agv->ReserveTimeLine(path,_serverTime);
        agv->ChangeState(AGVState::MOVE_TO_PICKUP);

        m_TaskQueue.pop();
    }      
}

uint32_t TaskScheduler::FindBestNode(Task _task,AGVState _agvCurState,float _serverTime,uint32_t _agvID)
{
    if(_agvCurState==AGVState::MOVE_TO_DROP)
    {
        uint32_t dispatchNodes[5]  = { 46,  47,  48,  35,  36};        
        uint32_t bestNodeID=dispatchNodes[0];

        for(int i=0;i<5;i++)
        {
            uint32_t candidateNodeID=dispatchNodes[i];
            bool avaliable=TrafficControlManager::GetInstance().IsTimeWindowAvailable(candidateNodeID,_serverTime,_serverTime+3.f,_agvID);
            
            if(avaliable)
            {
                bestNodeID=candidateNodeID;
            }
            return bestNodeID;
        }        
    }
    return -1;
}

Robo* TaskScheduler::FindBestAGVforTask(Task _currentTask,AGVState _agvState)
{            
    std::vector<ObjectPtr> agvs = AGVManager::GetInstance().m_AGVs;    
    
    MapNode loadNode = MapManager::GetInstance().GetMapNode(_currentTask.m_UnloadNodeID);
    
    Robo* result=nullptr;
    float mini_cost=1e9;
    
    for(int i=0;i<agvs.size();i++)
    {
        Robo* agv =dynamic_cast<Robo*>(agvs[i].get());
        if(agv->GetState()!=_agvState)
        {
            continue;
        }        

        float dist = std::sqrt(std::pow(agv->GetPosX()- loadNode.m_PosX,2)+std::pow(agv->GetPosZ()-loadNode.m_PosZ,2));        
        if(dist<mini_cost)
        {
            mini_cost=dist;
            result=agv;
        }
    }
    return result;
}

void TaskScheduler::AssignUnLoadRoute(float _serverTime,Robo* _agv)
{
    uint32_t dispatchNodeID = FindBestDispatchNode(_serverTime,_agv->GetNetworkID());
    
    AstarPathFinder apf;    
       
    std::vector<uint32_t> path=apf.FindPath(
                                        _agv->GetCurrentNode().m_Id,
                                        dispatchNodeID,
                                        MapManager::GetInstance().GetNodes(),
                                        MapManager::GetInstance().GetLinks(),
                                        _agv->GetNetworkID(),
                                        _serverTime);       

        _agv->SetTaskState(TaskState::ASSIGNED);
        _agv->SetNewTargetRoute(path);
        _agv->SetGoalNode(dispatchNodeID);
        _agv->ReserveTimeLine(path,_serverTime);        
}

uint32_t TaskScheduler::FindBestDispatchNode(float _serverTime,uint32_t _agvID)
{
    uint32_t dispatchNodes[5]  = {  46,  47,  48,  35,  36};
    uint32_t candidateNodeID=dispatchNodes[0];
    bool avaliable=false;
    for(int i=0;i<5;i++)
    { 
        candidateNodeID=dispatchNodes[i];
        if(TrafficControlManager::GetInstance().IsTimeWindowAvailable(candidateNodeID,_serverTime,_serverTime+3.f,_agvID))
        {
            return candidateNodeID;
        }
    }      
    return -1;
}