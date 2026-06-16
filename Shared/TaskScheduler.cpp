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
        Robo* agv = FindBestAGVforTask(currentTask,AGVState::LOADING);                

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
        uint32_t dispatchNodes[5]  = { 46,  47,  48,  49,  50};        
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
        if(agv->GetState()==_agvState && agv->GetTaskState()==TaskState::COMPLETED)
        {            
            float dist = std::sqrt(std::pow(agv->GetPosX()- loadNode.m_PosX,2)+std::pow(agv->GetPosZ()-loadNode.m_PosZ,2));        
            if(dist<mini_cost)
            {
                mini_cost=dist;
                result=agv;
            }
        }
    }

    if(result!=nullptr)
    {
        result->SetTaskState(TaskState::ASSIGNED);
        result->ChangeState(AGVState::MOVE_TO_DROP);
    }        

    return result;
}

void TaskScheduler::AssignUnLoadRoute(float _serverTime,Robo* _agv)
{
    TrafficControlManager::GetInstance().ClearAgvReservations(_agv->GetNetworkID());
    TrafficControlManager::GetInstance().ClearLinkReservations(_agv->GetNetworkID());
    uint32_t dispatchNodeID = FindBestDispatchNode(_serverTime,_agv->GetNetworkID());

    AstarPathFinder apf;    
       
    std::vector<uint32_t> path=apf.FindPath(
                                        _agv->GetCurrentNode().m_Id,
                                        dispatchNodeID,
                                        MapManager::GetInstance().GetNodes(),
                                        MapManager::GetInstance().GetLinks(),
                                        _agv->GetNetworkID(),
                                        _serverTime);       
        if (path.empty())
        {
            std::cout << "[FMS]  경고: AGV " << _agv->GetNetworkID() 
                  << "번이 " << _agv->GetCurrentNode().m_Id << "번 노드에서 최종 하역장 " << dispatchNodeID 
                  << "번으로 가는 후반전 시공간 경로 개척에 실패했습니다!" << std::endl;
            return;
        }
        _agv->SetTaskState(TaskState::ASSIGNED);
        _agv->SetNewTargetRoute(path);
        _agv->SetGoalNode(dispatchNodeID);
        _agv->ReserveTimeLine(path,_serverTime);        
}
static int as=0;
uint32_t TaskScheduler::FindBestDispatchNode(float _serverTime,uint32_t _agvID)
{
    uint32_t dispatchNodes[5]  = {  46,  47,  48,  49,  50};
    uint32_t candidateNodeID=dispatchNodes[as];
    as++;
    return  candidateNodeID;

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