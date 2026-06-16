#include "TaskScheduler.hpp"
#include "PathFinder.hpp"
#include "Robo.hpp"
#include "LinkingContext.hpp"
#include "AGVManager.hpp"
#include "Object.hpp"

TaskScheduler::TaskScheduler()
{

}

void TaskScheduler::AssignRoute(uint32_t _agvID,uint32_t _targetNodeID,uint32_t _serverTime,AGVState _nextState)
{    
    std::vector<ObjectPtr> objs = AGVManager::GetInstance().m_AGVs;
    Robo* agv=nullptr;    

    for(int i=0;i<objs.size();i++)
    {
        if(objs[i]->GetNetworkID()==_agvID)
        {
            agv=dynamic_cast<Robo*>(objs[i].get());            
            break;
        }           
    }
    
    if(agv==nullptr)
        return;

    TrafficControlManager::GetInstance().ClearAgvReservations(_agvID);

    AstarPathFinder apf;
    std::vector<uint32_t> path = apf.FindPath(
                                        agv->GetCurrentNode().m_Id,
                                        _targetNodeID,
                                        MapManager::GetInstance().GetNodes(),
                                        MapManager::GetInstance().GetLinks(),
                                        _agvID,_serverTime);
    
    if(!path.empty())
    {
        agv->SetNewTargetRoute(path);
        agv->SetGoalNode(_targetNodeID);
        agv->ReserveTimeLine(path, _serverTime);
        agv->ChangeState(_nextState); // (MOVE_TO_PICKUP or MOVE_TO_DROP or MOVE_TO_HOME)
    }
    else
    {
        std::cout << "[스케줄러] 경로 탐색 실패 AGV: " << _agvID << " ➔ Target: " << _targetNodeID << std::endl;
    }
}

void TaskScheduler::Update(float _deltaTime, float _serverTime)
{
    for(auto iter=m_PendingReplans.begin(); iter< m_PendingReplans.end();)
    {
        iter->retryTimer-=_deltaTime;

        if(iter->retryTimer<=0.f)
        {
            uint32_t retryAgvID = iter->agvID;
            iter = m_PendingReplans.erase(iter);
            Robo* agv =AGVManager::GetInstance().FindAGV(retryAgvID);
            ReplanPath(retryAgvID,_serverTime,iter->state);
        }
        else
        {
            iter++;
        }
    }
}

void TaskScheduler::ReplanPath(uint32_t _agvID, float _serverTime,AGVState _nextState)
{
    TrafficControlManager::GetInstance().ClearAgvReservations(_agvID);
    Robo* agv = AGVManager::GetInstance().FindAGV(_agvID);    

    MapNode curNode = agv->GetCurrentNode();
    MapNode goalNode = agv->GetGoalNode();

    AstarPathFinder apf;
    std::vector<uint32_t> path = apf.FindPath(curNode.m_Id, goalNode.m_Id, MapManager::GetInstance().GetNodes(), MapManager::GetInstance().GetLinks(), _agvID, _serverTime);
    
    if(!path.empty())
    {
        agv->SetNewTargetRoute(path);
        agv->ReserveTimeLine(path, _serverTime);
        agv->ChangeState(_nextState); 
    }
    else
    {                
        TrafficControlManager::GetInstance().ReserveNode(curNode.m_Id, _serverTime, _serverTime + 5.0f, _agvID);        
        agv->ChangeState(AGVState::WAITING);
        
        m_PendingReplans.push_back({_agvID, 1.5f,_nextState});
    }
}