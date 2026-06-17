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
    
    uint32_t curNodeID = agv->GetCurrentNode().m_Id; 

    if (!agv->GetFinalPathNodeIDs().empty()) 
    {
        if(agv->GetState()==AGVState::IDLE)
            curNodeID = agv->GetFinalPathNodeIDs()[agv->GetCurrentPathIndex()];
        else            
            curNodeID = agv->GetFinalPathNodeIDs()[agv->GetCurrentPathIndex()+1];
    }
    AstarPathFinder apf;
    std::vector<uint32_t> path = apf.FindPath(
                                        curNodeID,
                                        _targetNodeID,
                                        MapManager::GetInstance().GetNodes(),
                                        MapManager::GetInstance().GetLinks(),
                                        _agvID,_serverTime);
    
    if(!path.empty())
    {
        agv->SetNewTargetRoute(path);
        agv->SetCurrentIndex(0);
        agv->SetGoalNode(_targetNodeID);
        agv->ReserveTimeLine(path, _serverTime);
        agv->ChangeState(_nextState); // (MOVE_TO_PICKUP or MOVE_TO_DROP or MOVE_TO_HOME)
    }
    else
    {        
        std::cout << "[스케줄러] 경로 탐색 실패 AGV: " << _agvID << " ➔ Target: " << _targetNodeID << std::endl;
        MapNode curNode = agv->GetCurrentNode(); 
        uint32_t curNodeID=agv->GetFinalPathNodeIDs()[agv->GetCurrentPathIndex()];
        
        // 1. 내 발밑에 5초짜리 안전 보호막 전개
        TrafficControlManager::GetInstance().ReserveNode(curNodeID, _serverTime, _serverTime + 5.0f, _agvID);
        
        // 2. 물리 이동 강제 정지
        agv->ChangeState(AGVState::WAITING);
        
        // 3. 3초 뒤에 원래 가려던 상태(_nextState)로 재시도 큐에 쏙!
        m_PendingReplans.push_back({_agvID, 3.f, _nextState});
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

    
    uint32_t curNodeID=agv->GetFinalPathNodeIDs()[agv->GetCurrentPathIndex()];
    MapNode goalNode = agv->GetGoalNode();

    AstarPathFinder apf;
    std::vector<uint32_t> path = apf.FindPath(curNodeID, goalNode.m_Id, MapManager::GetInstance().GetNodes(), MapManager::GetInstance().GetLinks(), _agvID, _serverTime);
    
    if(!path.empty())
    {
        agv->SetNewTargetRoute(path);
        agv->ReserveTimeLine(path, _serverTime);
        agv->ChangeState(_nextState); 
    }
    else
    {                
        TrafficControlManager::GetInstance().ReserveNode(curNodeID, _serverTime, _serverTime + 5.0f, _agvID);        
        agv->ChangeState(AGVState::WAITING);
        
        m_PendingReplans.push_back({_agvID, 3.f,_nextState});
    }
}