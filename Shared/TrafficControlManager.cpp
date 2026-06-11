#include "TrafficControlManager.hpp"
#include <iostream>
#include <algorithm>
#include "LinkingContext.hpp"

TrafficControlManager::TrafficControlManager(){}

bool TrafficControlManager::IsTimeWindowAvailable(uint32_t _nodeID, float _start, float _end, uint32_t _myAgvID)
{
    if(m_ReservationTable.empty())
        return true;
    if(m_ReservationTable.find(_nodeID)==m_ReservationTable.end())
        return true;    

    for(const NodeReservation& node : m_ReservationTable[_nodeID])
    {
        if (node.agvID == _myAgvID)
            continue; 
        
        float myExtendedStart = _start - timeBuffer;
        float myExtendedEnd   = _end + timeBuffer;
        
        if (myExtendedStart < node.endTime && myExtendedEnd > node.startTime)
        {
            return false; 
        }
    }
    
    return true;    
}

void TrafficControlManager::ReserveNode(uint32_t _nodeID, float _start, float _end, uint32_t _agvID)
{
    NodeReservation nr;
    
    nr.agvID=_agvID;
    nr.startTime=_start;
    nr.endTime=_end;    

    m_ReservationTable[_nodeID].push_back(nr);
}

void TrafficControlManager::ClearAgvReservations(uint32_t _agvID)
{
    for (auto& pair : m_ReservationTable)
    {
        auto& resList = pair.second;

        for (auto iter = resList.begin(); iter != resList.end(); )
        {
            if (iter->agvID == _agvID)
            {
                iter = resList.erase(iter);
            }
            else
            {
                ++iter;
            }
        }
    }
}



void TrafficControlManager::ValidateReservation()
{    
    m_Conflicts.clear();
    for(auto r:m_ReservationTable)
    {        
        uint32_t nodeID=r.first;
        std::vector<NodeReservation>& reservation_node=r.second;
        //시간 순 오름차순
        std::sort(reservation_node.begin(),reservation_node.end(),[](const NodeReservation& a,const NodeReservation& b){return a.startTime<b.startTime;});                

        for(int i=0;i<reservation_node.size();i++)
        {            
            for(int j=i+1;j<reservation_node.size();j++)
            {
                if(Overlap(reservation_node[i],reservation_node[j]))
                {         
                    Conflict conflict;

                    conflict.NodeID=nodeID;
                    conflict.A_agvID=reservation_node[i].agvID;
                    conflict.B_agvID=reservation_node[j].agvID;
                    m_Conflicts.push_back(conflict);
                }
            }
        }
    }        
}


uint32_t TrafficControlManager::GetLoserAGVofConflict(const Conflict& _conflict)
{
    //todo : 지금은 우선순위 id값 순서대로 임
    uint32_t loserAGV=_conflict.A_agvID<=_conflict.B_agvID?_conflict.A_agvID:_conflict.B_agvID;
    return loserAGV;
}

void TrafficControlManager::ReleaseNodeReservation(uint32_t _prevNodeID,uint32_t _agvID)
{    
    std::vector<NodeReservation>& reservations = m_ReservationTable.find(_prevNodeID)->second;
    for(auto iter = reservations.begin(); iter != reservations.end();)
    {
        if(iter->agvID == _agvID)
        {
            iter = reservations.erase(iter);
        }
        else
        {
            ++iter;
        }
    }    
}

void TrafficControlManager::ReserveLink(uint32_t _from, uint32_t _to, float _start, float _end, uint32_t _agvID)
{
    LinkReservation lr;
    lr.agvID    = _agvID;  lr.fromNodeID = _from;
    lr.toNodeID = _to;     lr.startTime  = _start;
    lr.endTime  = _end;
    
    m_LinkReservations.push_back(lr);
}

bool TrafficControlManager::IsLinkAvailable(uint32_t _from, uint32_t _to, float _start, float _end, uint32_t _agvID)
{
    if(_from==6&&_to==17&&_agvID==6)
    {
        int a=0;
    }
    for(const auto& edge:m_LinkReservations)
    {
        // [정면충돌 저격]        
        if(edge.fromNodeID == _to && edge.toNodeID == _from)
        {
            float safeEdgeEndTime = edge.endTime + timeBuffer;
            
            if (_start < safeEdgeEndTime && _end > edge.startTime)
            {
                return false;    
            }            
        }            
    }
    return true;
}

void TrafficControlManager::ClearLinkReservations(uint32_t _agvID)
{
    for(auto iter=m_LinkReservations.begin();iter!=m_LinkReservations.end();)
    {
        if(iter->agvID==_agvID)
        {
            iter = m_LinkReservations.erase(iter);
        }
        else
        {
            iter++;
        }
    }
}