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