#include "TrafficControlManager.hpp"
#include <iostream>

TrafficControlManager::TrafficControlManager(){}

bool TrafficControlManager::IsTimeWindowAvailable(uint32_t _nodeID, float _start, float _end, uint32_t _myAgvID)
{
    if(m_ReservationTable.empty())
        return true;
    if(m_ReservationTable.find(_nodeID)==m_ReservationTable.end())
        return true;

    const float timeBuffer = 0.5f;

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

void TrafficControlManager::RegisterGoal(uint32_t _nodeID,uint32_t _agvID)
{
    NodeReservation nr;
    nr.agvID=_agvID;
    nr.endTime=9999999.f;    
    m_ReservationTable[_nodeID].push_back(nr);
}