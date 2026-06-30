#pragma once
#include <cstdint>
#include <vector>
#include <algorithm>
#include <cmath>
#include "ReservationTable.hpp" 
#include "AGVManager.hpp"
#include "Robo.hpp"
#include "WarehouseManager.hpp"

//#define _TESTCASE2
#define _TESTCASE3

struct NodeDistance 
{
    uint32_t nodeID;
    float distance;
    
    bool operator<(const NodeDistance& other) const 
    {
        return distance < other.distance;
    }
};

std::vector<uint32_t> loadNodes;
std::vector<uint32_t> dispatchNodes;

class DispatchManager 
{
public:
    DispatchManager() 
    {
        #ifdef _TESTCASE2
        for(int i=56; i<=61; i++) { loadNodes.push_back(i); }
        for(int i=4; i<=9; i++) { loadNodes.push_back(i); }
        loadNodes.push_back(13); loadNodes.push_back(30); loadNodes.push_back(47);
        
        dispatchNodes.push_back(64); dispatchNodes.push_back(63); dispatchNodes.push_back(55);
        dispatchNodes.push_back(46); dispatchNodes.push_back(29); dispatchNodes.push_back(12);
        dispatchNodes.push_back(3); dispatchNodes.push_back(2);
        
        #elifdef _TESTCASE3
        loadNodes.push_back(12);
        loadNodes.push_back(11);
        loadNodes.push_back(10);

        dispatchNodes.push_back(3);
        dispatchNodes.push_back(4);
        dispatchNodes.push_back(6);
        #endif
    }

    static DispatchManager& GetInstance() 
    { 
        static DispatchManager instance;                 
        return instance; 
    }        

    int FindBestLoadNode(float _serverTime, uint32_t _agvID)
    {                
        std::vector<uint32_t> availableNodes;
        
        for (uint32_t nodeID : loadNodes)
        {            
            if (ReservationTable::GetInstance().IsNodeFree(nodeID, _serverTime, _serverTime + 1.0f, _agvID))
            {
                availableNodes.push_back(nodeID);
            }
        }
        
        if (availableNodes.empty()) return -1; 

        int randomIndex = rand() % availableNodes.size();
        return availableNodes[randomIndex];
    }

    int FindBestDispatchNode(float _serverTime, uint32_t _agvID)
    {            
        std::vector<uint32_t> availableNodes;
        
        for (uint32_t nodeID : dispatchNodes)
        {
            
            if (ReservationTable::GetInstance().IsNodeFree(nodeID, _serverTime, _serverTime + 1.0f, _agvID))
            {
                availableNodes.push_back(nodeID);
            }
        }

        if (availableNodes.empty()) return -1;

        int randomIndex = rand() % availableNodes.size();
        return availableNodes[randomIndex];
    }

    uint32_t FindHomeNode(float _serverTime, uint32_t _agvID)
    {
       for (auto agv : AGVManager::GetInstance().m_AGVs) 
        {
            if (agv->GetNetworkID() == _agvID)
            {                
                Robo* r = dynamic_cast<Robo*>(agv.get());
                if (r) return r->GetHomeNode();                
            }            
        }
        return -1;
    }
};