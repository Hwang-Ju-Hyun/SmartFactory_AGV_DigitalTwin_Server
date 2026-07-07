#pragma once
#include <cstdint>
#include <vector>
#include <algorithm>
#include <cmath>
#include "ReservationTable.hpp" 
#include "AGVManager.hpp"
#include "Robo.hpp"
#include "WarehouseManager.hpp"

//#define _TESTCASE0
//#define _TESTCASE1
//#define _TESTCASE2
#define _TESTCASE3
//#define _TESTCASE4
//#define _TESTCASE5

struct NodeDistance 
{
    uint32_t nodeID;
    float distance;
    
    bool operator<(const NodeDistance& other) const 
    {
        return distance < other.distance;
    }
};

class DispatchManager 
{
public:
    std::vector<uint32_t> loadNodes;
    std::vector<uint32_t> dispatchNodes;

#ifdef _TESTCASE5    
    std::vector<uint32_t> groupA = {15,19,18,17,16};
    std::vector<uint32_t> groupB = {8,7,6,5,4};
#endif

    DispatchManager() 
    {
        #ifdef _TESTCASE0
        loadNodes.push_back(8);
        loadNodes.push_back(9);
        loadNodes.push_back(10);
        loadNodes.push_back(11);
        loadNodes.push_back(12);

        dispatchNodes.push_back(5);
        dispatchNodes.push_back(6);
        dispatchNodes.push_back(7);
        #elifdef _TESTCASE1
        for(int i=5; i<=460; i++) { loadNodes.push_back(i); }
        for(int i=1; i<=4; i++) { dispatchNodes.push_back(i); }

        #elifdef _TESTCASE2
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

        dispatchNodes.push_back(29);
        dispatchNodes.push_back(25);
        dispatchNodes.push_back(23);
        
        #elifdef _TESTCASE4
        loadNodes.push_back(5);
        loadNodes.push_back(3);
        dispatchNodes.push_back(4);
        dispatchNodes.push_back(6);

        #elifdef _TESTCASE5
        // _TESTCASE4 에서는 전역 loadNodes를 쓰지 않고 아래 함수에서 그룹을 교차시킵니다.
        #endif
    }

    static DispatchManager& GetInstance() 
    { 
        static DispatchManager instance;                
        return instance; 
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

    int FindBestLoadNode(float _serverTime, uint32_t _agvID)
    {                
        std::vector<uint32_t> availableNodes;
        
#ifdef _TESTCASE5        
        Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(_agvID));
        if (!agv) return -1;

        uint32_t currentLoc = agv->GetCurrentNodeID();
        uint32_t homeNode = FindHomeNode(_serverTime, _agvID);
        bool isHomeA = (std::find(groupA.begin(), groupA.end(), homeNode) != groupA.end());
        
        // 내 원래 고향이 A면 목표는 B, B면 A
        const std::vector<uint32_t>& targetGroup = isHomeA ? groupB : groupA;
        if (std::find(targetGroup.begin(), targetGroup.end(), currentLoc) != targetGroup.end())
        {
            return -1; 
        }

        // 아직 고향이라면 반대편 진영의 빈자리를 찾아서 출발합니다.
        for (uint32_t nodeID : targetGroup)
        {
            if (ReservationTable::GetInstance().IsNodeFree(nodeID, _serverTime, _serverTime + 1.0f, _agvID))
            {
                availableNodes.push_back(nodeID);
            }
        }
#else
        // 기존 로직
        for (uint32_t nodeID : loadNodes)
        {            
            if (ReservationTable::GetInstance().IsNodeFree(nodeID, _serverTime, _serverTime + 1.0f, _agvID))
            {
                availableNodes.push_back(nodeID);
            }
        }
#endif
        
        if (availableNodes.empty()) return -1; 

        int randomIndex = rand() % availableNodes.size();
        return availableNodes[randomIndex];
    }

    int FindBestDispatchNode(float _serverTime, uint32_t _agvID)
    {            
        std::vector<uint32_t> availableNodes;
        
#ifdef _TESTCASE5        
       Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(_agvID));
        if (agv) return agv->GetCurrentNodeID(); 

        return -1;
#else
        
        for (uint32_t nodeID : dispatchNodes)
        {
            if (ReservationTable::GetInstance().IsNodeFree(nodeID, _serverTime, _serverTime + 1.0f, _agvID))
            {
                availableNodes.push_back(nodeID);
            }
        }
#endif

        if (availableNodes.empty()) return -1;

        int randomIndex = rand() % availableNodes.size();
        return availableNodes[randomIndex];
    }
};