#pragma once
#include <cstdint>
#include <vector>
#include <algorithm>
#include <cmath>
#include "TrafficControlManager.hpp" //새로 만든 시공간 예약 장부!
#include "AGVManager.hpp"
#include "Robo.hpp"
#include "WarehouseManager.hpp"

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
        for(int i=56;i<=61;i++)
        {
            loadNodes.push_back(i);
        }
        for(int i=4;i<=9;i++)
        {
            loadNodes.push_back(i);
        }
        loadNodes.push_back(13);loadNodes.push_back(30);loadNodes.push_back(47);
        
        dispatchNodes.push_back(64);
        dispatchNodes.push_back(63);
        dispatchNodes.push_back(55);
        dispatchNodes.push_back(46);
        dispatchNodes.push_back(29);
        dispatchNodes.push_back(12);
        dispatchNodes.push_back(3);
        dispatchNodes.push_back(2);
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
            if (TrafficManager::GetInstance().IsNodeAvailable(nodeID, _serverTime, _serverTime + 1.0f, _agvID))
            {
                availableNodes.push_back(nodeID);
            }
        }
        
        // 2. 안전한 방이 하나도 없으면 대기 (-1 반환)
        if (availableNodes.empty())
         return -1; 

        // 3. 안전한 방들 중에서 '랜덤'으로 하나를 뽑아서 배정!
        int randomIndex = rand() % availableNodes.size();
        return availableNodes[randomIndex];
    }

    int FindBestDispatchNode(float _serverTime, uint32_t _agvID)
    {           
        std::vector<uint32_t> availableNodes;
        
        for (uint32_t nodeID : dispatchNodes)
        {
            if (TrafficManager::GetInstance().IsNodeAvailable(nodeID, _serverTime, _serverTime + 1.0f, _agvID))
            {
                availableNodes.push_back(nodeID);
            }
        }

        // 2. 안전한 방이 하나도 없으면 대기
        if (availableNodes.empty())
            return -1;

        // 3. 안전한 하차지 중에서 '랜덤'으로 하나를 뽑아서 배정!
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
                if (r)
                {
                    return r->GetHomeNode();
                }                
            }            
        }
        return -1;
    }
};