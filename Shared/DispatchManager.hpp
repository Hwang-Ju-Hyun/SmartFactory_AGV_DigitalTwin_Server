#pragma once
#include <cstdint>
#include "TrafficControlManager.hpp"
#include "AGVManager.hpp"
#include "Robo.hpp"
#include "WarehouseManager.hpp"
#include <algorithm>

#include <cstdlib>
#include <cmath>

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
    static DispatchManager& GetInstance() { static DispatchManager instance;return instance; }
    
    int FindBestLoadNode(float _serverTime, uint32_t _agvID)
    {        
        std::vector<uint32_t> loadNodes = {41, 42, 43, 44, 45}; 
        std::vector<uint32_t> validNodes;
        //todo :random 테스트 이거 지우셈
        //int n = loadNodes[rand()%200];
        int n=rand()%454+5;
        if (TrafficControlManager::GetInstance().IsTimeWindowAvailable(n, _serverTime, _serverTime + 5.0f, _agvID))
        {
            return n;
        }        
        return -1;


        // 1.재고가 남아있는(예약 가능한) 창고만 후보에 넣음
        for (uint32_t nodeID : loadNodes)
        {
            if (WarehouseManager::GetInstance().CanReserveStock(nodeID))
            {
                validNodes.push_back(nodeID);
            }
        }
        
        if (validNodes.empty()) 
            return 0; 

        Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(_agvID));
        if (!agv) 
            return validNodes[0];

        MapNode currentAgvNode = MapManager::GetInstance().GetNodes().at(agv->GetToNodeID());
        std::vector<NodeDistance> distances;
       
        for (uint32_t nodeID : validNodes)
        {
            MapNode targetNode = MapManager::GetInstance().GetNodes().at(nodeID);
            float dist = std::sqrt(std::pow(currentAgvNode.m_PosX - targetNode.m_PosX, 2) + 
                                std::pow(currentAgvNode.m_PosZ - targetNode.m_PosZ, 2));
            distances.push_back({nodeID, dist});
        }

        std::sort(distances.begin(), distances.end());

        
        for (const auto& nd : distances)
        {
            if (TrafficControlManager::GetInstance().IsTimeWindowAvailable(nd.nodeID, _serverTime, _serverTime + 5.0f, _agvID))
            {
                return nd.nodeID;
            }
        }

        return distances[0].nodeID;
    }

    // 2. "어디다 물건 내려놓을까?" (하역지 결정)
    int FindBestDispatchNode(float _serverTime, uint32_t _agvID)
    {                
        std::vector<uint32_t> dispatchNodes = {46, 47, 48, 49, 50}; 
        int n=(rand()%5)+1;

        if (TrafficControlManager::GetInstance().IsTimeWindowAvailable(n, _serverTime, _serverTime + 5.0f, _agvID))
        {
            return n;
        }
        return -1;


        Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(_agvID));
        if (!agv) 
            return dispatchNodes[0];

        MapNode currentAgvNode = MapManager::GetInstance().GetNodes().at(agv->GetToNodeID());
        std::vector<NodeDistance> distances;

        for (uint32_t nodeID : dispatchNodes)
        {
            MapNode targetNode = MapManager::GetInstance().GetNodes().at(nodeID);
            float dist = std::sqrt(std::pow(currentAgvNode.m_PosX - targetNode.m_PosX, 2) + 
                                std::pow(currentAgvNode.m_PosZ - targetNode.m_PosZ, 2));
            distances.push_back({nodeID, dist});
        }

        std::sort(distances.begin(), distances.end());

        for (const auto& nd : distances)
        {
            if (TrafficControlManager::GetInstance().IsTimeWindowAvailable(nd.nodeID, _serverTime, _serverTime + 5.0f, _agvID))
            {
                return nd.nodeID;
            }
        }

        return distances[0].nodeID;
    }

    uint32_t FindHomeNode(float _serverTime,uint32_t _agvID)
    {
        for(auto agv : AGVManager::GetInstance().m_AGVs) 
        {
            if(agv->GetNetworkID()==_agvID)
            {                
                Robo* r = dynamic_cast<Robo*>(agv.get());
                if(r)
                {
                    if(TrafficControlManager::GetInstance().IsTimeWindowAvailable(r->GetHomeNode(), _serverTime, _serverTime + 3.f, _agvID))
                        return r->GetHomeNode();
                }                
            }            
        }
        return -1;
    }
};