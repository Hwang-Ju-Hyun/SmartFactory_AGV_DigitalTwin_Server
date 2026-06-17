#pragma once
#include <cstdint>
#include "TrafficControlManager.hpp"
#include "AGVManager.hpp"
#include "Robo.hpp"
#include <algorithm>

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
    static DispatchManager& GetInstance() { static DispatchManager instance; return instance; }
    
    uint32_t FindBestLoadNode(float _serverTime, uint32_t _agvID)
    {        
        std::vector<uint32_t> loadNodes = {41, 42, 43, 44, 45}; 
        
        Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(_agvID));

        if (!agv) 
            return loadNodes[0]; 

        MapNode currentAgvNode = MapManager::GetInstance().GetNodes().at(agv->GetToNodeID());
        
        std::vector<NodeDistance> distances;
        
        for (uint32_t nodeID : loadNodes)
        {
            MapNode targetNode = MapManager::GetInstance().GetNodes().at(nodeID);
            
            float dist = std::sqrt(std::pow(currentAgvNode.m_PosX - targetNode.m_PosX, 2) + 
                                std::pow(currentAgvNode.m_PosZ - targetNode.m_PosZ, 2));

            distances.push_back({nodeID, dist});
        }

        // 2. 거리순으로 오름차순 정렬 (가까운 순)
        std::sort(distances.begin(), distances.end());

        // 3. 가장 가까운 곳부터 '예약 가능한지(비어있는지)' 확인
        for (const auto& nd : distances)
        {
         
            if (TrafficControlManager::GetInstance().IsTimeWindowAvailable(nd.nodeID, _serverTime, _serverTime + 10.0f, _agvID))
            {
                return nd.nodeID;
            }
        }

        // 모든 곳이 다 예약되어 있다면? 어쩔 수 없이 제일 가까운 곳 반환 (스케줄러가 알아서 대기시킬 것임)
        return distances[0].nodeID;
    }

    // 2. "어디다 물건 내려놓을까?" (하역지 결정)
    uint32_t FindBestDispatchNode(float _serverTime, uint32_t _agvID)
    {        
        
        std::vector<uint32_t> dispatchNodes = {46, 47, 48, 49, 50}; 
                
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