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

class DispatchManager 
{
public:
    static DispatchManager& GetInstance() { static DispatchManager instance; return instance; }
    
    int FindBestLoadNode(float _serverTime, uint32_t _agvID)
    {        
        std::vector<uint32_t> loadNodes = {41, 42, 43, 44, 45}; 
        std::vector<uint32_t> validNodes;
        int n=(rand()%3)+5;
        return n;
        // 1. 재고가 남아있는(예약 가능한) 창고만 후보에 넣음
        for (uint32_t nodeID : loadNodes)
        {
            if (WarehouseManager::GetInstance().CanReserveStock(nodeID))
            {
                validNodes.push_back(nodeID);
            }
        }
        
        if (validNodes.empty())
         return -1; // 재고가 하나도 없으면 실패

        Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(_agvID));
        if (!agv) return validNodes[0];

        MapNode currentAgvNode = MapManager::GetInstance().GetNodes().at(agv->GetToNodeID());
        std::vector<NodeDistance> distances;
       
        for (uint32_t nodeID : validNodes)
        {
            MapNode targetNode = MapManager::GetInstance().GetNodes().at(nodeID);
            float dist = std::sqrt(std::pow(currentAgvNode.m_PosX - targetNode.m_PosX, 2) + 
                                   std::pow(currentAgvNode.m_PosZ - targetNode.m_PosZ, 2));
            distances.push_back({nodeID, dist});
        }

        // 거리순으로 가장 가까운 상차지부터 정렬
        std::sort(distances.begin(), distances.end());
        
        //  2. 가까운 곳부터 차례대로 "시공간 장부"에 예약이 비어있는지 확인!
        for (const auto& nd : distances)
        {
            // 앞으로 5초 동안 해당 상차지가 비어있는지 검사 (새로 만든 IsNodeAvailable 사용)
            if (TrafficManager::GetInstance().IsNodeAvailable(nd.nodeID, _serverTime, _serverTime + 5.0f, _agvID))
            {
                return nd.nodeID; // 찾았다!
            }
        }

        // 전부 예약이 꽉 차 있다면 실패
        return -1;
    }

    int FindBestDispatchNode(float _serverTime, uint32_t _agvID)
    {   
        int n=(rand()%5)+8;
        return n;
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

        //  가까운 하차지부터 차례대로 빈자리 검사
        for (const auto& nd : distances)
        {
            if (TrafficManager::GetInstance().IsNodeAvailable(nd.nodeID, _serverTime, _serverTime + 5.0f, _agvID))
            {
                return nd.nodeID;
            }
        }

        return -1; // 빈자리가 없으면 실패
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
                    // 집(Home)은 무조건 갈 수 있도록 굳이 예약 검사를 빡세게 안 해도 됨 (목적지 알박기용)
                    return r->GetHomeNode();
                }                
            }            
        }
        return -1;
    }
};