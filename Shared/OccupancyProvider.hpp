#pragma once
#include <unordered_map>
#include <cstdint>
#include <iostream>
#include <string>
#include <cassert>

class OccupancyProvider
{
private:
    std::unordered_map<uint32_t, uint32_t> m_NodeOccupancy; // NodeID -> AGV ID
    std::unordered_map<uint64_t, uint32_t> m_EdgeOccupancy; // EdgeKey -> AGV ID
    std::unordered_map<uint32_t, uint32_t> m_AgvCurrentNode;
    std::unordered_map<uint32_t, uint64_t> m_AgvCurrentEdge; 

    OccupancyProvider() = default;

public:
    static OccupancyProvider& GetInstance() { static OccupancyProvider instance; return instance; }

    bool IsNodeOccupiedByOther(uint32_t nodeID, uint32_t myAgvID) const {
        return (m_NodeOccupancy.count(nodeID) && m_NodeOccupancy.at(nodeID) != myAgvID);
    }

    bool IsEdgeOccupiedByOther(uint64_t edgeKey, uint32_t myAgvID) const {
        return (m_EdgeOccupancy.count(edgeKey) && m_EdgeOccupancy.at(edgeKey) != myAgvID);
    }

    uint32_t GetNodeOccupant(uint32_t nodeID) const { return m_NodeOccupancy.at(nodeID); }
    uint32_t GetEdgeOccupant(uint64_t edgeKey) const { return m_EdgeOccupancy.at(edgeKey); }

    void SetEdgeOccupancy(uint64_t edgeKey, uint32_t agvID) 
    {
        m_EdgeOccupancy[edgeKey] = agvID;
        m_AgvCurrentEdge[agvID] = edgeKey;
    }

    void OccupyNode(uint32_t agvID, uint32_t nodeID) 
    {
        
        if (m_NodeOccupancy.count(nodeID) && m_NodeOccupancy[nodeID] != agvID) {
            std::cerr << "\n[CRITICAL DEVIATION] AGV " << agvID << " 가 AGV " << m_NodeOccupancy[nodeID] 
                      << " 가 선점 중인 노드 " << nodeID << " 를 침범했습니다!\n" << std::endl;
            assert(false && "Occupancy Collision Detected!");
        }

        auto edgeIt = m_AgvCurrentEdge.find(agvID);
        if (edgeIt != m_AgvCurrentEdge.end()) 
        {
            m_EdgeOccupancy.erase(edgeIt->second);
            m_AgvCurrentEdge.erase(edgeIt);
        }
        
        m_NodeOccupancy[nodeID] = agvID;
        m_AgvCurrentNode[agvID] = nodeID;
        
        PrintState(agvID);
    }
    
    void LeaveNode(uint32_t agvID, uint32_t nodeID) 
    {
        auto nodeIt = m_AgvCurrentNode.find(agvID);
        if(nodeIt != m_AgvCurrentNode.end() && nodeIt->second == nodeID) {
            m_NodeOccupancy.erase(nodeID);
            m_AgvCurrentNode.erase(agvID);
            PrintState(agvID);
        }
    }

    void ClearExecutionState(uint32_t agvID) 
    {
        auto edgeIt = m_AgvCurrentEdge.find(agvID);
        if (edgeIt != m_AgvCurrentEdge.end()) {
            m_EdgeOccupancy.erase(edgeIt->second);
            m_AgvCurrentEdge.erase(edgeIt);
        }
        auto nodeIt = m_AgvCurrentNode.find(agvID);
        if (nodeIt != m_AgvCurrentNode.end()) {
            m_NodeOccupancy.erase(nodeIt->second);
            m_AgvCurrentNode.erase(nodeIt);
        }
    }

    void PrintState(uint32_t agvID)
    {
        bool hasNode = m_AgvCurrentNode.count(agvID) > 0;
        bool hasEdge = m_AgvCurrentEdge.count(agvID) > 0;
        
        std::string log = "[REAL OCCUPY] AGV " + std::to_string(agvID) + " -> ";
        if (hasNode && hasEdge) log += "NODE & EDGE (점유 오버랩 교차구간)";
        else if (hasNode) log += "NODE 전용 점유 (완전 정지)";
        else if (hasEdge) log += "EDGE 전용 점유 (링크 순항 상태)";
        else log += "RELEASED (완전 해제)";
        
        std::cout << log << "\n";
    }
};