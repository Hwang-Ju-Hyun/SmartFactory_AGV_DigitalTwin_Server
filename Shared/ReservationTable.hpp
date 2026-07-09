#pragma once
#include <unordered_map>
#include <vector>
#include <cstdint>
#include <algorithm>
#include <iostream>
#include <cmath>

enum class ReservationType
{
    Normal,     
    Goal        
};

struct TimeInterval
{
    float start;
    float end;
    uint32_t agvID;
    ReservationType type;

    bool Overlaps(float s, float e) const
    {
        return !(e <= start || s >= end);
    }
};

inline uint64_t MakeEdgeKey(uint32_t from, uint32_t to)
{
    uint32_t minNode = std::min(from, to);
    uint32_t maxNode = std::max(from, to);
    return (static_cast<uint64_t>(minNode) << 32) | static_cast<uint64_t>(maxNode);
}

class ReservationTable
{
public:
    static ReservationTable& GetInstance() { static ReservationTable instance; return instance; }

    bool IsNodeFree(uint32_t _nodeID, float _startTime, float _endTime, uint32_t _ignoreAgvID) const
    {
        auto it = m_NodeReservations.find(_nodeID);
        if (it == m_NodeReservations.end()) return true;

        for (const auto& interval : it->second)
        {
            if (interval.agvID == _ignoreAgvID) continue; 
            if (interval.Overlaps(_startTime, _endTime)) return false;
        }
        return true;
    }

    bool IsEdgeFree(uint32_t _from, uint32_t _to, float _startTime, float _endTime, uint32_t _ignoreAgvID) const
    {
        uint64_t key = MakeEdgeKey(_from, _to);
        auto it = m_EdgeReservations.find(key);
        if (it == m_EdgeReservations.end()) return true;

        for (const auto& interval : it->second)
        {
            if (interval.agvID == _ignoreAgvID) continue;
            if (interval.Overlaps(_startTime, _endTime)) return false;
        }
        return true;
    }

    void ReserveNode(uint32_t _nodeID, float _startTime, float _endTime, uint32_t _agvID, ReservationType _type = ReservationType::Normal)
    {
        auto& intervals = m_NodeReservations[_nodeID];
        
        for (const auto& interval : intervals)
        {
            if (interval.agvID == _agvID && 
                std::abs(interval.start - _startTime) < 0.001f &&
                interval.type == _type) 
            {
                return;
            }
        }

        intervals.push_back({ _startTime, _endTime, _agvID, _type });
        std::sort(intervals.begin(), intervals.end(), [](const TimeInterval& a, const TimeInterval& b) {return a.start < b.start;});
    }

    void ReserveEdge(uint32_t _from, uint32_t _to, float _startTime, float _endTime, uint32_t _agvID, ReservationType _type = ReservationType::Normal)
    {
        uint64_t edgeKey = MakeEdgeKey(_from, _to);
        auto& intervals = m_EdgeReservations[edgeKey];

        for (const auto& interval : intervals)
        {
            if (interval.agvID == _agvID && std::abs(interval.start- _startTime) < 0.001f && interval.type == _type) 
            {
                return;
            }
        }

        intervals.push_back({ _startTime, _endTime, _agvID, _type });
        std::sort(intervals.begin(), intervals.end(), [](const TimeInterval& a, const TimeInterval& b) {return a.start < b.start;});
    }
    
    void OverrideFutureReservations(uint32_t _agvID, float _serverTime, float _safetyMargin)
    {        
        float cutoffTime = _serverTime + _safetyMargin;

        for (auto mapIt = m_NodeReservations.begin(); mapIt != m_NodeReservations.end(); ) 
        {
            for (auto it = mapIt->second.begin(); it != mapIt->second.end(); ) 
            {
                if (it->agvID == _agvID) 
                {
                    if (it->start > cutoffTime) 
                    {
                        it = mapIt->second.erase(it); 
                        continue; 
                    } else if (it->start <= cutoffTime && it->end > cutoffTime) 
                    {
                        it->end = cutoffTime;
                        if (it->type == ReservationType::Goal) 
                            it->type = ReservationType::Normal; 
                    }
                }
                ++it;
            }
            if (mapIt->second.empty()) mapIt = m_NodeReservations.erase(mapIt);
            else ++mapIt;
        }                               
            
        for (auto mapIt = m_EdgeReservations.begin(); mapIt != m_EdgeReservations.end(); )
        {
            auto& intervals = mapIt->second;
            for (auto it = intervals.begin(); it != intervals.end(); )
            {
                if (it->agvID == _agvID)
                {
                    if (it->start > cutoffTime)
                    {
                        it = intervals.erase(it);
                        continue; 
                    }
                    else if (it->start <= cutoffTime && it->end > cutoffTime)
                    {
                        it->end = cutoffTime;
                        if (it->type == ReservationType::Goal) {
                            it->type = ReservationType::Normal; 
                        }
                    }
                }
                ++it;
            }
            
            if (intervals.empty()) mapIt = m_EdgeReservations.erase(mapIt);
            else ++mapIt;
        }
    }

    void OccupyNode(uint32_t agvID, uint32_t nodeID)
    {
        // 1. 기존 점유 중이던 링크 확인 및 해제
        auto edgeIt = m_AgvCurrentEdge.find(agvID);

        if (edgeIt != m_AgvCurrentEdge.end()) 
        {
            uint64_t oldEdge = edgeIt->second;
            m_EdgeOccupancy.erase(oldEdge);
            m_AgvCurrentEdge.erase(edgeIt);
        }

        // 2. 기존 점유 중이던 노드 확인 및 해제
        auto nodeIt = m_AgvCurrentNode.find(agvID);
        if(nodeIt != m_AgvCurrentNode.end()) 
        {
            uint32_t oldNode = nodeIt->second;
            m_NodeOccupancy.erase(oldNode);
        }

        // 3. 새 노드 점유
        m_NodeOccupancy[nodeID] = agvID;
        m_AgvCurrentNode[agvID] = nodeID;

        DebugPrintOccupancy();
    }

    
    void OccupyEdge(uint32_t agvID, uint32_t fromNode, uint32_t toNode)
    {
        
        // 1. 노드 점유 해제
        m_NodeOccupancy.erase(fromNode); 
        m_AgvCurrentNode.erase(agvID);

        // 2. 새 링크 점유
        uint64_t edgeKey = MakeEdgeKey(fromNode, toNode);
        m_EdgeOccupancy[edgeKey] = agvID;
        m_AgvCurrentEdge[agvID] = edgeKey; // 현재 타고 있는 링크 기록
        DebugPrintOccupancy();
    }

    bool IsResourceClearForExecution(uint32_t agvID, uint32_t fromNode, uint32_t toNode)
    {
        uint64_t edgeKey = MakeEdgeKey(fromNode, toNode);        

    if(m_EdgeOccupancy.count(edgeKey))
    {
        std::cout
            << "[BLOCK EDGE] "
            << agvID
            << " waiting "
            << fromNode
            << "->"
            << toNode
            << " occupied by "
            << m_EdgeOccupancy.at(edgeKey)
            << std::endl;
    }
std::cout
<< "[RESOURCE]"
<< " AGV="
<< agvID
<< " "
<< fromNode
<< "->"
<< toNode
<< " edge="
<< (m_EdgeOccupancy.count(edgeKey) ? "BUSY" : "FREE")
<< " node="
<< (m_NodeOccupancy.count(toNode) ? "BUSY" : "FREE")
<< std::endl;
    if(m_NodeOccupancy.count(toNode))
    {
        std::cout
            << "[BLOCK NODE] "
            << agvID
            << " waiting node "
            << toNode
            << " occupied by "
            << m_NodeOccupancy.at(toNode)
            << std::endl;
    }
        if (m_EdgeOccupancy.count(edgeKey) && m_EdgeOccupancy.at(edgeKey) != agvID) return false;
        if (m_NodeOccupancy.count(toNode) && m_NodeOccupancy.at(toNode) != agvID) return false;
        
        return true;
    }
    void ReleaseAllOccupancyAndReservations(uint32_t agvID)
    {
        // ==========================================
        // 1. 현재 물리적 점유(Occupancy) 강제 해제
        // ==========================================
        auto edgeIt = m_AgvCurrentEdge.find(agvID);
        if (edgeIt != m_AgvCurrentEdge.end()) 
        {
            m_EdgeOccupancy.erase(edgeIt->second);
            m_AgvCurrentEdge.erase(edgeIt);
        }

        auto nodeIt = m_AgvCurrentNode.find(agvID);
        if (nodeIt != m_AgvCurrentNode.end()) 
        {
            m_NodeOccupancy.erase(nodeIt->second);
            m_AgvCurrentNode.erase(nodeIt);
        }

        // ==========================================
        // 2. 미래 예약(Reservation) 장부 강제 파기
        // ==========================================
        for (auto mapIt = m_NodeReservations.begin(); mapIt != m_NodeReservations.end(); ) 
        {
            for (auto it = mapIt->second.begin(); it != mapIt->second.end(); ) 
            {
                if (it->agvID == agvID) it = mapIt->second.erase(it);
                else ++it;
            }
            if (mapIt->second.empty()) mapIt = m_NodeReservations.erase(mapIt);
            else ++mapIt;
        }

        for (auto mapIt = m_EdgeReservations.begin(); mapIt != m_EdgeReservations.end(); ) 
        {
            for (auto it = mapIt->second.begin(); it != mapIt->second.end(); ) 
            {
                if (it->agvID == agvID) it = mapIt->second.erase(it);
                else ++it;
            }
            if (mapIt->second.empty()) mapIt = m_EdgeReservations.erase(mapIt);
            else ++mapIt;
        }
        
        std::cout << "[ReservationTable] AGV " << agvID << "번의 모든 점유 및 예약 데이터가 삭제되었습니다." << std::endl;
    }
    void DebugPrintOccupancy()
{
    std::cout << "\n========== OCCUPANCY ==========\n";

    std::cout << "[Nodes]\n";
    for (auto& p : m_NodeOccupancy)
    {
        std::cout
            << " Node "
            << p.first
            << " -> AGV "
            << p.second
            << "\n";
    }

    std::cout << "[Edges]\n";
    for (auto& p : m_EdgeOccupancy)
    {
        uint32_t from = p.first >> 32;
        uint32_t to   = p.first & 0xffffffff;

        std::cout
            << " Edge "
            << from
            << "->"
            << to
            << " -> AGV "
            << p.second
            << "\n";
    }

    std::cout << "===============================\n";
}
private:
    ReservationTable() = default;

    std::unordered_map<uint32_t, std::vector<TimeInterval>> m_NodeReservations;
    std::unordered_map<uint64_t, std::vector<TimeInterval>> m_EdgeReservations;

    std::unordered_map<uint32_t, uint32_t> m_NodeOccupancy; 
    std::unordered_map<uint64_t, uint32_t> m_EdgeOccupancy; 
    std::unordered_map<uint32_t, uint32_t> m_AgvCurrentNode;
    
    std::unordered_map<uint32_t, uint64_t> m_AgvCurrentEdge; 
};