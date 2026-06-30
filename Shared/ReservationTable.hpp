#pragma once
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <algorithm>

// 🌟 예약 타입 정의
enum class ReservationType
{
    Normal,     // 경로 이동 중의 점유
    Goal        // 목적지 도달 후 장기 점유
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
        auto it = m_NodeTable.find(_nodeID);
        if (it == m_NodeTable.end()) return true;

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
        auto it = m_EdgeTable.find(key);
        if (it == m_EdgeTable.end()) return true;

        for (const auto& interval : it->second)
        {
            if (interval.agvID == _ignoreAgvID) continue;
            if (interval.Overlaps(_startTime, _endTime)) return false;
        }
        return true;
    }

    void ReserveNode(uint32_t _nodeID, float _startTime, float _endTime, uint32_t _agvID, ReservationType _type = ReservationType::Normal)
    {
        auto& intervals = m_NodeTable[_nodeID];
        for (const auto& interval : intervals)
        {
            if (interval.agvID == _agvID && std::abs(interval.start - _startTime) < 0.001f && std::abs(interval.end - _endTime) < 0.001f)
                return;
        }
        intervals.push_back({_startTime, _endTime, _agvID, _type});
    }

    void ReserveEdge(uint32_t _from, uint32_t _to, float _startTime, float _endTime, uint32_t _agvID, ReservationType _type = ReservationType::Normal)
    {
        uint64_t key = MakeEdgeKey(_from, _to);
        auto& intervals = m_EdgeTable[key];
        for (const auto& interval : intervals)
        {
            if (interval.agvID == _agvID && std::abs(interval.start - _startTime) < 0.001f && std::abs(interval.end - _endTime) < 0.001f)
                return;
        }
        intervals.push_back({_startTime, _endTime, _agvID, _type});
    }

    
    void OverrideFutureReservations(uint32_t _agvID, float _currentTime, float _safetyMargin)
    {
        float safeTime = _currentTime + _safetyMargin;
        
        for (auto& pair : m_NodeTable)
        {
            auto& intervals = pair.second;
            intervals.erase(std::remove_if(intervals.begin(), intervals.end(),
                [_agvID, safeTime](const TimeInterval& t) { 
                    // 현재부터 safetyMargin 이후의 "완전한 미래"만 지우고, 발밑은 잠가둠!
                    return t.agvID == _agvID && (t.type == ReservationType::Goal || t.start > safeTime); 
                }),
                intervals.end());
        }
        for (auto& pair : m_EdgeTable)
        {
            auto& intervals = pair.second;
            intervals.erase(std::remove_if(intervals.begin(), intervals.end(),
                [_agvID, safeTime](const TimeInterval& t) { 
                    return t.agvID == _agvID && (t.type == ReservationType::Goal || t.start > safeTime); 
                }),
                intervals.end());
        }
    }

private:
    ReservationTable() = default;

    std::unordered_map<uint32_t, std::vector<TimeInterval>> m_NodeTable;
    std::unordered_map<uint64_t, std::vector<TimeInterval>> m_EdgeTable;
};