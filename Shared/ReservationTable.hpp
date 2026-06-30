#pragma once
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <algorithm>


enum class ReservationType
{
    Normal,     // 일반적인 이동 예약
    Waiting,    // 길 막힘으로 인한 1초 임시 대기 (재탐색용)
    Goal        // 최종 목적지에 도착한 후의 장기 알박기
};

// 1. 시공간 구간 (Float 기반)
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
#include <iostream>

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
            if (interval.Overlaps(_startTime, _endTime))
            {
                    std::cout
            << "Conflict node "
            << _nodeID
            << " requester "
            << _ignoreAgvID
            << " existing "
            << interval.agvID
            << " existing ["
            << interval.start
            << ","
            << interval.end
            << "] request ["
            << _startTime
            << ","
            << _endTime
            << "]\n";
            return false;
            }
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

    // 🌟 [핵심 2] 예약 시 타입을 받도록 수정 (기본값은 Normal)
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

    void ClearFutureReservations(uint32_t _agvID, float _currentTime)
    {
        for (auto& pair : m_NodeTable)
        {
            auto& intervals = pair.second;
            intervals.erase(std::remove_if(intervals.begin(), intervals.end(),
                [_agvID, _currentTime](const TimeInterval& t) { 
                    if (t.agvID != _agvID) return false;
                    
                    if (t.type == ReservationType::Waiting || t.type == ReservationType::Goal) return true;
                    return t.start >= _currentTime; 
                }),
                intervals.end());
        }
        
        // 엣지 장부 정리
        for (auto& pair : m_EdgeTable)
        {
            auto& intervals = pair.second;
            intervals.erase(std::remove_if(intervals.begin(), intervals.end(),
                [_agvID, _currentTime](const TimeInterval& t) { 
                    if (t.agvID != _agvID) return false;
                    
                    if (t.type == ReservationType::Waiting || t.type == ReservationType::Goal) return true;
                    return t.start >= _currentTime; 
                }),
                intervals.end());
        }
    }

private:
    ReservationTable() = default;

    std::unordered_map<uint32_t, std::vector<TimeInterval>> m_NodeTable;
    std::unordered_map<uint64_t, std::vector<TimeInterval>> m_EdgeTable;
};