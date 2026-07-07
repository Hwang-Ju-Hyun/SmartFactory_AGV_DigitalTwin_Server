#pragma once
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <algorithm>
#include <iostream>


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
        if (interval.agvID ==  _agvID && 
            std::abs(interval.start - _startTime) < 0.001f &&
            interval.type == _type) 
        {
            return; // 완벽히 똑같은 예약이 이미 존재함
        }
    }

    intervals.push_back({ _startTime, _endTime, _agvID, _type });

    // 🌟 [버그 3 수정] 무조건 시작 시간 순서대로 빳빳하게 정렬!
    std::sort(intervals.begin(), intervals.end(), [](const TimeInterval& a, const TimeInterval& b) {return a.start < b.start;});

    }

    void ReserveEdge(uint32_t _from, uint32_t _to, float _startTime, float _endTime, uint32_t _agvID, ReservationType _type = ReservationType::Normal)
    {
        uint32_t minNode = std::min(_from, _to);
    uint32_t maxNode = std::max(_from, _to);
    uint64_t edgeKey = ((uint64_t)minNode << 32) | maxNode;

    auto& intervals = m_EdgeTable[edgeKey];

    // 🌟 [버그 4 수정]
    for (const auto& interval : intervals)
    {
        if (interval.agvID == _agvID && std::abs(interval.start- _startTime) < 0.001f &&interval.type == _type) 
        {
            return;
        }
    }

    intervals.push_back({ _startTime, _endTime, _agvID, _type });

    // 🌟 [버그 5 수정] Edge 예약도 깔끔하게 정렬!
    std::sort(intervals.begin(), intervals.end(), [](const TimeInterval& a, const TimeInterval& b) {return a.start < b.start;});

    }

    
    void OverrideFutureReservations(uint32_t _agvID, float _currentTime, float _safetyMargin)
    {
        // float safeTime = _currentTime + _safetyMargin;
        
        // for (auto& pair : m_NodeTable)
        // {
        //     auto& intervals = pair.second;
        //     intervals.erase(std::remove_if(intervals.begin(), intervals.end(),
        //         [_agvID, safeTime](const TimeInterval& t) { 
        //             // 현재부터 safetyMargin 이후의 "완전한 미래"만 지우고, 발밑은 잠가둠!
        //             return t.agvID == _agvID && (t.type == ReservationType::Goal || t.start > safeTime); 
        //         }),
        //         intervals.end());
        // }
        // for (auto& pair : m_EdgeTable)
        // {
        //     auto& intervals = pair.second;
        //     intervals.erase(std::remove_if(intervals.begin(), intervals.end(),
        //         [_agvID, safeTime](const TimeInterval& t) { 
        //             return t.agvID == _agvID && (t.type == ReservationType::Goal || t.start > safeTime); 
        //         }),
        //         intervals.end());
        // }
            float cutoffTime = _currentTime + _safetyMargin;

        // ==========================================
        // 1. Node 장부 갱신 및 메모리 정리
        // ==========================================
        for (auto mapIt = m_NodeTable.begin(); mapIt != m_NodeTable.end(); )
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
                    // 🌟 [버그 1 수정] 조건의 범위를 명확하게 명시하여 휴먼 에러 방지
                    else if (it->start <= cutoffTime && it->end > cutoffTime)
                    {
                        it->end = cutoffTime;
                        
                        
                        if (it->type == ReservationType::Goal) 
                        {
                            it->type = ReservationType::Normal; 
                        }
                    }
                }
                ++it;
            }
            
            // 🌟 [최적화 추가] 예약이 텅 빈 노드는 맵에서 통째로 날려버림 (메모리 릭 방지)
            if (intervals.empty()) {
                mapIt = m_NodeTable.erase(mapIt);
            } else {
                ++mapIt;
            }
        }
        
        // ==========================================
        // 2. Edge 장부 갱신 및 메모리 정리 (Node와 동일)
        // ==========================================
        for (auto mapIt = m_EdgeTable.begin(); mapIt != m_EdgeTable.end(); )
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
            
            if (intervals.empty()) 
            {
                mapIt = m_EdgeTable.erase(mapIt);
            }
            else 
            {
                ++mapIt;
            }
        }
    }
    void DebugPrintNode(uint32_t node)
    {
        auto it = m_NodeTable.find(node);

        if(it==m_NodeTable.end())
        {
            std::cout << "EMPTY\n";
            return;
        }

        for(auto& r : it->second)
        {
            std::cout
                << "AGV "
                << r.agvID
                << " "
                << r.start
                << " "
                << r.end
                << " "
                << (r.type==ReservationType::Goal?"Goal":"Normal")
                << std::endl;
        }
    }
private:
    ReservationTable() = default;

    std::unordered_map<uint32_t, std::vector<TimeInterval>> m_NodeTable;
    std::unordered_map<uint64_t, std::vector<TimeInterval>> m_EdgeTable;
};