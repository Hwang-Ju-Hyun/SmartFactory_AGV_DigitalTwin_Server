// #pragma once
// #include <unordered_map>
// #include <vector>
// #include <cstdint>
// #include <algorithm>
// #include <iostream>
// #include <cmath>
// #include "OccupancyProvider.hpp" // 연동

// enum class ReservationType { Normal, Goal };

// struct TimeInterval 
// {
//     float start; float end; uint32_t agvID; ReservationType type;
//     bool Overlaps(float s, float e) const { return !(e <= start || s >= end); }
// };

// inline uint64_t MakeEdgeKey(uint32_t from, uint32_t to) 
// {
//     uint32_t minNode = std::min(from, to);
//     uint32_t maxNode = std::max(from, to);
//     return (static_cast<uint64_t>(minNode) << 32) | static_cast<uint64_t>(maxNode);
// }

// class ReservationTable
// {
// private:
//     std::unordered_map<uint32_t, std::vector<TimeInterval>> m_NodeReservations;
//     std::unordered_map<uint64_t, std::vector<TimeInterval>> m_EdgeReservations;
//     ReservationTable() = default;

// public:
//     static ReservationTable& GetInstance() { static ReservationTable instance; return instance; }

//     bool IsNodeFree(uint32_t _nodeID, float _startTime, float _endTime, uint32_t _ignoreAgvID) const {
//         auto it = m_NodeReservations.find(_nodeID);
//         if (it == m_NodeReservations.end()) return true;
//         for (const auto& interval : it->second) {
//             if (interval.agvID == _ignoreAgvID) continue; 
//             if (interval.Overlaps(_startTime, _endTime)) return false;
//         }
//         return true;
//     }

//     bool IsEdgeFree(uint32_t _from, uint32_t _to, float _startTime, float _endTime, uint32_t _ignoreAgvID) const {
//         uint64_t key = MakeEdgeKey(_from, _to);
//         auto it = m_EdgeReservations.find(key);
//         if (it == m_EdgeReservations.end()) return true;
//         for (const auto& interval : it->second) {
//             if (interval.agvID == _ignoreAgvID) continue;
//             if (interval.Overlaps(_startTime, _endTime)) return false;
//         }
//         return true;
//     }

//     void ReserveNode(uint32_t _nodeID, float _startTime, float _endTime, uint32_t _agvID, ReservationType _type = ReservationType::Normal) {
//         auto& intervals = m_NodeReservations[_nodeID];
//         for (const auto& interval : intervals) {
//             if (interval.agvID == _agvID && std::abs(interval.start - _startTime) < 0.001f && interval.type == _type) return;
//         }
//         intervals.push_back({ _startTime, _endTime, _agvID, _type });
//         std::sort(intervals.begin(), intervals.end(), [](const TimeInterval& a, const TimeInterval& b) {return a.start < b.start;});
//     }

//     void ReserveEdge(uint32_t _from, uint32_t _to, float _startTime, float _endTime, uint32_t _agvID, ReservationType _type = ReservationType::Normal) {
//         uint64_t edgeKey = MakeEdgeKey(_from, _to);
//         auto& intervals = m_EdgeReservations[edgeKey];
//         for (const auto& interval : intervals) {
//             if (interval.agvID == _agvID && std::abs(interval.start- _startTime) < 0.001f && interval.type == _type) return;
//         }
//         intervals.push_back({ _startTime, _endTime, _agvID, _type });
//         std::sort(intervals.begin(), intervals.end(), [](const TimeInterval& a, const TimeInterval& b) {return a.start < b.start;});
//     }
    
//     void OverrideFutureReservations(uint32_t _agvID, float _serverTime, float _safetyMargin) {        
//         float cutoffTime = _serverTime + _safetyMargin;
//         for (auto mapIt = m_NodeReservations.begin(); mapIt != m_NodeReservations.end(); ) {
//             for (auto it = mapIt->second.begin(); it != mapIt->second.end(); ) {
//                 if (it->agvID == _agvID) {
//                     if (it->start > cutoffTime) { it = mapIt->second.erase(it); continue; }
//                     else if (it->start <= cutoffTime && it->end > cutoffTime) {
//                         it->end = cutoffTime;
//                         if (it->type == ReservationType::Goal) it->type = ReservationType::Normal; 
//                     }
//                 }
//                 ++it;
//             }
//             if (mapIt->second.empty()) mapIt = m_NodeReservations.erase(mapIt); else ++mapIt;
//         }                               
//         for (auto mapIt = m_EdgeReservations.begin(); mapIt != m_EdgeReservations.end(); ) {
//             auto& intervals = mapIt->second;
//             for (auto it = intervals.begin(); it != intervals.end(); ) {
//                 if (it->agvID == _agvID) {
//                     if (it->start > cutoffTime) { it = intervals.erase(it); continue; }
//                     else if (it->start <= cutoffTime && it->end > cutoffTime) {
//                         it->end = cutoffTime;
//                         if (it->type == ReservationType::Goal) it->type = ReservationType::Normal; 
//                     }
//                 }
//                 ++it;
//             }
//             if (intervals.empty()) mapIt = m_EdgeReservations.erase(mapIt); else ++mapIt;
//         }
//     }

//     // [Atomic 트랜잭션 전용 함수] 분리된 OccupancyProvider와 정밀 연동 수행
//     bool TryOccupyEdgeForExecution(uint32_t agvID, uint32_t fromNode, uint32_t toNode, float serverTime, float expectedArrivalTime)
//     {std::cout << "[TRY EXEC] AGV " << agvID
//           << " " << fromNode << " -> " << toNode << std::endl;
//         uint64_t edgeKey = MakeEdgeKey(fromNode, toNode);        
//         auto& occ = OccupancyProvider::GetInstance();

//         // 1단계: 실시간 물리 상태 검사
//         if (occ.IsEdgeOccupiedByOther(edgeKey, agvID)) {
//             std::cout << "[EXEC WAIT] AGV " << agvID << " : 링크 " << fromNode << "->" << toNode << " 실시간 주행차량 존재 (by AGV " << occ.GetEdgeOccupant(edgeKey) << ")\n";
//             return false;
//         }
//         if (occ.IsNodeOccupiedByOther(toNode, agvID)) {
//             std::cout << "[EXEC WAIT] AGV " << agvID << " : 노드 " << toNode << " 에 교차 정지차량 존재 (by AGV " << occ.GetNodeOccupant(toNode) << ")\n";
//             return false;
//         }
        
//         // 2단계: 최상위 시공간 예약계획 검사
//         if (!IsNodeFree(toNode, serverTime, expectedArrivalTime, agvID)) {
//             std::cout << "[PLAN WAIT] AGV " << agvID << " : 노드 " << toNode << " 시공간 계획 테이블 선점 확인\n";
//             return false;
//         }
//         if (!IsEdgeFree(fromNode, toNode, serverTime, expectedArrivalTime, agvID)) {
//             std::cout << "[PLAN WAIT] AGV " << agvID << " : 링크 " << fromNode << "->" << toNode << " 시공간 계획 테이블 선점 확인\n";
//             return false;
//         }

//         // 원자적 물리 점유 도장 찍기
//         occ.SetEdgeOccupancy(edgeKey, agvID);
//         return true; 
//     }

//     void ReleaseAllOccupancyAndFutureReservations(uint32_t agvID, float serverTime) {
//         OccupancyProvider::GetInstance().ClearExecutionState(agvID);
//         OverrideFutureReservations(agvID, serverTime, 0.0f);
//     }
// };

#pragma once
#include <unordered_map>
#include <vector>
#include <cstdint>
#include <algorithm>
#include <iostream>
#include <cmath>
#include "OccupancyProvider.hpp" // 연동

enum class ReservationType { Normal, Goal };

struct TimeInterval 
{
    float start; float end; uint32_t agvID; ReservationType type;
    bool Overlaps(float s, float e) const { return !(e <= start || s >= end); }
};

inline uint64_t MakeEdgeKey(uint32_t from, uint32_t to) 
{
    uint32_t minNode = std::min(from, to);
    uint32_t maxNode = std::max(from, to);
    return (static_cast<uint64_t>(minNode) << 32) | static_cast<uint64_t>(maxNode);
}

class ReservationTable
{
private:
    std::unordered_map<uint32_t, std::vector<TimeInterval>> m_NodeReservations;
    std::unordered_map<uint64_t, std::vector<TimeInterval>> m_EdgeReservations;
    ReservationTable() = default;

public:
    static ReservationTable& GetInstance() { static ReservationTable instance; return instance; }

    bool IsNodeFree(uint32_t _nodeID, float _startTime, float _endTime, uint32_t _ignoreAgvID) const {
        auto it = m_NodeReservations.find(_nodeID);
        if (it == m_NodeReservations.end()) return true;
        for (const auto& interval : it->second) {
            if (interval.agvID == _ignoreAgvID) continue; 
            if (interval.Overlaps(_startTime, _endTime)) return false;
        }
        return true;
    }

    bool IsEdgeFree(uint32_t _from, uint32_t _to, float _startTime, float _endTime, uint32_t _ignoreAgvID) const {
        uint64_t key = MakeEdgeKey(_from, _to);
        auto it = m_EdgeReservations.find(key);
        if (it == m_EdgeReservations.end()) return true;
        for (const auto& interval : it->second) {
            if (interval.agvID == _ignoreAgvID) continue;
            if (interval.Overlaps(_startTime, _endTime)) return false;
        }
        return true;
    }

    void ReserveNode(uint32_t _nodeID, float _startTime, float _endTime, uint32_t _agvID, ReservationType _type = ReservationType::Normal) {
        auto& intervals = m_NodeReservations[_nodeID];
        for (const auto& interval : intervals) {
            if (interval.agvID == _agvID && std::abs(interval.start - _startTime) < 0.001f && interval.type == _type) return;
        }
        intervals.push_back({ _startTime, _endTime, _agvID, _type });
        std::sort(intervals.begin(), intervals.end(), [](const TimeInterval& a, const TimeInterval& b) {return a.start < b.start;});
    }

    void ReserveEdge(uint32_t _from, uint32_t _to, float _startTime, float _endTime, uint32_t _agvID, ReservationType _type = ReservationType::Normal) {
        uint64_t edgeKey = MakeEdgeKey(_from, _to);
        auto& intervals = m_EdgeReservations[edgeKey];
        for (const auto& interval : intervals) {
            if (interval.agvID == _agvID && std::abs(interval.start- _startTime) < 0.001f && interval.type == _type) return;
        }
        intervals.push_back({ _startTime, _endTime, _agvID, _type });
        std::sort(intervals.begin(), intervals.end(), [](const TimeInterval& a, const TimeInterval& b) {return a.start < b.start;});
    }
    
    void OverrideFutureReservations(uint32_t _agvID, float _serverTime, float _safetyMargin) {        
        float cutoffTime = _serverTime + _safetyMargin;
        for (auto mapIt = m_NodeReservations.begin(); mapIt != m_NodeReservations.end(); ) {
            for (auto it = mapIt->second.begin(); it != mapIt->second.end(); ) {
                if (it->agvID == _agvID) {
                    if (it->start > cutoffTime) { it = mapIt->second.erase(it); continue; }
                    else if (it->start <= cutoffTime && it->end > cutoffTime) {
                        it->end = cutoffTime;
                        if (it->type == ReservationType::Goal) it->type = ReservationType::Normal; 
                    }
                }
                ++it;
            }
            if (mapIt->second.empty()) mapIt = m_NodeReservations.erase(mapIt); else ++mapIt;
        }                               
        for (auto mapIt = m_EdgeReservations.begin(); mapIt != m_EdgeReservations.end(); ) {
            auto& intervals = mapIt->second;
            for (auto it = intervals.begin(); it != intervals.end(); ) {
                if (it->agvID == _agvID) {
                    if (it->start > cutoffTime) { it = intervals.erase(it); continue; }
                    else if (it->start <= cutoffTime && it->end > cutoffTime) {
                        it->end = cutoffTime;
                        if (it->type == ReservationType::Goal) it->type = ReservationType::Normal; 
                    }
                }
                ++it;
            }
            if (intervals.empty()) mapIt = m_EdgeReservations.erase(mapIt); else ++mapIt;
        }
    }

    // [Atomic 트랜잭션 전용 함수] 분리된 OccupancyProvider와 정밀 연동 수행
    bool TryOccupyEdgeForExecution(uint32_t agvID, uint32_t fromNode, uint32_t toNode, float serverTime, float expectedArrivalTime)
    {
        constexpr float EXECUTION_CLEARANCE_TIME = 0.6f;
        // std::cout << "[TRY EXEC] AGV " << agvID
        //   << " " << fromNode << " -> " << toNode << std::endl;

        uint64_t edgeKey = MakeEdgeKey(fromNode, toNode);        
        auto& occ = OccupancyProvider::GetInstance();

        float arrivalCheckStart = std::max(serverTime, expectedArrivalTime);
        float arrivalCheckEnd = arrivalCheckStart + EXECUTION_CLEARANCE_TIME;
        float edgeCheckEnd = expectedArrivalTime + EXECUTION_CLEARANCE_TIME;
        if (edgeCheckEnd < serverTime) 
            edgeCheckEnd = serverTime + EXECUTION_CLEARANCE_TIME;

        // 1단계: 실시간 물리 상태 검사. 링크 점유는 즉시 충돌 영역이므로 출발 전에 막는다.
        if (occ.IsEdgeOccupiedByOther(edgeKey, agvID)) {
            //std::cout << "[EXEC WAIT] AGV " << agvID << " : 링크 " << fromNode << "->" << toNode << " 실시간 주행차량 존재 (by AGV " << occ.GetEdgeOccupant(edgeKey) << ")\n";
            return false;
        }
        if (occ.IsNodeOccupiedByOther(toNode, agvID)) {
            //std::cout << "[EXEC WAIT] AGV " << agvID << " : 노드 " << toNode << " 실제 점유 존재 (by AGV " << occ.GetNodeOccupant(toNode) << ")\n";
            return false;
        }
        
        // 2단계: 최상위 시공간 예약계획 검사.
        // 목적지 노드는 지금부터 도착 전까지 비어있을 필요가 없고, 도착 시각 이후만 안전하면 된다.
        if (!IsEdgeFree(fromNode, toNode, serverTime, edgeCheckEnd, agvID)) 
        {
            //std::cout << "[PLAN WAIT] AGV " << agvID << " : 링크 " << fromNode << "->" << toNode << " 시공간 계획 테이블 선점 확인\n";
            return false;
        }
        if (!IsNodeFree(toNode, arrivalCheckStart, arrivalCheckEnd, agvID)) 
        {
            //std::cout << "[PLAN WAIT] AGV " << agvID << " : 노드 " << toNode << " 도착 시간창 선점 확인\n";
            return false;
        }

        // 원자적 물리 점유 도장 찍기
        occ.SetEdgeOccupancy(edgeKey, agvID);
        return true; 
    }

    void ReleaseAllOccupancyAndFutureReservations(uint32_t agvID, float serverTime) 
    {
        OccupancyProvider::GetInstance().ClearExecutionState(agvID);
        OverrideFutureReservations(agvID, serverTime, 0.0f);
    }
};
