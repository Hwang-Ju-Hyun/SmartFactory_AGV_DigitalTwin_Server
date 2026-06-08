#pragma once
#include <cstdint>
#include <unordered_map>
#include <vector>

struct NodeReservation
{
    uint32_t agvID;
    float startTime;
    float endTime;
};

class TrafficControlManager
{
private:
    TrafficControlManager();
public:
    static TrafficControlManager& GetInstance()
    {
        static TrafficControlManager instance;
        return instance;
    }
private:
    //키(NodeID) -> 값(그 노드에 걸려있는 시간대별 예약 리스트)
    std::unordered_map<uint32_t,std::vector<NodeReservation>> m_ReservationTable;
public:    
    //특정 노드의 특정 시간대가 사용 가능한지 검사
    bool IsTimeWindowAvailable(uint32_t _nodeID, float _start, float _end, uint32_t _myAgvID);

    //경로가 확정되었을 때 장부에 예약 도장 찍기
    void ReserveNode(uint32_t _nodeID, float _start, float _end, uint32_t _agvID);

    //목적지가 바뀌거나 경로가 재계획되었을 때 기존 예약 싹 지우기
    void ClearAgvReservations(uint32_t _agvID);

    void RegisterGoal(uint32_t _nodeID,uint32_t _agvID);
};