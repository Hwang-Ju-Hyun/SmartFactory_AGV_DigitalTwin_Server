#pragma once
#include <cstdint>
#include <unordered_map>
#include <vector>
#include <queue>


struct NodeReservation
{
    uint32_t agvID;
    float startTime;
    float endTime;
};

struct LinkReservation
{
    uint32_t agvID;
    uint32_t fromNodeID;
    uint32_t toNodeID;
    float    startTime;
    float    endTime;
};

struct Conflict
{
    uint32_t NodeID;
    uint32_t A_agvID;
    uint32_t B_agvID;
};

class TrafficControlManager
{
private:
    TrafficControlManager();
    const float timeBuffer = 2.5f;  
    std::vector<Conflict> m_Conflicts;
public:
    static TrafficControlManager& GetInstance()
    {
        static TrafficControlManager instance;
        return instance;
    }    
public:
    //키(NodeID) -> 값(그 노드에 걸려있는 시간대별 예약 리스트)
    std::unordered_map<uint32_t,std::vector<NodeReservation>> m_ReservationTable;
    std::vector<LinkReservation> m_LinkReservations;
    std::vector<Conflict> GetConflicts()const{return m_Conflicts;}   
public:    
    //특정 노드의 특정 시간대가 사용 가능한지 검사
    bool IsTimeWindowAvailable(uint32_t _nodeID, float _start, float _end, uint32_t _myAgvID);

    //경로가 확정되었을 때 장부에 예약 도장 찍기
    void ReserveNode(uint32_t _nodeID, float _start, float _end, uint32_t _agvID);

    //목적지가 바뀌거나 경로가 재계획되었을 때 기존 예약 싹 지우기
    void ClearAgvReservations(uint32_t _agvID);

    //각 예약리스트들은 한번 더 전체 검사(각 agv들은 미래를 예측하지 못한상태에서 reservation table에 넣음)
    void ValidateReservation();     
    
    uint32_t GetLoserAGVofConflict(const Conflict& _conflict);

    void ReleaseNodeReservation(uint32_t _prevNodeID,uint32_t _agvID);
private:
    bool Overlap(const NodeReservation& a,const NodeReservation& b){return a.startTime < b.endTime &&a.endTime   > b.startTime;}
public:
    void ReserveLink(uint32_t _from, uint32_t _to, float _start, float _end, uint32_t _agvID);
    bool IsLinkAvailable(uint32_t _from, uint32_t _to, float _start, float _end, uint32_t _agvID);
    void ClearLinkReservations(uint32_t _agvID);     
};