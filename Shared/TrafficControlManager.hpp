#pragma once
#include <unordered_map>
#include <string>
#include <cstdint>
#include <cmath>
#include <vector>

constexpr float SLOT_DURATION = 1.0f; 

struct SlotRange 
{ 
    int start; 
    int end; 
};

class TrafficManager 
{
public:
    static TrafficManager& GetInstance() { static TrafficManager instance; return instance; }

    // float 시간을 0.1초 단위의 정수 슬롯으로 변환 (오차 방지 )
    //static int TimeToSlot(float _time) { return static_cast<int>(std::round(_time));}        

    static int GetStartSlot(float _time) { return static_cast<int>(std::floor(_time / SLOT_DURATION)); }
    static int GetEndSlot(float _time) { return static_cast<int>(std::ceil(_time / SLOT_DURATION)); }

    static SlotRange GetSlotRange(float _enterTime, float _leaveTime) 
    {
        return { GetStartSlot(_enterTime), GetEndSlot(_leaveTime) };
    }

    // 예약 장부 초기화 (로봇이 새 길을 찾을 때 기존 예약을 지움)
    void ClearFutureReservations(uint32_t _agvID, float _currentTime);

    // 노드 점유 예약
    bool ReserveNode(uint32_t _nodeID, float _enterTime, float _leaveTime, uint32_t _agvID);    
    // 링크(도로) 점유 예약 (마주보고 달리는 정면 충돌 방지용)
    bool ReserveLink(uint32_t _fromNode, uint32_t _toNode, float _enterTime, float _leaveTime, uint32_t _agvID);

    // 해당 시간에 노드가 비어있는지 확인 (A* 검색 시 사용)
    bool IsNodeAvailable(uint32_t _nodeID, float _enterTime, float _leaveTime, uint32_t _agvID);
    
    // 해당 시간에 링크가 비어있는지 확인
    bool IsLinkAvailable(uint32_t _fromNode, uint32_t _toNode, float _enterTime, float _leaveTime, uint32_t _agvID);           
private:
    TrafficManager() = default;

    // "NodeID_TimeSlot" -> agvID 형태의 해시맵
    std::unordered_map<std::string, uint32_t> m_NodeReservations;
    
    // "FromNode_ToNode_TimeSlot" -> agvID 형태의 해시맵
    std::unordered_map<std::string, uint32_t> m_LinkReservations;

    //AGV ID별로 자신이 등록한 예약 키 목록을 들고 있음
    std::unordered_map<uint32_t, std::vector<std::string>> m_AgvOwnedKeys;            
};