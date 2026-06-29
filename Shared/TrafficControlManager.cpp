#include "TrafficControlManager.hpp"
#include <iostream>

void TrafficManager::ClearFutureReservations(uint32_t _agvID, float _currentTime)
{            
    if (m_AgvOwnedKeys.find(_agvID) == m_AgvOwnedKeys.end()) 
        return;

    int safeSlot = GetEndSlot(_currentTime);
    std::vector<std::string> keysToKeep; // 지우면 안 되는 '과거~현재'의 키들

    for (const std::string& key : m_AgvOwnedKeys[_agvID])
    {
        // 키 문자열 포맷: "NodeID_TimeSlot" 또는 "Node1_Node2_TimeSlot"
        // 문자열의 맨 뒤에서부터 '_'를 찾아 TimeSlot 숫자를 추출함
        size_t lastUnderscorePos = key.find_last_of('_');
        if (lastUnderscorePos != std::string::npos)
        {
            int slot = std::stoi(key.substr(lastUnderscorePos + 1));
            
            //현재 시간의 슬롯보다 '미래'인 경우만 장부에서 삭제!
            if (slot > safeSlot)
            {
                m_NodeReservations.erase(key);
                m_LinkReservations.erase(key);
            }
            else
            {
                // 과거~현재 슬롯은 유지해야 하므로 따로 빼둠
                keysToKeep.push_back(key);
            }
        }
    }

    // 내 키 보관함을 싹 비운 뒤, 살려둔 '과거~현재' 키들만 다시 넣어줌
    m_AgvOwnedKeys[_agvID].clear();
    m_AgvOwnedKeys[_agvID] = keysToKeep;
    std::cout
<< "AGV "
<< _agvID
<< " ClearReservation "
<< safeSlot
<< std::endl;
}

bool TrafficManager::ReserveNode(uint32_t _nodeID, float _enterTime, float _leaveTime, uint32_t _agvID)
{
//     int startSlot = GetStartSlot(_enterTime);
//     int endSlot = GetEndSlot(_leaveTime);
     
//     for (int t = startSlot; t <= endSlot; ++t)
//     {
//         std::string key = std::to_string(_nodeID) + "_" + std::to_string(t);

//         auto it = m_NodeReservations.find(key);
//         if (it != m_NodeReservations.end() && it->second != _agvID)
//         {
//             // 다른 AGV가 이미 선점한 슬롯이라면 대참사 경고를 띄웁니다.
//             std::cerr << "[대참사 발생] DOUBLE RESERVE DETECTED!! "
//                       << "Node: " << _nodeID << " | Slot: " << t 
//                       << " | 기존 주인: AGV " << it->second 
//                       << " | 가해자: AGV " << _agvID << std::endl;
                      
//             // 필요하다면 assert(false); 를 걸어 실시간으로 서버를 멈추고 콜스택을 따볼 수도 있습니다.
//             continue; 
//         }


//         m_NodeReservations[key] = _agvID;
        
//         //내 키 보관함에 기록
//         m_AgvOwnedKeys[_agvID].push_back(key);
//     }
//     std::cout
// << "Reserve "
// << _nodeID
// << " "
// << startSlot
// << "~"
// << endSlot
// << " agv "
// << _agvID
// << std::endl;
if (!IsNodeAvailable(_nodeID,
                     _enterTime,
                     _leaveTime,
                     _agvID))
{
    std::cout
        << "Reserve 직전 이미 충돌 "
        << _nodeID
        << " "
        << _enterTime
        << std::endl;
}
SlotRange range = GetSlotRange(_enterTime, _leaveTime);
     
    // ==========================================================
    //  1. 검증 페이즈 (Validation): 단 하나라도 막히면 즉시 취소!
    // ==========================================================
    for (int t = range.start; t <= range.end; ++t)
    {
        std::string key = std::to_string(_nodeID) + "_" + std::to_string(t);
        auto it = m_NodeReservations.find(key);
        if (it != m_NodeReservations.end() && it->second != _agvID)
        {
            std::cerr << "[예약 거부] DOUBLE RESERVE 시도 차단! Node: " << _nodeID 
                      << " | Slot: " << t << " | 주인: AGV " << it->second 
                      << " | 가해자: AGV " << _agvID << std::endl;
            return false; //반쪽짜리 예약을 막기 위해 즉시 false 반환
        }
    }

    // ==========================================================
    // 2. 기록 페이즈 (Commit): 100% 안전할 때만 덮어씁니다.
    // ==========================================================
    for (int t = range.start; t <= range.end; ++t)
    {
        std::string key = std::to_string(_nodeID) + "_" + std::to_string(t);
        m_NodeReservations[key] = _agvID;
        m_AgvOwnedKeys[_agvID].push_back(key);
    }
    
    return true;
}

void TrafficManager::ReserveLink(uint32_t _fromNode, uint32_t _toNode, float _enterTime, float _leaveTime, uint32_t _agvID)
{
    int startSlot = GetStartSlot(_enterTime);
    int endSlot = GetEndSlot(_leaveTime);

    // 무방향 그래프라면 정면 충돌 방지를 위해 번호가 작은 노드를 항상 앞에 두어 키를 생성
    uint32_t minNode = std::min(_fromNode, _toNode);
    uint32_t maxNode = std::max(_fromNode, _toNode);

    for (int t = startSlot; t <= endSlot; ++t)
    {
        std::string key = std::to_string(minNode) + "_" + std::to_string(maxNode) + "_" + std::to_string(t);
        m_LinkReservations[key] = _agvID;

        m_AgvOwnedKeys[_agvID].push_back(key);
    }
}

bool TrafficManager::IsNodeAvailable(uint32_t _nodeID, float _enterTime, float _leaveTime, uint32_t _agvID)
{
    int startSlot = GetStartSlot(_enterTime);
    int endSlot = GetEndSlot(_leaveTime);

    for (int t = startSlot; t <= endSlot; ++t)
    {
        std::string key = std::to_string(_nodeID) + "_" + std::to_string(t);
        if (m_NodeReservations.find(key) != m_NodeReservations.end() && m_NodeReservations[key] != _agvID)
        {
            return false;
        }
    }
    return true;
}

bool TrafficManager::IsLinkAvailable(uint32_t _fromNode, uint32_t _toNode, float _enterTime, float _leaveTime, uint32_t _agvID)
{
    int startSlot = GetStartSlot(_enterTime);
    int endSlot = GetEndSlot(_leaveTime);
    
    uint32_t minNode = std::min(_fromNode, _toNode);
    uint32_t maxNode = std::max(_fromNode, _toNode);

    for (int t = startSlot; t <= endSlot; ++t)
    {
        std::string key = std::to_string(minNode) + "_" + std::to_string(maxNode) + "_" + std::to_string(t);
        if (m_LinkReservations.find(key) != m_LinkReservations.end() && m_LinkReservations[key] != _agvID)
        {
            return false;
        }
    }
    return true;
}