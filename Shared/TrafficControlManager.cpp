#include "TrafficControlManager.hpp"
#include <iostream>

void TrafficManager::ClearFutureReservations(uint32_t _agvID, float _currentTime)
{            
    if (m_AgvOwnedKeys.find(_agvID) == m_AgvOwnedKeys.end()) 
        return;

    // [핵심] 현재 시간이 포함된 시작 슬롯을 안전망으로 잡습니다.
    int safeSlot = GetStartSlot(_currentTime);
    std::vector<std::string> keysToKeep; 

    for (const std::string& key : m_AgvOwnedKeys[_agvID])
    {
        size_t lastUnderscorePos = key.find_last_of('_');
        if (lastUnderscorePos != std::string::npos)
        {
            int slot = std::stoi(key.substr(lastUnderscorePos + 1));
            
            // 안전 슬롯보다 미래인 경우에만 삭제!
            if (slot > safeSlot)
            {
                m_NodeReservations.erase(key);
                m_LinkReservations.erase(key);
            }
            else
            {
                keysToKeep.push_back(key);
            }
        }
    }

    m_AgvOwnedKeys[_agvID].clear();
    m_AgvOwnedKeys[_agvID] = keysToKeep;
    std::cout << "AGV " << _agvID << " ClearReservation " << safeSlot << std::endl;
}

bool TrafficManager::ReserveNode(uint32_t _nodeID, float _enterTime, float _leaveTime, uint32_t _agvID)
{
int startSlot = GetStartSlot(_enterTime);
    int endSlot = GetEndSlot(_leaveTime);
     
    // ==========================================================
    // 1. 검증 페이즈 (Validation): 단 하나라도 충돌나면 바로 false 반환!
    // ==========================================================
    for (int t = startSlot; t < endSlot; ++t) //  [핵심] <= 가 아니라 < 입니다. [start, end)
    {
        std::string key = std::to_string(_nodeID) + "_" + std::to_string(t);
        auto it = m_NodeReservations.find(key);
        if (it != m_NodeReservations.end() && it->second != _agvID)
        {
            std::cerr << "[예약 거부] DOUBLE RESERVE 차단! Node: " << _nodeID 
                      << " | Slot: " << t << " | 주인: AGV " << it->second 
                      << " | 가해자: AGV " << _agvID << std::endl;
            return false; // 반쪽짜리 예약을 막기 위해 즉시 취소!
        }
    }

    // ==========================================================
    // 2. 기록 페이즈 (Commit): 100% 안전할 때만 장부에 씁니다.
    // ==========================================================
    for (int t = startSlot; t < endSlot; ++t) // 여기도 무조건 < 입니다.
    {
        std::string key = std::to_string(_nodeID) + "_" + std::to_string(t);
        m_NodeReservations[key] = _agvID;
        m_AgvOwnedKeys[_agvID].push_back(key);
    }
    
    std::cout << "Reserve " << _nodeID << " " << startSlot << "~" << endSlot << " agv " << _agvID << std::endl;
    return true;
}

bool TrafficManager::ReserveLink(uint32_t _fromNode, uint32_t _toNode, float _enterTime, float _leaveTime, uint32_t _agvID)
{
   int startSlot = GetStartSlot(_enterTime);
    int endSlot = GetEndSlot(_leaveTime);

    uint32_t minNode = std::min(_fromNode, _toNode);
    uint32_t maxNode = std::max(_fromNode, _toNode);

    // 1. 검증 페이즈
    for (int t = startSlot; t < endSlot; ++t) //  <
    {
        std::string key = std::to_string(minNode) + "_" + std::to_string(maxNode) + "_" + std::to_string(t);
        auto it = m_LinkReservations.find(key);
        if (it != m_LinkReservations.end() && it->second != _agvID)
            return false;
    }

    // 2. 기록 페이즈
    for (int t = startSlot; t < endSlot; ++t) //  <
    {
        std::string key = std::to_string(minNode) + "_" + std::to_string(maxNode) + "_" + std::to_string(t);
        m_LinkReservations[key] = _agvID;
        m_AgvOwnedKeys[_agvID].push_back(key);
    }
    return true;
}

bool TrafficManager::IsNodeAvailable(uint32_t _nodeID, float _enterTime, float _leaveTime, uint32_t _agvID)
{
   int startSlot = GetStartSlot(_enterTime);
    int endSlot = GetEndSlot(_leaveTime);

    for (int t = startSlot; t < endSlot; ++t) 
    {
        std::string key = std::to_string(_nodeID) + "_" + std::to_string(t);
        if (m_NodeReservations.find(key) != m_NodeReservations.end() && m_NodeReservations[key] != _agvID)
            return false;
    }
    return true;
}

bool TrafficManager::IsLinkAvailable(uint32_t _fromNode, uint32_t _toNode, float _enterTime, float _leaveTime, uint32_t _agvID)
{
    int startSlot = GetStartSlot(_enterTime);
    int endSlot = GetEndSlot(_leaveTime);
    
    uint32_t minNode = std::min(_fromNode, _toNode);
    uint32_t maxNode = std::max(_fromNode, _toNode);

    for (int t = startSlot; t < endSlot; ++t) 
    {
        std::string key = std::to_string(minNode) + "_" + std::to_string(maxNode) + "_" + std::to_string(t);
        if (m_LinkReservations.find(key) != m_LinkReservations.end() && m_LinkReservations[key] != _agvID)
            return false;
    }
    return true;
}