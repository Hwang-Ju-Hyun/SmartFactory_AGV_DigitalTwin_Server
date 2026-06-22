#include "WarehouseManager.hpp"
#include <iostream>
#include <vector>

void WarehouseManager::Init()
{    
    std::vector<uint32_t> loadNodes = {41, 42, 43, 44, 45};
    
    for (uint32_t nodeID : loadNodes)
    {
        m_ActualStock[nodeID] = 5; 
        m_ReservedStock[nodeID] = 0;
    }
}

bool WarehouseManager::CanReserveStock(uint32_t _nodeID)
{ 
    int availableStock = m_ActualStock[_nodeID] - m_ReservedStock[_nodeID];
    if(availableStock>0)
        return true;
    return false;
    
}

void WarehouseManager::ReserveStock(uint32_t _nodeID)
{
    if (CanReserveStock(_nodeID))
    {
        m_ReservedStock[_nodeID]++;
    }
}

void WarehouseManager::ConsumeStock(uint32_t _nodeID)
{
    if (m_ActualStock[_nodeID] > 0 && m_ReservedStock[_nodeID] > 0)
    {
        m_ActualStock[_nodeID]--;   
        m_ReservedStock[_nodeID]--; 
        std::cout << "[창고] " << _nodeID << "번 창고 재고 감소. 남은 수량: " << m_ActualStock[_nodeID] << std::endl;
    }
}