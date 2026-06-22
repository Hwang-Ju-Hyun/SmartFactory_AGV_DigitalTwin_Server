#pragma once
#include <unordered_map>
#include <cstdint>

class WarehouseManager
{
public:
    static WarehouseManager& GetInstance() { static WarehouseManager instance; return instance; }

    void Init(); 
    
    // 이 창고에 내가 예약할 수 있는 물건이 남아있는가?
    bool CanReserveStock(uint32_t _nodeID); 
    
    // 로봇이 창고로 출발할 때 물건 하나를 미리 찜(예약)함
    void ReserveStock(uint32_t _nodeID); 

    // 로봇이 도착해서 실제로 물건을 집어 들었을 때 재고 깎기
    void ConsumeStock(uint32_t _nodeID); 

private:
    WarehouseManager() = default;
    
    std::unordered_map<uint32_t, int> m_ActualStock;    
    std::unordered_map<uint32_t, int> m_ReservedStock;  // 로봇들이 가지러 오고 있어서 선점된 재고
};