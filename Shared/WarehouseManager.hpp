#pragma once
#include <vector>
#include <cstdint>
#include <memory>
class Warehouse
{
public:
    Warehouse(uint32_t _nodeID,uint32_t _stock,bool _isReq)
    :m_NodeID(_nodeID)
    ,m_Stock(_stock)
    ,m_isReqeusted(_isReq)
    {
        
    }
public:
    uint32_t m_NodeID;
    uint32_t m_Stock;
    bool m_isReqeusted;
};

class WarehouseManager
{
private:
    WarehouseManager(){Init();}
public:
    static WarehouseManager GetInstance()
    {
        static WarehouseManager s;
        return s;
    }
public:
    void Init();    
    std::vector<std::shared_ptr<Warehouse>> m_houses;
    std::vector<std::shared_ptr<Warehouse>> m_dispatcher;
    void Update();
};