#include "WarehouseManager.hpp"
#include "TaskScheduler.hpp"

static int a=0;
uint32_t storeNodes[5]     = { 41,  42,  43,  29,  45};
uint32_t dispatchNodes[5]  = { 46,  47,  48,  49,  50};

void WarehouseManager::Init()
{    
    m_houses.push_back(std::make_shared<Warehouse>(41,1,false));
    m_houses.push_back(std::make_shared<Warehouse>(42,1,false));
    m_houses.push_back(std::make_shared<Warehouse>(43,1,false));
    m_houses.push_back(std::make_shared<Warehouse>(29,1,false));
    m_houses.push_back(std::make_shared<Warehouse>(45,1,false));

    m_houses.push_back(std::make_shared<Warehouse>(46,0,false));
    m_houses.push_back(std::make_shared<Warehouse>(47,0,false));
    m_houses.push_back(std::make_shared<Warehouse>(48,0,false));
    m_houses.push_back(std::make_shared<Warehouse>(49,0,false));
    m_houses.push_back(std::make_shared<Warehouse>(50,0,false));
}
static bool ac=false;
void WarehouseManager::Update()
{       
            
    if(wh==nullptr)
        return;    

    
}