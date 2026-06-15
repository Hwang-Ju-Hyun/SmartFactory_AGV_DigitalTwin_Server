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
    Task task;
    std::shared_ptr<Warehouse> wh=nullptr;
    //agv가 가져오는 창고    
    if(ac)
        return;
    for(int i=0;i<m_houses.size();i++)
    {
        if(m_houses[i]->m_Stock!=0&&m_houses[i]->m_isReqeusted==false)
        {                           
            task.m_LoadNodeID=m_houses[i]->m_NodeID;     
            task.m_UnloadNodeID=m_houses[i+5]->m_NodeID;                            
            wh=m_houses[i];
            m_houses[i]->m_isReqeusted=true;
            m_houses[i]->m_Stock--;             
            TaskScheduler::GetInstance().PushTaks(task);                
        }
    }    
    ac=true;
    return;
    
    //하역장
    // for(int i=0;i<m_dispatcher.size();i++)
    // {
    //     if(m_dispatcher[i]->m_isReqeusted==false)
    //     {
    //         task.m_UnloadNodeID=m_dispatcher[i]->m_NodeID;            
    //         m_dispatcher[i]->m_isReqeusted=true;
    //         break;
    //     }
    // }    
            
    if(wh==nullptr)
        return;    

    
}