#pragma once
#include <cstdint>
#include "TrafficControlManager.hpp"
#include "AGVManager.hpp"
#include "Robo.hpp"
static int b=0;

class DispatchManager 
{
public:
    static DispatchManager& GetInstance() { static DispatchManager instance; return instance; }
    
    uint32_t FindBestLoadNode(float _serverTime, uint32_t _agvID)
    {
        uint32_t loadNodes[5] = { 1, 3, 5, 7, 9 }; 
        for(int i=0; i<5; i++) 
        {
            if(TrafficControlManager::GetInstance().IsTimeWindowAvailable(loadNodes[i], _serverTime, _serverTime + 3.f, _agvID))
                return loadNodes[i];
        }
        return loadNodes[0];
    }

    // 2. "어디다 물건 내려놓을까?" (하역지 결정)
    uint32_t FindBestDispatchNode(float _serverTime, uint32_t _agvID)
    {        
        uint32_t dispatchNodes[5] = { 46, 47, 48, 49, 50 };
        if(TrafficControlManager::GetInstance().IsTimeWindowAvailable(dispatchNodes[b%5], _serverTime, _serverTime + 5.f, _agvID))                
        {
            b++;
            int c=b%5;
            return dispatchNodes[c];
        }        

    
        for(int i=0; i<5; i++) 
        {
            if(TrafficControlManager::GetInstance().IsTimeWindowAvailable(dispatchNodes[b%5], _serverTime, _serverTime + 5.f, _agvID))                
                return dispatchNodes[b];
        }
        return dispatchNodes[0];
    }

    uint32_t FindHomeNode(float _serverTime,uint32_t _agvID)
    {
        for(auto agv : AGVManager::GetInstance().m_AGVs) 
        {
            if(agv->GetNetworkID()==_agvID)
            {                
                Robo* r = dynamic_cast<Robo*>(agv.get());
                if(r)
                {
                    if(TrafficControlManager::GetInstance().IsTimeWindowAvailable(r->GetHomeNode(), _serverTime, _serverTime + 3.f, _agvID))
                        return r->GetHomeNode();
                }                
            }            
        }
        return -1;
    }
};