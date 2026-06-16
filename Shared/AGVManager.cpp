#include "AGVManager.hpp"

AGVManager::AGVManager()
{
    
}

Robo* AGVManager::FindAGV(uint32_t _agvID)
{
    Robo* robo=nullptr;
    for(int i=0;i<m_AGVs.size();i++)
    {
        if(m_AGVs[i]->GetNetworkID()==_agvID)
        {
            robo = dynamic_cast<Robo*>(m_AGVs[i].get());
            return robo;
        }
    }
}