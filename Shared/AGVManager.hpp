#pragma once
#include <vector>
#include "Object.hpp"
#include "Robo.hpp"

class AGVManager
{
private:
    AGVManager();
public:
    static AGVManager& GetInstance()
    {
        static AGVManager sInstance;
        return sInstance;
    }
public:
    std::vector<ObjectPtr> m_AGVs;    
    Robo* FindAGV(uint32_t _agvID);
};