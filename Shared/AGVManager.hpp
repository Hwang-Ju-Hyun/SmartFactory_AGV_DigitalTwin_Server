#pragma once
#include <vector>
#include "Object.hpp"

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
};