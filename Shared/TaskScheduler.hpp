#pragma once
#include <cstdint>
#include <queue>
#include <vector>

class Robo;


class TaskScheduler
{
private:
    TaskScheduler();
    
public:
    static TaskScheduler& GetInstance()
    {
        static TaskScheduler instance;
        return instance;
    }    
};