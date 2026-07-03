#pragma once
#include <unordered_map>
#include <memory>
#include "IRobotController.hpp"

class RobotManager
{
private:
    // 로봇 ID를 키값으로, 그 로봇을 조종할 리모컨(IRobotController)을 들고 있습니다.
    std::unordered_map<uint32_t, std::unique_ptr<IRobotController>> m_RobotControllers;

    RobotManager() = default;

public:
    static RobotManager& GetInstance()
    {
        static RobotManager instance;
        return instance;
    }
    
    void RegisterRobot(uint32_t agvID, std::unique_ptr<IRobotController> controller)
    {
        m_RobotControllers[agvID] = std::move(controller);
    }
    
    IRobotController* GetRobotController(uint32_t agvID)
    {
        auto it = m_RobotControllers.find(agvID);
        if (it != m_RobotControllers.end())
        {
            return it->second.get();
        }
        return nullptr;
    }
};