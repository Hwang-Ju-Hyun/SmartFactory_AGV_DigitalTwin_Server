#pragma once
#include <unordered_map>
#include <memory>
#include "IRobotController.hpp"
#include "ReservationTable.hpp"

class RobotManager
{
private:
    std::unordered_map<uint32_t, std::unique_ptr<IRobotController>> m_RobotControllers;
    RobotManager() = default;

public:
    static RobotManager& GetInstance() { static RobotManager instance; return instance; }
    
    void RegisterRobot(uint32_t agvID, std::unique_ptr<IRobotController> controller)
    {
        // Atomic 실행 허가 및 점유 획득 콜백 바인딩
        controller->SetTryOccupyEdgeCallback([agvID](uint32_t from, uint32_t to, float serverTime, float expectedArrival) {
            return ReservationTable::GetInstance().TryOccupyEdgeForExecution(agvID, from, to, serverTime, expectedArrival);
        });

        // 꼬리가 빠져나갔을 때 실시간 노드 점유 해제 콜백 바인딩
        controller->SetNodeLeaveCallback([agvID](uint32_t fromNode) {
            OccupancyProvider::GetInstance().LeaveNode(agvID, fromNode);
        });

        controller->SetCanEnterNodeCallback([agvID](uint32_t nodeID) {
            auto& occ = OccupancyProvider::GetInstance();
            if (occ.IsNodeOccupiedByOther(nodeID, agvID))
                return false;

            occ.OccupyNode(agvID, nodeID);
            return true;
        });

        controller->SetIsNodeFreeCallback([agvID](uint32_t nodeID) {
            return !OccupancyProvider::GetInstance().IsNodeOccupiedByOther(nodeID, agvID);
        });

        m_RobotControllers[agvID] = std::move(controller);
    }
    
    IRobotController* GetRobotController(uint32_t agvID)
    {
        auto it = m_RobotControllers.find(agvID);
        if (it != m_RobotControllers.end()) return it->second.get();
        return nullptr;
    }

    void Update(float _dt, float serverTime)
    {
        for(auto& pair : m_RobotControllers) {
            pair.second->Update(_dt, serverTime);
        }
    }

    const std::unordered_map<uint32_t, std::unique_ptr<IRobotController>>& GetRobotControllers() const
    {
        return m_RobotControllers;
    }
};
