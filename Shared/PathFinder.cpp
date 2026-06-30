#include "PathFinder.hpp"
#include "Map.hpp"
#include <queue>
#include "ReservationTable.hpp" 
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <iostream>

const float AGV_SPEED = 4.0f;
const float WAIT_TIME = 1.0f; 
constexpr float CLEARANCE_TIME = 0.6f;

inline std::string GenerateTimeSpaceKey(uint32_t nodeID, float time)
{
    int timeSlot = static_cast<int>(std::round(time * 10.0f)); 
    return std::to_string(nodeID) + "_" + std::to_string(timeSlot);
}

std::vector<PathStep> PathFinder::FindPath(uint32_t _startNodeID, uint32_t _targetNodeID, 
                                           uint32_t _agvID, float _startTime, float _windowTimeLimit, RRAStar& _rraEngine)
{
    std::vector<PathStep> finalPath;

    if (_startNodeID == _targetNodeID)
    {
        finalPath.push_back({_startNodeID, _startTime, _startTime + 18.0f});
        return finalPath;
    }

    std::priority_queue<std::shared_ptr<AStarNode>, std::vector<std::shared_ptr<AStarNode>>, 
        auto(*)(const std::shared_ptr<AStarNode>&, const std::shared_ptr<AStarNode>&)->bool> 
        openList([](const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) { return *a > *b; });
    
    std::unordered_map<std::string, std::shared_ptr<AStarNode>> closedList;
    std::unordered_map<std::string, std::shared_ptr<AStarNode>> openRegistryList;

    auto startNode = std::make_shared<AStarNode>(_startNodeID);
    startNode->arrivalTime = _startTime;
    startNode->departureTime = _startTime; 
    startNode->g = 0.f;
    startNode->h = _rraEngine.GetAbstractDistance(_startNodeID) / AGV_SPEED; 
    startNode->f = startNode->g + startNode->h;

    openList.push(startNode);
    std::string startKey = GenerateTimeSpaceKey(_startNodeID, _startTime);
    openRegistryList[startKey] = startNode;

    std::shared_ptr<AStarNode> endNode = nullptr;

    while (!openList.empty())
    {
        auto current = openList.top();
        openList.pop();

        
        std::string currentKey = GenerateTimeSpaceKey(current->id, current->departureTime);
        
        if (closedList.find(currentKey) != closedList.end()) continue;
        closedList[currentKey] = current;

        if (current->id == _targetNodeID)
        {
            endNode = current;
            break;
        }

        if (current->departureTime - _startTime >= _windowTimeLimit)
        {
            endNode = current;
            break;
        }

        // ==========================================
        // 탐색 1: 제자리 대기 (WAIT)
        // ==========================================
        float waitLeaveTime = current->departureTime + WAIT_TIME;
        std::string waitKey = GenerateTimeSpaceKey(current->id, waitLeaveTime);

        
        if (ReservationTable::GetInstance().IsNodeFree(current->id, current->arrivalTime, waitLeaveTime + CLEARANCE_TIME, _agvID))
        {
            if (closedList.find(waitKey) == closedList.end())
            {
                float nextG = current->g + WAIT_TIME;
                if (openRegistryList.find(waitKey) == openRegistryList.end() || nextG < openRegistryList[waitKey]->g)
                {
                    auto waitNode = std::make_shared<AStarNode>(current->id); 
                    waitNode->parentNode = current;
                    waitNode->arrivalTime = current->arrivalTime;
                    waitNode->departureTime = waitLeaveTime;
                    waitNode->g = nextG;
                    waitNode->h = _rraEngine.GetAbstractDistance(current->id) / AGV_SPEED; 
                    waitNode->f = waitNode->g + waitNode->h;

                    openList.push(waitNode); 
                    openRegistryList[waitKey] = waitNode; 
                }
            }
        }

        // ==========================================
        // 탐색 2: 주변 노드로 이동 (MOVE)
        // ==========================================
        const auto& links = MapManager::GetInstance().GetLinks();
        for (const auto& link : links)
        {
            if (link.m_FromNodeID != current->id || link.m_IsBlocked) continue;

            uint32_t neighborID = link.m_ToNodeID;
            auto fromNodeGeo = MapManager::GetInstance().GetNodes().at(current->id);
            auto toNodeGeo = MapManager::GetInstance().GetNodes().at(neighborID);
            
            float dist = (link.m_Type == 1) ? link.m_Dist : std::sqrt(std::pow(toNodeGeo.m_PosX - fromNodeGeo.m_PosX, 2) + std::pow(toNodeGeo.m_PosZ - fromNodeGeo.m_PosZ, 2));
            
            
            float travelTime = dist / AGV_SPEED;

            float enterTime = current->departureTime;
            float leaveTime = enterTime + travelTime;
            std::string neighborKey = GenerateTimeSpaceKey(neighborID, leaveTime);

            if (closedList.find(neighborKey) != closedList.end()) continue;        
            
            // 🌟 [핵심 변경] 조언자 규칙 통일: 엣지 점유 = [departure, next.arrival] + 마진
            if (!ReservationTable::GetInstance().IsEdgeFree(current->id, neighborID, enterTime, leaveTime + CLEARANCE_TIME, _agvID)) continue;                
            
            float requiredDwellTime = (neighborID == _targetNodeID) ? (_windowTimeLimit + 2.0f) : CLEARANCE_TIME;
            
            // 🌟 [핵심 변경] 조언자 규칙 통일: 다음 노드 점유 = [next.arrival, next.departure]
            if (!ReservationTable::GetInstance().IsNodeFree(neighborID, leaveTime, leaveTime + requiredDwellTime, _agvID)) continue;

            float nextG = current->g + travelTime;
            if (current->parentNode != nullptr && neighborID == current->parentNode->id) continue; 

            if (openRegistryList.find(neighborKey) == openRegistryList.end() || nextG < openRegistryList[neighborKey]->g)
            {
                auto neighborNode = std::make_shared<AStarNode>(neighborID); 
                neighborNode->parentNode = current;
                neighborNode->arrivalTime = leaveTime;
                neighborNode->departureTime = leaveTime; 
                neighborNode->g = nextG;                
                neighborNode->h = _rraEngine.GetAbstractDistance(neighborID) / AGV_SPEED; 
                neighborNode->f = neighborNode->g + neighborNode->h;

                openList.push(neighborNode); 
                openRegistryList[neighborKey] = neighborNode; 
            }
        }
    }

    // Trace 단계는 이전과 동일
    if (endNode != nullptr)
    {
        std::vector<std::shared_ptr<AStarNode>> rawTrace;
        auto trace = endNode;
        while (trace != nullptr)
        {
            rawTrace.push_back(trace);
            trace = trace->parentNode;
        }
        std::reverse(rawTrace.begin(), rawTrace.end()); 

        if (!rawTrace.empty())
        {
            PathStep currentStep;
            currentStep.nodeID = rawTrace[0]->id;
            currentStep.arrivalTime = rawTrace[0]->arrivalTime;
            currentStep.departureTime = rawTrace[0]->departureTime;

            for (size_t i = 1; i < rawTrace.size(); ++i)
            {
                if (rawTrace[i]->id == currentStep.nodeID)
                {
                    currentStep.departureTime = rawTrace[i]->departureTime;
                }
                else
                {
                    finalPath.push_back(currentStep);
                    currentStep.nodeID = rawTrace[i]->id;
                    currentStep.arrivalTime = rawTrace[i]->arrivalTime;
                    currentStep.departureTime = rawTrace[i]->departureTime; 
                }
            }
            finalPath.push_back(currentStep); 
        }
    }
    return finalPath;
}