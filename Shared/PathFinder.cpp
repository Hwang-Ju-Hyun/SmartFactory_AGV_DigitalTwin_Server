#include "PathFinder.hpp"
#include "Map.hpp"
#include "TrafficControlManager.hpp"
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <iostream>

const float AGV_SPEED = 4.0f;
const float WAIT_TIME = 1.0f; 
constexpr float CLEARANCE_TIME = 0.6f;

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
    startNode->departureTime = _startTime; // 시작점은 도착하자마자 출발 대기 상태
    startNode->g = 0.f;
    startNode->h = _rraEngine.GetAbstractDistance(_startNodeID) / AGV_SPEED; 
    startNode->f = startNode->g + startNode->h;

    openList.push(startNode);
    std::string startKey = std::to_string(_startNodeID) + "_" + std::to_string(TrafficManager::GetStartSlot(_startTime));
    openRegistryList[startKey] = startNode;

    std::shared_ptr<AStarNode> endNode = nullptr;

    while (!openList.empty())
    {
        auto current = openList.top();
        openList.pop();

        // 🌟 정밀해진 슬롯 검사: 이 노드에서 출발하는 시각(departureTime)을 기준으로 닫힌 목록 관리
        int currentSlot = TrafficManager::GetStartSlot(current->departureTime);
        std::string currentKey = std::to_string(current->id) + "_" + std::to_string(currentSlot);
        
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
        // 탐색 1: 제자리 대기 (WAIT) 액션
        // ==========================================
        float waitLeaveTime = current->departureTime + WAIT_TIME;
        std::string waitKey = std::to_string(current->id) + "_" + std::to_string(TrafficManager::GetStartSlot(waitLeaveTime));

        if (TrafficManager::GetInstance().IsNodeAvailable(current->id, current->departureTime, waitLeaveTime, _agvID))
        {
            if (closedList.find(waitKey) == closedList.end())
            {
                float nextG = current->g + WAIT_TIME;
                if (openRegistryList.find(waitKey) == openRegistryList.end() || nextG < openRegistryList[waitKey]->g)
                {
                    auto waitNode = std::make_shared<AStarNode>(current->id); 
                    waitNode->parentNode = current;
                    // 🌟 WAIT 액션: 도착은 그대로 두고, 출발 시각만 뒤로 미룹니다!
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
        // 탐색 2: 주변 노드로 이동 (MOVE) 액션
        // ==========================================
        const auto& links = MapManager::GetInstance().GetLinks();
        for (const auto& link : links)
        {
            if (link.m_FromNodeID != current->id || link.m_IsBlocked) continue;

            uint32_t neighborID = link.m_ToNodeID;
            auto fromNodeGeo = MapManager::GetInstance().GetNodes().at(current->id);
            auto toNodeGeo = MapManager::GetInstance().GetNodes().at(neighborID);
            
            float dist = (link.m_Type == 1) ? link.m_Dist : std::sqrt(std::pow(toNodeGeo.m_PosX - fromNodeGeo.m_PosX, 2) + std::pow(toNodeGeo.m_PosZ - fromNodeGeo.m_PosZ, 2));
            float travelTime = std::ceil(dist / AGV_SPEED);

            // 🌟 [핵심 수정] 이전 노드를 '떠난 시각(departureTime)'부터 링크 주행을 시작합니다!
            float enterTime = current->departureTime;
            float leaveTime = enterTime + travelTime;
            std::string neighborKey = std::to_string(neighborID) + "_" + std::to_string(TrafficManager::GetStartSlot(leaveTime));

            if (closedList.find(neighborKey) != closedList.end()) continue;        
            if (!TrafficManager::GetInstance().IsNodeAvailable(current->id, enterTime, enterTime + CLEARANCE_TIME, _agvID)) continue;
            if (!TrafficManager::GetInstance().IsLinkAvailable(current->id, neighborID, enterTime, leaveTime, _agvID)) continue;                
            
            float requiredDwellTime = (neighborID == _targetNodeID) ? (_windowTimeLimit + 2.0f) : 1.0f;
            if (!TrafficManager::GetInstance().IsNodeAvailable(neighborID, leaveTime, leaveTime + requiredDwellTime, _agvID)) continue;

            float nextG = current->g + travelTime;
            if (current->parentNode != nullptr && neighborID == current->parentNode->id) continue; // 유턴 방지

            if (openRegistryList.find(neighborKey) == openRegistryList.end() || nextG < openRegistryList[neighborKey]->g)
            {
                auto neighborNode = std::make_shared<AStarNode>(neighborID); 
                neighborNode->parentNode = current;
                // 🌟 MOVE 액션: 새로운 노드에 도착했으므로 arrival과 departure를 통일시킵니다.
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

    // =========================================================
    // 역추적 페이즈 (Trace): 중복 노드 없이 깔끔하게 압축 맵핑
    // =========================================================
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
                    // 🌟 내부 노드에서 연장된 departureTime을 최종 시간표에 그대로 흡수합니다!
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