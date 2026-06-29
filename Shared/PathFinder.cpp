#include "PathFinder.hpp"
#include "Map.hpp"
#include "TrafficControlManager.hpp"
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <iostream>

const float AGV_SPEED = 4.0f;
const float WAIT_TIME = 1.0f; // 제자리 대기 시간
constexpr float CLEARANCE_TIME=0.6;

std::vector<uint32_t> PathFinder::FindPath(uint32_t _startNodeID, uint32_t _targetNodeID, 
                                           uint32_t _agvID, float _startTime, float _windowTimeLimit,RRAStar& _rraEngine)
{
    std::vector<uint32_t> finalPath;

    if (_startNodeID == _targetNodeID)
    {
        finalPath.push_back(_startNodeID);
        return finalPath;
    }

    std::priority_queue<std::shared_ptr<AStarNode>, std::vector<std::shared_ptr<AStarNode>>, 
        auto(*)(const std::shared_ptr<AStarNode>&, const std::shared_ptr<AStarNode>&)->bool> 
        openList([](const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) { return *a > *b; });
    
    std::unordered_map<std::string, std::shared_ptr<AStarNode>> closedList;
    std::unordered_map<std::string, std::shared_ptr<AStarNode>> openRegistryList;

    std::shared_ptr<AStarNode> startNode = std::make_shared<AStarNode>(_startNodeID);
    startNode->accumulatedTime = _startTime;
    startNode->g = 0.f;
    startNode->h = _rraEngine.GetAbstractDistance(_startNodeID) / AGV_SPEED; 
    startNode->f = startNode->g + startNode->h;

    openList.push(startNode);
    std::string startKey = std::to_string(_startNodeID) + "_" + std::to_string(TrafficManager::TimeToSlot(_startTime));
    openRegistryList[startKey] = startNode;

    std::shared_ptr<AStarNode> endNode = nullptr;

    while (!openList.empty())
    {
        auto current = openList.top();
        openList.pop();

        int currentSlot = TrafficManager::TimeToSlot(current->accumulatedTime);
        std::string currentKey = std::to_string(current->id) + "_" + std::to_string(currentSlot);
        
        // 이미 ClosedList에 있으면(더 짧은 거리로 방문했었다면) 무시! (Lazy Deletion 핵심)
        if (closedList.find(currentKey) != closedList.end()) 
        {
            continue;
        }
        
        closedList[currentKey] = current;

        // 목적지 도달
        if (current->id == _targetNodeID)
        {
            endNode = current;
            break;
        }

        // WHCA* 핵심 1: 윈도우(Window) 컷오프
        int stepsTaken = 0;
        auto temp = current;
        while(temp->parentNode != nullptr) 
        { 
            stepsTaken++; 
            temp = temp->parentNode; 
        }
        
        if (current->accumulatedTime - _startTime >= _windowTimeLimit)
        {
            // 현재 노드까지 오는 데 걸린 시간이 윈도우 시간(예: 15초)을 넘겼으면 탐색 중지!
            endNode = current;
            break;
        }

        // ==========================================
        // 탐색 1: 제자리 대기(WAIT) 액션 
        // ==========================================
        float waitLeaveTime = current->accumulatedTime + WAIT_TIME;
        std::string waitKey = std::to_string(current->id) + "_" + std::to_string(TrafficManager::TimeToSlot(waitLeaveTime));

        if (TrafficManager::GetInstance().IsNodeAvailable(current->id, current->accumulatedTime, waitLeaveTime, _agvID))
        {
            if (closedList.find(waitKey) == closedList.end())
            {
                float nextG = current->g + WAIT_TIME;
                bool isAlreadyOpen = (openRegistryList.find(waitKey) != openRegistryList.end());
                
                if (!isAlreadyOpen || nextG < openRegistryList[waitKey]->g)
                {
                    auto waitNode = std::make_shared<AStarNode>(current->id); // 새 노드 무조건 생성
                    waitNode->parentNode = current;
                    waitNode->accumulatedTime = waitLeaveTime;
                    waitNode->g = nextG;
                    waitNode->h = _rraEngine.GetAbstractDistance(current->id) / AGV_SPEED; 
                    waitNode->f = waitNode->g + waitNode->h;

                    openList.push(waitNode); // 무조건 다시 push!
                    openRegistryList[waitKey] = waitNode; // 레지스트리 갱신
                }
            }
        }

        // ==========================================
        //  탐색 2: 주변 노드로 이동 액션 (수정된 코드 적용)
        // ==========================================
        const auto& links = MapManager::GetInstance().GetLinks();
        for (const auto& link : links)
        {
            uint32_t neighborID = 0;
            if (link.m_FromNodeID == current->id) 
            {
                neighborID = link.m_ToNodeID;
            }                
            // 무방향 맵이라면 아래 줄 주석 해제
            else if (link.m_ToNodeID == current->id) 
            {
                neighborID = link.m_FromNodeID;
            }
            else 
            {
                continue;
            }

            if (link.m_IsBlocked) 
                continue;

            auto from = MapManager::GetInstance().GetNodes().at(current->id);
            auto to = MapManager::GetInstance().GetNodes().at(neighborID);                        
            float dist = 0.f;

            
            if (link.m_Type == 1)
            {                
                // 유니티 스플라인이 계산해서 JSON으로 구워준 '실제 곡선 길이'를 그대로 가져다 씁니다.
                dist = link.m_Dist; 
            }
            else
            {
                // 기존 직선 거리 계산
                dist = std::sqrt(std::pow(to.m_PosX - from.m_PosX, 2) + std::pow(to.m_PosZ - from.m_PosZ, 2));
            }

            //2. 올림(ceil)을 통해 로봇(Robo.cpp)과 관제탑이 소모 틱(Tick)을 동일하게 인지하도록 동기화!
            float travelTime = std::ceil(dist / AGV_SPEED);

            float enterTime = current->accumulatedTime;
            float leaveTime = enterTime + travelTime;
            std::string neighborKey = std::to_string(neighborID) + "_" + std::to_string(TrafficManager::TimeToSlot(leaveTime));

            if (closedList.find(neighborKey) != closedList.end()) 
                continue;        
            if (!TrafficManager::GetInstance().IsNodeAvailable(current->id, enterTime, enterTime + CLEARANCE_TIME, _agvID)) 
            {
                continue;
            }
            if (!TrafficManager::GetInstance().IsLinkAvailable(current->id, neighborID, enterTime, leaveTime, _agvID)) 
            {
                continue;
            }                
            if (!TrafficManager::GetInstance().IsNodeAvailable(neighborID, leaveTime, leaveTime + SLOT_DURATION, _agvID)) 
            {
                continue;
            }

            float nextG = current->g + travelTime;
            
            if (current->parentNode != nullptr && neighborID == current->parentNode->id)
            {
                nextG += 999.0f; // 999초의 가짜 비용을 줘서 절대 선택하지 않게 만듦 (U턴 방지)
            }
            bool isAlreadyOpen = (openRegistryList.find(neighborKey) != openRegistryList.end());

            if (!isAlreadyOpen || nextG < openRegistryList[neighborKey]->g)
            {
                auto neighborNode = std::make_shared<AStarNode>(neighborID); // 새 노드 무조건 생성
                neighborNode->parentNode = current;
                neighborNode->accumulatedTime = leaveTime;
                neighborNode->g = nextG;                
                neighborNode->h = _rraEngine.GetAbstractDistance(neighborID) / AGV_SPEED;; 
                neighborNode->f = neighborNode->g + neighborNode->h;

                openList.push(neighborNode); // 무조건 다시 push!
                openRegistryList[neighborKey] = neighborNode; // 레지스트리 갱신
            }
        }
    }

    if (endNode != nullptr)
    {
        auto trace = endNode;
        while (trace != nullptr)
        {
            finalPath.push_back(trace->id);
            trace = trace->parentNode;
        }
        std::reverse(finalPath.begin(), finalPath.end());
    }

    return finalPath;
}