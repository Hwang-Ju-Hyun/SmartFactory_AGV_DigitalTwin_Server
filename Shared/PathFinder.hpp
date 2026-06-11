#pragma once
#include <cstdint>
#include "Map.hpp"
#include <cmath>

class AstarPathFinder
{
public:
    struct AStarNode 
    {
        AStarNode(uint32_t _id) : id(_id) {}
        uint32_t id;
        float g = 0.0f; // 출발지부터 여기까지 온 실제 거리
        float h = 0.0f; // 목적지까지 남은 유클리드 예상 거리
        float f = 0.0f; // 총점 (g + h)
        uint32_t parentID = 0; // 경로 역추적을 위한 부모 노드 ID
        float accumulatedTime; //이 노드에 도착했을 때의 시간 축 정보        
    };
    
    struct CompareNode 
    {
        bool operator()(const std::shared_ptr<AStarNode>& _a, const std::shared_ptr<AStarNode>& _b) 
        {
            return _a->f > _b->f; // 오름차순 정렬 (F가 작을수록 우선순위가 높음)
        }
    };

public:
    float CalculateHeuristic(const MapNode& _a,const MapNode& _b)
    {
        return std::sqrt(std::pow(_a.m_PosX-_b.m_PosX,2)+std::pow(_a.m_PosZ-_b.m_PosZ,2));
    }
    std::vector<uint32_t> FindPath(uint32_t _startNodeID, uint32_t _endNodeID, const std::unordered_map<uint32_t,MapNode>& _nodes, const std::vector<MapLink>& _links,uint32_t _avgID,float _startTime);
};