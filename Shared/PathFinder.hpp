#pragma once
#include <vector>
#include <memory>
#include <string>
#include <cstdint>
#include "RRAstar.hpp"

// 시공간 A*에서 사용할 노드 구조체
struct AStarNode
{
    uint32_t id;
    float accumulatedTime; // 이 노드에 도착할 예상 시간 (T)
    float g; // 시작점부터 여기까지 온 실제 비용 (시간)
    float h; // 목적지까지의 RRA* 휴리스틱 비용 (시간)
    float f;
    
    std::shared_ptr<AStarNode> parentNode;

    AStarNode(uint32_t _id) 
        : id(_id), accumulatedTime(0.f), g(0.f), h(0.f), f(0.f), parentNode(nullptr) 
    {

    }

    // f값이 작은 게 우선순위 큐의 Top이 되도록 설정
    bool operator>(const AStarNode& other) const {return f > other.f;}
};

class PathFinder 
{
public:
    PathFinder() = default;
    ~PathFinder() = default;

    // WHCA* 메인 경로 탐색 함수
    // _windowSize: 예약 장부를 보며 정밀 탐색할 최대 스텝 수 (논문 권장 16)
    std::vector<uint32_t> FindPath(uint32_t _startNodeID, uint32_t _targetNodeID, uint32_t _agvID, float _startTime, float _windowTimeLimit,RRAStar& _rraEngine);
};