#pragma once
#include <vector>
#include <unordered_map>
#include <queue>
#include <memory>
#include <cstdint>


struct RRANode 
{
    uint32_t id;
    float g; // 목적지(Goal)로부터 현재 노드까지의 '진짜' 누적 거리
    float h; // 출발지(Start)를 향한 휴리스틱 (방향성 제공)
    float f;

    RRANode(uint32_t _id, float _g, float _h) : id(_id), g(_g), h(_h), f(_g + _h) {}
    
    bool operator>(const RRANode& other) const {return f > other.f;}
};

class RRAStar 
{
public:
    RRAStar();
    ~RRAStar() = default;

    // 목적지와 출발지를 세팅하고 RRA* 엔진을 초기화
    void Init(uint32_t _goalNodeID);

    // WHCA*가 "이 노드에서 목적지까지 얼마나 남았어?" 라고 물어볼 때 호출하는 함수
    float GetAbstractDistance(uint32_t _nodeID);    

private:
    // 휴리스틱용 단순 맨해튼/유클리드 거리 계산
    float CalculateHeuristic(uint32_t _fromNode, uint32_t _toNode);

    uint32_t m_GoalNodeID;
    uint32_t m_StartNodeID;

    // 탐색 대기열 (Open List)
    std::priority_queue<RRANode, std::vector<RRANode>, std::greater<RRANode>> m_OpenList;
    
    // 방문 완료된 노드들의 g값 (진짜 남은 거리) 저장소 (Closed List & Cache)
    std::unordered_map<uint32_t, float> m_ClosedList;
};