#include "RRAstar.hpp"
#include "Map.hpp" // MapManager, MapNode, MapLink 등이 있다고 가정
#include <cmath>

RRAStar::RRAStar() : m_GoalNodeID(0), m_StartNodeID(0) {}

void RRAStar::Init(uint32_t _goalNodeID)
{
    m_GoalNodeID = _goalNodeID;

    m_OpenList = std::priority_queue<RRANode, std::vector<RRANode>, std::greater<RRANode>>();
    m_ClosedList.clear();

    // 출발지가 없으므로 h(휴리스틱)는 0으로 고정 
    // 이렇게 하면 목적지에서부터 사방으로 퍼져나가는 '다익스트라 캐시'로 작동함
    m_OpenList.push(RRANode(m_GoalNodeID, 0.0f, 0.0f));
}

float RRAStar::GetAbstractDistance(uint32_t _nodeID)
{
    // 1. 이미 예전에 탐색해서 '진짜 거리'를 아는 노드라면? (캐시 히트)
    if (m_ClosedList.find(_nodeID) != m_ClosedList.end())
    {
        return m_ClosedList[_nodeID];
    }

    // 2. 모르는 노드라면? 알 때까지 OpenList를 까면서 탐색을 "재개(Resume)"함
    while (!m_OpenList.empty())
    {
        RRANode current = m_OpenList.top();
        m_OpenList.pop();

        // 이미 Closed에 있다면 (더 짧은 경로로 먼저 방문했다면) 패스
        if (m_ClosedList.find(current.id) != m_ClosedList.end())
            continue;

        // 방문 완료 처리 (목적지로부터 이 노드까지의 진짜 거리를 확정)
        m_ClosedList[current.id] = current.g;

        // 드디어 우리가 원하던 그 노드의 진짜 거리를 알아냈다
        if (current.id == _nodeID)
        {
            return current.g;
        }

        // 주변 이웃 노드 확장 (주의: 일반 A*처럼 로봇이나 예약을 신경 쓰지 않음! 오직 정적 '맵'만 봄)
        const auto& links = MapManager::GetInstance().GetLinks();
        for (const auto& link : links)
        {
            // 역방향 탐색이므로 ToNode에서 FromNode를 찾거나, 양방향 맵이면 연결된 모든 노드를 탐색
            uint32_t neighborID = 0;
            if (link.m_FromNodeID == current.id)
            {
                neighborID = link.m_ToNodeID;
            } 
            else if (link.m_ToNodeID == current.id)
            {
                neighborID = link.m_FromNodeID;
            } 
            else 
            {
                continue; // 현재 노드와 연결되지 않은 링크면 패스
            }

            // 동적으로 맵 자체가 막힌 곳(예: 사고 발생 구역)은 못 간다고 판단
            if (link.m_IsBlocked) 
            {
                continue;
            }

            if (m_ClosedList.find(neighborID) == m_ClosedList.end())
            {
                // 이웃까지의 거리 계산
                float distToNeighbor = CalculateHeuristic(current.id, neighborID);
                float nextG = current.g + distToNeighbor;
                //float nextH = CalculateHeuristic(neighborID, m_StartNodeID); // 출발지를 향해 퍼지도록 유도

                m_OpenList.push(RRANode(neighborID, nextG, 0.f));
            }
        }
    }

    // OpenList를 다 깠는데도 못 찾았다면 물리적으로 아예 길이 끊긴 것임
    return 999999.0f; 
}

float RRAStar::CalculateHeuristic(uint32_t _fromNode, uint32_t _toNode)
{
    // MapManager에서 노드 좌표를 가져와서 유클리드(또는 맨해튼) 거리를 구함
    auto from = MapManager::GetInstance().GetNodes().at(_fromNode);
    auto to = MapManager::GetInstance().GetNodes().at(_toNode);

    // 유클리드 거리 (대각선 이동이 가능하면 유클리드, 상하좌우만 가능하면 맨해튼 사용)
    return std::sqrt(std::pow(from.m_PosX - to.m_PosX, 2) + std::pow(from.m_PosZ - to.m_PosZ, 2));
}