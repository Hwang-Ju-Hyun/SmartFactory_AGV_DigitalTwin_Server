#include "Map.hpp"

void MapManager::Init()
{

    auto AddNode = [this](uint32_t id, float x, float z) {
        m_Nodes[id] = MapNode{ id, x, z, 0 };
    };

    uint32_t nextLinkId = 1;
    auto AddLink = [this, &nextLinkId](uint32_t from, uint32_t to) {
        m_Links.push_back(MapLink{ nextLinkId++, from, to });
    };
    
    AddNode(1, -15.0f,  0.0f);   // 1번: 메인 출발 스테이션
    AddNode(7, -15.0f, -10.0f);  // 7번: 좌측 하단 구석
    AddNode(8, -15.0f,  10.0f);  // 8번: 좌측 상단 우회로 진입점

    // 중앙 라인 (센터)
    AddNode(4,  0.0f,   0.0f);   // 4번: 중앙 교차로 허브
    AddNode(10, 0.0f,  -10.0f);  // 10번: 하단 중앙 충전 구역
    AddNode(9,  0.0f,   10.0f);  // 9번: 상단 중앙 교차로

    // 오른쪽 라인 (동쪽)
    AddNode(2,  15.0f,  0.0f);   // 2번: 우측 대기 구역
    AddNode(5,  25.0f,  0.0f);   // 5번: 우측 끝 최종 하역장 탈출구
    AddNode(6,  15.0f, -5.0f);   // 6번: 우측 하단 진입로
    AddNode(3,  15.0f, -10.0f);  // 3번: 우측 하단 구석 스테이션 (테스트 타겟)
    AddNode(11, 15.0f,  10.0f);  // 11번: 우측 상단 구석
    
    AddLink(1, 4); AddLink(1, 7); AddLink(1, 8);
    
    // 4번 노드 (중앙 허브) ➔ 사방 교차로
    AddLink(4, 1); AddLink(4, 2); AddLink(4, 9); AddLink(4, 10);
    
    // 2번 노드 ➔ 우측 분기
    AddLink(2, 4); AddLink(2, 5); AddLink(2, 6);
    
    // 5번 노드 (막다른 곳 탈출구)
    AddLink(5, 2);
    
    // 6번 노드 ➔ 3번으로 가는 길목
    AddLink(6, 2); AddLink(6, 3);
    
    // 3번 노드 (우측 하단 구석)
    AddLink(3, 6); AddLink(3, 10);
    
    // 7번 노드 (좌측 하단 구석)
    AddLink(7, 1); AddLink(7, 10);
    
    // 10번 노드 (하단 중앙)
    AddLink(10, 7); AddLink(10, 4); AddLink(10, 3);
    
    // 8번 노드 (좌측 상단)
    AddLink(8, 1); AddLink(8, 9);
    
    // 9번 노드 (상단 중앙)
    AddLink(9, 8); AddLink(9, 4); AddLink(9, 11);
    
    // 11번 노드 (우측 상단 막다른 우회로)
    AddLink(11, 9);
}