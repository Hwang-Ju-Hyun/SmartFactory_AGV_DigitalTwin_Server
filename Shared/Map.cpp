#include "Map.hpp"

void MapManager::Init()
{
    auto AddNode = [this](uint32_t id, float x, float z) {
        m_Nodes[id] = MapNode{ id, x, z, 0 };
    };

    uint32_t nextLinkId = 1;
    auto AddLink = [this, &nextLinkId](uint32_t from, uint32_t to) {
        m_Links.push_back(MapLink{ nextLinkId++, from, to, false }); 
    };

    // ====================================================================
    // 🌟 1. 노드 배치 (총 50개 노드 비정형 레이아웃)
    // ====================================================================
    
    // [Zone A: 1 ~ 10] 좌측 물류 투입 및 스폰/대기 버퍼 구역 (X: -30 ~ -20)
    AddNode(1,  -30.0f,  20.0f); AddNode(2,  -20.0f,  20.0f);
    AddNode(3,  -30.0f,  10.0f); AddNode(4,  -20.0f,  10.0f);
    AddNode(5,  -30.0f,   0.0f); AddNode(6,  -20.0f,   0.0f);
    AddNode(7,  -30.0f, -10.0f); AddNode(8,  -20.0f, -10.0f);
    AddNode(9,  -30.0f, -20.0f); AddNode(10, -20.0f, -20.0f);

    // [Zone B: 11 ~ 25] 중앙 메인 이송 및 다중 교차로 허브 구역 (X: -10 ~ 10)
    AddNode(11, -10.0f,  20.0f); AddNode(12,   0.0f,  20.0f); AddNode(13,  10.0f,  20.0f);
    AddNode(14, -10.0f,  10.0f); AddNode(15,   0.0f,  10.0f); AddNode(16,  10.0f,  10.0f);
    AddNode(17, -10.0f,   0.0f); AddNode(18,   0.0f,   0.0f); AddNode(19,  10.0f,   0.0f); // 18번: 전역 대형 허브
    AddNode(20, -10.0f, -10.0f); AddNode(21,   0.0f, -10.0f); AddNode(22,  10.0f, -10.0f);
    AddNode(23, -10.0f, -20.0f); AddNode(24,   0.0f, -20.0f); AddNode(25,  10.0f, -20.0f);

    // [Zone C: 26 ~ 35] 우측 제조 공정 및 조립 라인 샛길 구역 (X: 20, Z축 오프셋 비정형화)
    AddNode(26,  20.0f,  25.0f); AddNode(27,  20.0f,  15.0f);
    AddNode(28,  20.0f,   5.0f); AddNode(29,  20.0f,  -5.0f);
    AddNode(30,  20.0f, -15.0f); AddNode(31,  20.0f, -25.0f);
    AddNode(32,  25.0f,  10.0f); AddNode(33,  25.0f,   0.0f); // 짱박히는 공정실 노드
    AddNode(34,  25.0f, -10.0f); AddNode(35,  25.0f, -20.0f);

    // [Zone D: 36 ~ 45] 우측 끝 최종 하역장 및 반출구 링 (X: 35)
    AddNode(36,  35.0f,  20.0f); AddNode(37,  35.0f,  10.0f);
    AddNode(38,  35.0f,   0.0f); AddNode(39,  35.0f, -10.0f);
    AddNode(40,  35.0f, -20.0f);
    AddNode(41,  45.0f,  20.0f); AddNode(42,  45.0f,  10.0f); // 외곽 대우회 선로 노드들
    AddNode(43,  45.0f,   0.0f); AddNode(44,  45.0f, -10.0f);
    AddNode(45,  45.0f, -20.0f);

    // [Zone E: 46 ~ 50] 최하단 배터리 충전소 및 비상 정비창 (독립 루프)
    AddNode(46, -15.0f, -30.0f); AddNode(47,  -5.0f, -30.0f); AddNode(48,   5.0f, -30.0f);
    AddNode(49,  15.0f, -30.0f); AddNode(50,  25.0f, -30.0f);


    // ====================================================================
    // 🌟 2. 양방향 링크 연결 (복잡한 우회 및 정체 분산 유도)
    // ====================================================================
    
    // --- Zone A (좌측 스폰/버퍼 단선 및 복선 복합 구조) ---
    AddLink(1, 2); AddLink(2, 1); AddLink(3, 4); AddLink(4, 3);
    AddLink(5, 6); AddLink(6, 5); AddLink(7, 8); AddLink(8, 7);
    AddLink(9, 10); AddLink(10, 9);
    
    AddLink(1, 3); AddLink(3, 1); AddLink(3, 5); AddLink(5, 3);
    AddLink(5, 7); AddLink(7, 5); AddLink(7, 9); AddLink(9, 7);
    AddLink(2, 4); AddLink(4, 2); AddLink(4, 6); AddLink(6, 4);
    AddLink(6, 8); AddLink(8, 6); AddLink(8, 10); AddLink(10, 8);

    // --- Zone A ➔ Zone B 연결 다리 (진입 병목 매커니즘) ---
    AddLink(2, 11); AddLink(11, 2);
    AddLink(6, 17); AddLink(17, 6);
    AddLink(10, 23); AddLink(23, 10);

    // --- Zone B (중앙 그리드 허브 - 다차선 교차로) ---
    // 가로 연결
    AddLink(11, 12); AddLink(12, 11); AddLink(12, 13); AddLink(13, 12);
    AddLink(14, 15); AddLink(15, 14); AddLink(15, 16); AddLink(16, 15);
    AddLink(17, 18); AddLink(18, 17); AddLink(18, 19); AddLink(19, 18); // 18번 메인 교차로
    AddLink(20, 21); AddLink(21, 20); AddLink(21, 22); AddLink(22, 21);
    AddLink(23, 24); AddLink(24, 23); AddLink(24, 25); AddLink(25, 24);
    // 세로 연결
    AddLink(11, 14); AddLink(14, 11); AddLink(14, 17); AddLink(17, 14); AddLink(17, 20); AddLink(20, 17); AddLink(20, 23); AddLink(23, 20);
    AddLink(12, 15); AddLink(15, 12); AddLink(15, 18); AddLink(18, 15); AddLink(18, 21); AddLink(21, 18); AddLink(21, 24); AddLink(24, 21);
    AddLink(13, 16); AddLink(16, 13); AddLink(16, 19); AddLink(19, 16); AddLink(19, 22); AddLink(22, 19); AddLink(22, 25); AddLink(25, 22);

    // --- Zone B ➔ Zone C 연결 ---
    AddLink(13, 26); AddLink(26, 13);
    AddLink(16, 27); AddLink(27, 16);
    AddLink(19, 28); AddLink(28, 19);
    AddLink(22, 30); AddLink(30, 22);
    AddLink(25, 31); AddLink(31, 25);

    // --- Zone C (우측 비정형 공정실 및 복잡한 지그재그 선로) ---
    AddLink(26, 27); AddLink(27, 26); AddLink(27, 28); AddLink(28, 27);
    AddLink(28, 29); AddLink(29, 28); AddLink(29, 30); AddLink(30, 29);
    AddLink(30, 31); AddLink(31, 30);
    // 공정실 독방 연결 (막다른 골목 레이아웃 연출)
    AddLink(27, 32); AddLink(32, 27); AddLink(28, 33); AddLink(33, 28);
    AddLink(30, 34); AddLink(34, 30); AddLink(31, 35); AddLink(35, 31);

    // --- Zone C ➔ Zone D 연결 ---
    AddLink(26, 36); AddLink(36, 26);
    AddLink(32, 37); AddLink(37, 32);
    AddLink(33, 38); AddLink(38, 33);
    AddLink(34, 39); AddLink(39, 34);
    AddLink(35, 40); AddLink(40, 35);

    // --- Zone D (최우측 대우회 외곽 고속 링 코스) ---
    AddLink(36, 37); AddLink(37, 36); AddLink(37, 38); AddLink(38, 37);
    AddLink(38, 39); AddLink(39, 38); AddLink(39, 40); AddLink(40, 39);
    
    AddLink(36, 41); AddLink(41, 36); AddLink(37, 42); AddLink(42, 37);
    AddLink(38, 43); AddLink(43, 38); AddLink(39, 44); AddLink(44, 39);
    AddLink(40, 45); AddLink(45, 40);

    AddLink(41, 42); AddLink(42, 41); AddLink(42, 43); AddLink(43, 42);
    AddLink(43, 44); AddLink(44, 43); AddLink(44, 45); AddLink(45, 44);

    // --- Zone E (최하단 독립 충전 스테이션 루프 선로) ---
    //AddLink(21, 47); AddLink(47, 21); // 중앙에서 충전소 진입
    
    AddLink(24,47); AddLink(47,24);
    AddLink(24,48); AddLink(48,24);
    AddLink(25,49); AddLink(49,25);
    AddLink(23,47); AddLink(47,23);
    AddLink(25,48); AddLink(48,25);
    AddLink(10,46); AddLink(46,10);
    AddLink(29,33); AddLink(33,29);
    AddLink(29,34); AddLink(34,29);
    AddLink(50,35); AddLink(35,50);
    AddLink(14,4); AddLink(4,14);
    AddLink(20,8); AddLink(8,20);

    AddLink(23, 46); AddLink(46, 23);
    AddLink(25, 50); AddLink(50, 25);

    AddLink(46, 47); AddLink(47, 46); AddLink(47, 48); AddLink(48, 47);
    AddLink(48, 49); AddLink(49, 48); AddLink(49, 50); AddLink(50, 49);
}


MapLink MapManager::FindLink(uint32_t _linkID)
{    
    MapLink error;
    error.m_FromNodeID=-1;
    error.m_ToNodeID=-1;
    error.m_Id=-1;
    error.m_IsBloacked=false;

    for(int i=0;i<m_Links.size();i++)
    {
        if(m_Links[i].m_Id == _linkID)
            return m_Links[i];
    }
    return error;   
}

MapLink& MapManager::FindLink(uint32_t _fromNodeID,uint32_t _toNodeID)
{
    MapLink error;
    error.m_FromNodeID=-1;
    error.m_ToNodeID=-1;
    error.m_Id=-1;
    error.m_IsBloacked=false;
    
    for(int i=0;i<m_Links.size();i++)
    {
        if(m_Links[i].m_FromNodeID==_fromNodeID&&m_Links[i].m_ToNodeID==_toNodeID)
            return m_Links[i];
    }
    return error;
}