#include "Map.hpp"
#include "Map.hpp"
#include <fstream>
#include <iostream>
#include <vector>
// nlohmann/json 헤더 포함 (json.hpp 파일만 프로젝트 폴더에 넣으면 됨)
#include "json.hpp"
#include <filesystem> 

using json = nlohmann::json;

void MapManager::Init()
{
    std::cout << "[디버그] 현재 실행 작업 디렉터리: " 
              << std::filesystem::current_path() << std::endl;    
    std::ifstream file("../../Shared/MapData.json"); 
    if (!file.is_open())
    {
        std::cerr << "[에러] MapData.json 파일을 열 수 없습니다" << std::endl;
        return;
    }

    json mapJson;
    file >> mapJson; // 파일 스트림을 통해 JSON 객체로 파싱
    file.close();

    // 2. 기존 데이터 초기화
    m_Nodes.clear();
    m_Links.clear();

    // 3. "nodes" 배열 파싱 및 C++ 벡터에 저장  
    if (mapJson.contains("nodes") && mapJson["nodes"].is_array())
    {
        for (const auto& nodeItem : mapJson["nodes"])
        {
            MapNode node;
            node.m_Id = nodeItem["id"].get<uint32_t>();
            node.m_PosX = nodeItem["x"].get<float>();
            node.m_PosZ = nodeItem["z"].get<float>();
            
            m_Nodes.insert({node.m_Id, node});             
        }
    }

    // 4. "links" 배열 파싱 및 C++ 벡터에 저장
    if (mapJson.contains("links") && mapJson["links"].is_array())
    {
        uint32_t linkIdCounter = 1; // C++ 서버 관리용 고유 Link ID 부여
        for (const auto& linkItem : mapJson["links"])
        {
            MapLink link;
            link.m_Id = linkIdCounter++;
            link.m_FromNodeID = linkItem["from"].get<uint32_t>();
            link.m_ToNodeID = linkItem["to"].get<uint32_t>();
            link.m_IsBlocked = false; // 초기 상태는 차단되지 않음

            m_Links.push_back(link);
        }
    }

    std::cout << "맵 데이터 로드 완료 (노드: " << m_Nodes.size() 
              << "개, 링크: " << m_Links.size() << "개)" << std::endl;
}

MapLink& MapManager::FindLink(uint32_t _fromNodeID,uint32_t _toNodeID)
{
    MapLink error;
    error.m_FromNodeID=-1;
    error.m_ToNodeID=-1;
    error.m_Id=-1;
    error.m_IsBlocked=false;
    
    for(int i=0;i<m_Links.size();i++)
    {
        if(m_Links[i].m_FromNodeID==_fromNodeID&&m_Links[i].m_ToNodeID==_toNodeID)
            return m_Links[i];
    }
    return error;
}