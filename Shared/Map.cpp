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
    file >> mapJson;
    file.close();

    m_Nodes.clear();
    m_Links.clear();

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

    if (mapJson.contains("links") && mapJson["links"].is_array())
    {
        uint32_t linkIdCounter = 1; 
        for (const auto& linkItem : mapJson["links"])
        {
            MapLink link;
            link.m_Id = linkIdCounter++;
            link.m_FromNodeID = linkItem["from"].get<uint32_t>();
            link.m_ToNodeID = linkItem["to"].get<uint32_t>();
            link.m_IsBlocked = false; 
            
            // 1. 새로 추가된 데이터 파싱
            link.m_Type = linkItem.value("type", 0);
            link.m_Dist = linkItem.value("dist", 0.0f); // 유니티가 계산해준 실제 길이

            if(link.m_Type == 1)
            {
                // 2. 3차 베지어 제어점 2개 모두 파싱
                link.m_CX1 = linkItem.value("cx1", 0.0f);
                link.m_CZ1 = linkItem.value("cz1", 0.0f);
                link.m_CX2 = linkItem.value("cx2", 0.0f);
                link.m_CZ2 = linkItem.value("cz2", 0.0f);
            }
            else
            {
                link.m_CX1 = 0.0f; link.m_CZ1 = 0.0f;
                link.m_CX2 = 0.0f; link.m_CZ2 = 0.0f;
            }

            m_Links.push_back(link);
        }
    }

    std::cout << "맵 데이터 로드 완료 (노드: " << m_Nodes.size() 
              << "개, 링크: " << m_Links.size() << "개)" << std::endl;
}

MapLink& MapManager::FindLink(uint32_t _fromNodeID,uint32_t _toNodeID)
{
    // static을 붙여 프로그램이 끝날 때까지 메모리에 안전하게 살아있게 만듭니다.
    static MapLink error_link; 
    error_link.m_FromNodeID = -1;
    error_link.m_ToNodeID = -1;
    error_link.m_Id = -1;
    error_link.m_IsBlocked = false;
    error_link.m_Type = 255;
    
    for(int i = 0; i < m_Links.size(); i++)
    {
        if(m_Links[i].m_FromNodeID == _fromNodeID && m_Links[i].m_ToNodeID == _toNodeID)
            return m_Links[i];
    }
    
    return error_link; // 이제 안전하게 참조를 반환합니다.
}