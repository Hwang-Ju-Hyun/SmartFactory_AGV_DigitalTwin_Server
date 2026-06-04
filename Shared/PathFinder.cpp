#include "PathFinder.hpp"
#include <vector>
#include <unordered_map>
#include <queue>
#include <memory>
#include <algorithm>

std::vector<uint32_t> AstarPathFinder::FindPath(uint32_t _startNodeID, uint32_t _endNodeID, const std::unordered_map<uint32_t,MapNode>& _nodes, const std::vector<MapLink>& _links)
{
    std::vector<uint32_t> finalPath;

    if(_startNodeID==_endNodeID)
        return finalPath;

    std::unordered_map<uint32_t,std::vector<MapLink>> adjacencyList;
    
    for(auto link:_links)
    {
        adjacencyList[link.m_FromNodeID].push_back(link);
    }
    std::priority_queue<std::shared_ptr<AStarNode>> openList;
    std::unordered_map<uint32_t,std::shared_ptr<AStarNode>> closedList;
    std::unordered_map<uint32_t,std::shared_ptr<AStarNode>> openRegistryList;

    std::shared_ptr startAstarNode = std::make_shared<AStarNode>(_startNodeID);

    startAstarNode->g=0.f;            
    startAstarNode->h=CalculateHeuristic(_nodes.find(_startNodeID)->second,_nodes.find(_endNodeID)->second);    
    startAstarNode->f=startAstarNode->g+startAstarNode->h;

    openList.push(startAstarNode);
    openRegistryList[_startNodeID]=startAstarNode;

    bool IsFound=false;

    std::shared_ptr<AStarNode> endNode = nullptr;

    while(!openList.empty())
    {
        std::shared_ptr currentNode=openList.top();
        openList.pop();

        uint32_t currentID=currentNode->id;

        closedList[currentID]=currentNode;

        if(currentID==_endNodeID)
        {
            IsFound=true;
            endNode=currentNode;
            break;
        }    

        for(int i=0; i<adjacencyList[currentID].size(); i++)
        {            
            uint32_t adjacencyNodeID=adjacencyList[currentID][i].m_ToNodeID;

            //이미 검증이 끝난 노드는 패스
            if(closedList.find(adjacencyNodeID)!=closedList.end())
                continue;

            // 이동 비용(g) 계산: 현재까지 온 거리 + 이 링크의 실제 길이
            
            float linkLength=CalculateHeuristic(_nodes.find(currentID)->second,_nodes.find(adjacencyNodeID)->second);
            float tentativeGn=currentNode->g+linkLength;
            
            bool isAlreadyInOpenList = openRegistryList.find(adjacencyNodeID)!=openRegistryList.end()? true:false;                        
            std::shared_ptr<AStarNode> adjacencyNode = isAlreadyInOpenList ? openRegistryList[adjacencyNodeID] : std::make_shared<AStarNode>(adjacencyNodeID);
                        
            //첨봤거나 더 다른 괜찮은(더 짧은) 노드가 있다면
            if(isAlreadyInOpenList==false || tentativeGn < adjacencyNode->g)
            {
                adjacencyNode->parentID=currentID;
                adjacencyNode->g=tentativeGn;                
                adjacencyNode->h=CalculateHeuristic(_nodes.find(adjacencyNodeID)->second,_nodes.find(_endNodeID)->second);
                adjacencyNode->f=adjacencyNode->g+adjacencyNode->h;

                if(isAlreadyInOpenList==false)
                {
                    openList.push(adjacencyNode);
                    openRegistryList[adjacencyNodeID]=adjacencyNode;
                }               
            }
        }
    }
    if(IsFound&&endNode!=nullptr)
    {
        std::shared_ptr<AStarNode> traceNode=endNode;
        while(traceNode!=nullptr)
        {
            finalPath.push_back(traceNode->id);

            if(traceNode->parentID==0)
                break;                
            traceNode=closedList[traceNode->parentID];
        }    
        std::reverse(finalPath.begin(),finalPath.end());
    }
    return finalPath;
}