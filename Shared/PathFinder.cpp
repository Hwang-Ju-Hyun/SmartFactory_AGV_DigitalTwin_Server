#include "PathFinder.hpp"
#include <vector>
#include <unordered_map>
#include <queue>
#include <memory>
#include <algorithm>
#include "TrafficControlManager.hpp"
#include <iostream>

const float WaitTime=3.f;
const float speed=3.8f;
const float stayTime=1.f;

std::vector<uint32_t> AstarPathFinder::FindPath(uint32_t _startNodeID, uint32_t _endNodeID, const std::unordered_map<uint32_t,MapNode>& _nodes, const std::vector<MapLink>& _links,uint32_t _avgID,float _startTime)
{
    std::vector<uint32_t> finalPath;

    if(_startNodeID==_endNodeID)
        return finalPath;
    
    std::unordered_map<uint32_t,std::vector<MapLink>> adjacencyList;
    
    for(auto link:_links)
    {
        adjacencyList[link.m_FromNodeID].push_back(link);
    }
    std::priority_queue<std::shared_ptr<AStarNode>,std::vector<std::shared_ptr<AStarNode>>,CompareNode> openList;
    std::unordered_map<std::string,std::shared_ptr<AStarNode>> closedList;
    std::unordered_map<std::string,std::shared_ptr<AStarNode>> openRegistryList;    

    std::shared_ptr startAstarNode = std::make_shared<AStarNode>(_startNodeID);
    
    startAstarNode->g=0.f;            
    startAstarNode->h=CalculateHeuristic(_nodes.find(_startNodeID)->second,_nodes.find(_endNodeID)->second);    
    startAstarNode->f=startAstarNode->g+startAstarNode->h;
    startAstarNode->parentNode=nullptr;
    startAstarNode->accumulatedTime=_startTime;

    openList.push(startAstarNode);
    
    int startSlot = static_cast<int>(_startTime*10.f);

    std::string startStateKey=std::to_string(_startNodeID)+"_"+std::to_string(startSlot);    

    openRegistryList[startStateKey]=startAstarNode;

    bool IsFound=false;
    std::shared_ptr<AStarNode> endNode = nullptr;    
    while(!openList.empty())
    {
        std::shared_ptr currentNode=openList.top();
        
        openList.pop();

        uint32_t currentNodeID=currentNode->id;

       int currentTimeSlot = static_cast<int>(currentNode->accumulatedTime * 10.f);

        std::string currentStateKey=std::to_string(currentNodeID)+"_"+std::to_string(currentTimeSlot);
        closedList[currentStateKey]=currentNode;        

        if(currentNodeID == _endNodeID)
        {
            IsFound=true;
            endNode=currentNode;
            break;
        }

        //WAIT
        {
            float expectedEnterTime = currentNode->accumulatedTime;
            float expectedLeaveTime = expectedEnterTime + WaitTime;
            int nextTimeSlot=static_cast<int>(expectedLeaveTime*10.f);

            if(currentNodeID!=_startNodeID)
            {
                if(TrafficControlManager::GetInstance().IsTimeWindowAvailable(currentNodeID,expectedEnterTime,expectedLeaveTime,_avgID))
                {
                    std::string waitState = std::to_string(currentNodeID)+"_"+std::to_string(nextTimeSlot);

                    if(closedList.find(waitState)==closedList.end())
                    {
                        float tentativeG= currentNode->g + WaitTime;
                        bool isAlreadyInOpenList  = openRegistryList.find(waitState) != openRegistryList.end() ? true : false;
                        std::shared_ptr<AStarNode> waitNode = isAlreadyInOpenList? openRegistryList.find(waitState)->second:std::make_shared<AStarNode>(currentNodeID);
                        if(!isAlreadyInOpenList||tentativeG<waitNode->g)
                        {
                            waitNode->parentNode=currentNode;
                            waitNode->h=currentNode->h;
                            waitNode->g=tentativeG;
                            waitNode->f=waitNode->h+waitNode->g;
                            waitNode->accumulatedTime=expectedLeaveTime;
                        }
                    
                    if(!isAlreadyInOpenList)
                    {
                        openList.push(waitNode);
                        openRegistryList.insert({waitState,waitNode});
                    }
                    }
                }
            }            
        }

        for(int i=0; i<adjacencyList[currentNodeID].size(); i++)
        {            
            uint32_t adjacencyNodeID=adjacencyList[currentNodeID][i].m_ToNodeID;            

            // g 계산: 현재까지 온 거리 + 이 링크의 실제 길이        
            float linkLength=CalculateHeuristic(_nodes.find(currentNodeID)->second,_nodes.find(adjacencyNodeID)->second);

            float travelTime=linkLength/speed;
              
            float actualLinkStartTime = currentNode->accumulatedTime;        // 현재 노드 출발 시간
            float actualLinkEndTime   = currentNode->accumulatedTime + travelTime; // 다음 노드 도착 시간


            float expectedEnterTime = actualLinkEndTime; 
            float expectedLeaveTime = expectedEnterTime ;
            
            int nextTimeSlot = static_cast<int>(expectedLeaveTime*10.f);

            std::string adjacencyNodeState=std::to_string(adjacencyNodeID)+"_"+std::to_string(nextTimeSlot);
                        

            //이미 검증이 끝난 노드는 패스
            if(closedList.find(adjacencyNodeState)!=closedList.end())
                continue;

            //링크위의 장애물 건너띄기
            if(adjacencyList[currentNodeID][i].m_IsBloacked)
            {
                continue;                
            }
                
            //노드 검사
            if(!TrafficControlManager::GetInstance().IsTimeWindowAvailable(adjacencyNodeID,expectedEnterTime,expectedLeaveTime,_avgID))
            {
                continue;
            }

            //링크(엣지)검사
            if(!TrafficControlManager::GetInstance().IsLinkAvailable(currentNodeID,adjacencyNodeID,actualLinkStartTime,actualLinkEndTime,_avgID))
            {
                continue;
            }

            float tentativeGn=currentNode->g + linkLength;            
            bool isAlreadyInOpenList = openRegistryList.find(adjacencyNodeState)!=openRegistryList.end()? true:false;                        
                        
            std::shared_ptr<AStarNode> adjacencyNode = isAlreadyInOpenList ? openRegistryList[adjacencyNodeState] : std::make_shared<AStarNode>(adjacencyNodeID);


            //첨봤거나 더 다른 괜찮은(더 짧은) 노드가 있다면
            if(isAlreadyInOpenList==false || tentativeGn < adjacencyNode->g)
            {
                adjacencyNode->parentNode=currentNode;
                adjacencyNode->g = tentativeGn;                
                adjacencyNode->h = CalculateHeuristic(_nodes.find(adjacencyNodeID)->second,_nodes.find(_endNodeID)->second);
                adjacencyNode->f = adjacencyNode->g + adjacencyNode->h;

                adjacencyNode->accumulatedTime=expectedLeaveTime;

                if(isAlreadyInOpenList==false)
                {
                    openList.push(adjacencyNode);
                    openRegistryList[adjacencyNodeState]=adjacencyNode;
                }               
            }
        }
    }

    if(IsFound&&endNode!=nullptr)
    {
        std::shared_ptr<AStarNode> traceNode=endNode;
        float currentTime = endNode->accumulatedTime;
        
        while(traceNode!=nullptr)
        {
            finalPath.push_back(traceNode->id);                        

            traceNode=traceNode->parentNode;           
        }    
        std::reverse(finalPath.begin(),finalPath.end());
    }
    return finalPath;
}