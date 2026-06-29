#include "Robo.hpp"
#include "header.hpp"
#include <iostream>
#include "Event.hpp"
#include "WarehouseManager.hpp"

Robo::Robo()
{
    SetName("Robo");
    m_ClassID=ClassID::OBJ_AGV;    
    m_Speed=4.0f;
    m_Progress=0.f;
}

float Robo::GetTimeSpentOnCurrentLink_FromNode()
{
    auto x = GetPosX();
    auto z =GetPosZ();
    float dist=std::sqrt(std::pow(m_FromNode.m_PosX-x,2)+std::pow(m_FromNode.m_PosZ-z,2));         
    float travelTime = dist/m_Speed;
    return travelTime;
}

float Robo::GetTimeSpendOnCurrentLink_ToNode()
{
    auto x = GetPosX();
    auto z =GetPosZ();
    float dist=std::sqrt(std::pow(m_ToNode.m_PosX-x,2)+std::pow(m_ToNode.m_PosZ-z,2));         
    float travelTime = dist/m_Speed;
    return travelTime;
}



void Robo::AssignNextStep(const MapNode& _from, const MapNode& _to, AGVState _newState,float _serverTime)
{
    m_FromNode = _from;
    m_ToNode = _to;
    m_Progress = 0.0f;
    m_State = _newState;
    //2. 명령을 받는 즉시 절대 덮어씌워지지 않는 시작 시간 세팅!
    m_MoveStartTime = _serverTime; 

    if (_newState == AGVState::MOVING)
    {
        if (_from.m_Id != _to.m_Id) // 진짜 이동일 경우
        {
            float dist = std::sqrt(std::pow(_to.m_PosX - _from.m_PosX, 2) + std::pow(_to.m_PosZ - _from.m_PosZ, 2));
            m_PlannedTravelTime = dist / m_Speed;            
        }
        else // 제자리 대기일 경우
        {
            m_PlannedTravelTime = 1.0f; // WAIT_TIME
        }                
    }
}

void Robo::UpdateNavigation(float _deltaTime, float _serverTime)
{   
    if (m_State == AGVState::IDLE) 
    {
        m_AccStayTime += _deltaTime;
        if (m_AccStayTime > 0.1f) 
        {
            m_AccStayTime = 0.f; // 타이머 초기화
            RobotEvent re = { RobotEventType::IDLE_READY, GetNetworkID(), _serverTime };
            EventManager::GetInstance().Publish(re);
        }
        return;
    }

    // 1. 작업(상/하차) 상태 처리
    if (m_State == AGVState::LOADING || m_State == AGVState::UNLOADING)
    {
        m_AccStayTime += _deltaTime;
        if (m_AccStayTime > 1.0f) // m_StayTime 대신 상수 1.0f 등 사용 (설정에 맞게)
        {
            m_AccStayTime = 0.f;       
            
            RobotEventType eType = (m_State == AGVState::LOADING) ? RobotEventType::PICKUP_COMPLETED : RobotEventType::DROP_COMPLETED;
            
            //  작업이 끝났으니 일단 백수(IDLE)로 대기 (관제탑이 새 명령을 줄 때까지)
            m_State = AGVState::IDLE; 
            
            RobotEvent re = { eType, GetNetworkID(), _serverTime };
            EventManager::GetInstance().Publish(re);
        }
        return;
    }

    if (m_State == AGVState::MOVING)
    {
        //  이미 이번 프레임에서 도착 처리가 끝났다면 더 이상 업데이트하지 않음
        if (m_Progress >= 1.0f) 
            return; 

        if (m_FromNode.m_Id == m_ToNode.m_Id)
        {
            float elapsedTime = _serverTime - m_MoveStartTime;
            m_Progress = elapsedTime / m_PlannedTravelTime;
        }
        else 
        {
            float dist = std::sqrt(std::pow(m_ToNode.m_PosX - m_FromNode.m_PosX, 2) + std::pow(m_ToNode.m_PosZ - m_FromNode.m_PosZ, 2));
                        
            if (dist < 0.001f) 
            {
                m_Progress = 1.0f; 
            }
            else
            {             
               m_Progress += _deltaTime / m_PlannedTravelTime;
            
               if (m_Progress > 1.0f) 
               {
                    m_Progress = 1.0f;
               }                

                // 현재 내가 달리고 있는 링크 정보 찾기 (최적화를 위해 미리 캐싱해두면 더 좋습니다)
                MapLink currentLink; 
                bool isCurve = false;
                for (const auto& l : MapManager::GetInstance().GetLinks()) 
                {
                    if ((l.m_FromNodeID == m_FromNode.m_Id && l.m_ToNodeID == m_ToNode.m_Id) || 
                        (l.m_FromNodeID == m_ToNode.m_Id && l.m_ToNodeID == m_FromNode.m_Id)) 
                    {
                        currentLink = l;
                        if (l.m_Type == 1) 
                            isCurve = true;
                        break;
                    }
                }

                if (isCurve)
                {                
                    float t = m_Progress;
                    float u = 1.0f - t;
                    
                    // 연산 최적화를 위해 거듭제곱 미리 계산
                    float tt = t * t;
                    float uu = u * u;
                    float uuu = uu * u;
                    float ttt = tt * t;

                    // P0 = 출발점 (fromNode)
                    // P1 = 제어점1 (cx1, cz1)
                    // P2 = 제어점2 (cx2, cz2)
                    // P3 = 도착점 (toNode)

                    m_posX = (uuu * m_FromNode.m_PosX) + 
                            (3.0f * uu * t * currentLink.m_CX1) + 
                            (3.0f * u * tt * currentLink.m_CX2) + 
                            (ttt * m_ToNode.m_PosX);

                    m_posZ = (uuu * m_FromNode.m_PosZ) + 
                            (3.0f * uu * t * currentLink.m_CZ1) + 
                            (3.0f * u * tt * currentLink.m_CZ2) + 
                            (ttt * m_ToNode.m_PosZ);                         
                }
                else
                {
                    // 기존 직선(Lerp) 이동
                    m_posX = m_FromNode.m_PosX + (m_ToNode.m_PosX - m_FromNode.m_PosX) * m_Progress;
                    m_posZ = m_FromNode.m_PosZ + (m_ToNode.m_PosZ - m_FromNode.m_PosZ) * m_Progress;                    
                }
            }                  
        }

        // 주행 완료 시 관제탑에 보고
        if (m_Progress >= 1.0f)
        {
            m_Progress = 1.0f;
            
            // [상태 변경] 보고하기 전에 먼저 상태를 바꿔버려서, 같은 프레임에서 중복 이벤트가 발생하는 것을 원천 차단!
            m_State = AGVState::IDLE; 
            
            m_CurrentNodeID = m_ToNode.m_Id;

            float actualTravelTime = _serverTime - m_MoveStartTime;
            float diff = actualTravelTime - m_PlannedTravelTime;              
            
            float scheduledArrivalTime = m_MoveStartTime + m_PlannedTravelTime;
            
            RobotEvent re = { RobotEventType::MOVING_WAITING_COMPLETED, GetNetworkID(), scheduledArrivalTime };
            EventManager::GetInstance().Publish(re);
        }
    }
}