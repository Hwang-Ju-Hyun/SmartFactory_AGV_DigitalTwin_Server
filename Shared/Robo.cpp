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

    m_AccStayTime = 0.0f; 

    if (_newState == AGVState::IDLE)
    {
        
        m_bIdleEventSent = false; 
    }

    if (_newState == AGVState::MOVING)
    {
        if (_from.m_Id != _to.m_Id) 
        {
            //매 프레임 찾지 말고, 명령받을 때 딱 한 번만 찾아 캐싱
            m_CurrentLink = MapManager::GetInstance().FindLink(_from.m_Id, _to.m_Id);
            
            // 곡선이면 반드시 JSON의 m_Dist를 사용
            float dist = 0.0f;
            if (m_CurrentLink.m_Type == 1) 
            {
                dist = m_CurrentLink.m_Dist; 
            }
            else 
            {
                dist = std::sqrt(std::pow(_to.m_PosX - _from.m_PosX, 2) + std::pow(_to.m_PosZ - _from.m_PosZ, 2));
            }

            //관제탑과 동일한 시간축(ceil) 강제 주입
            m_PlannedTravelTime = std::ceil(dist / m_Speed); 

            if (m_PlannedTravelTime < 1.0f) 
                m_PlannedTravelTime = 1.0f; 
        }
        else 
        {
            m_PlannedTravelTime = 1.0f; 
        }             
    }
}

void Robo::UpdateNavigation(float _deltaTime, float _serverTime)
{   
    if (m_State == AGVState::WAIT_REPLAN) 
        return;
    if (m_State == AGVState::IDLE) 
    {
        m_AccStayTime += _deltaTime;
        if (!m_bIdleEventSent || m_AccStayTime > 5.0f) 
        {
            m_bIdleEventSent = true; 
            m_AccStayTime = 0.0f; // 5초마다 타이머 초기화
            
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
        if (m_Progress >= 1.0f) 
            return; 
        if (_serverTime < m_MoveStartTime) 
            return;

        float elapsedTime = _serverTime - m_MoveStartTime;
        m_Progress = elapsedTime / m_PlannedTravelTime;
            
        if (m_Progress > 1.0f) 
            m_Progress = 1.0f;                

        if (m_FromNode.m_Id != m_ToNode.m_Id)
        {
            if (m_CurrentLink.m_Type == 1) 
            {                
                float t = m_Progress;
                float u = 1.0f - t;
                float tt = t * t; float uu = u * u;
                float uuu = uu * u; float ttt = tt * t;

                m_PosX = (uuu * m_FromNode.m_PosX) + 
                         (3.0f * uu * t * m_CurrentLink.m_CX1) + 
                         (3.0f * u * tt * m_CurrentLink.m_CX2) + 
                         (ttt * m_ToNode.m_PosX);

                m_PosZ = (uuu * m_FromNode.m_PosZ) + 
                         (3.0f * uu * t * m_CurrentLink.m_CZ1) + 
                         (3.0f * u * tt * m_CurrentLink.m_CZ2) + 
                         (ttt * m_ToNode.m_PosZ);                        
            }
            else 
            {
                m_PosX = m_FromNode.m_PosX + (m_ToNode.m_PosX - m_FromNode.m_PosX) * m_Progress;
                m_PosZ = m_FromNode.m_PosZ + (m_ToNode.m_PosZ - m_FromNode.m_PosZ) * m_Progress;                    
            }
        }
    
        if (m_Progress >= 1.0f)
        {
            m_Progress = 1.0f;
            
            m_PosX = m_ToNode.m_PosX;
            m_PosZ = m_ToNode.m_PosZ;

            m_State = AGVState::IDLE; 
            m_CurrentNodeID = m_ToNode.m_Id;
            
            
            float scheduledArrivalTime = m_MoveStartTime + m_PlannedTravelTime;
            RobotEvent re = { RobotEventType::MOVING_WAITING_COMPLETED, GetNetworkID(), scheduledArrivalTime };
            EventManager::GetInstance().Publish(re);
        }
    }
}