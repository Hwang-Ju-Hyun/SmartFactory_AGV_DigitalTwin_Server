#include "Robo.hpp"
#include "header.hpp"
#include <iostream>
#include "Event.hpp"
#include "WarehouseManager.hpp"

Robo::Robo()
{
    SetName("Robo");
    m_ClassID = ClassID::OBJ_AGV;    
    m_Speed = 4.0f;
    m_Progress = 0.f;
}

float Robo::GetTimeSpentOnCurrentLink_FromNode()
{
    auto x = GetPosX(); auto z = GetPosZ();
    float dist = std::sqrt(std::pow(m_FromNode.m_PosX - x, 2) + std::pow(m_FromNode.m_PosZ - z, 2));         
    return dist / m_Speed;
}

float Robo::GetTimeSpendOnCurrentLink_ToNode()
{
    auto x = GetPosX(); auto z = GetPosZ();
    float dist = std::sqrt(std::pow(m_ToNode.m_PosX - x, 2) + std::pow(m_ToNode.m_PosZ - z, 2));         
    return dist / m_Speed;
}


void Robo::AssignNextStep(const MapNode& _from, const MapNode& _to, AGVState _newState, float _startTime, float _arrivalTime)
{std::cout
<< "[Assign]"
<< " AGV "
<< GetNetworkID()
<< " "
<< _from.m_Id
<< " -> "
<< _to.m_Id
<< " state="
<< (int)_newState
<< std::endl;
    m_FromNode = _from;
    m_ToNode = _to;
    m_Progress = 0.0f;
    m_State = _newState;
    
    // 관제탑이 지정해준 출발 및 도착 시간
    m_MoveStartTime = _startTime; 
    m_PlannedTravelTime = _arrivalTime - _startTime; 

    // 안전장치 (동일 시간 에러 방지)
    if (m_PlannedTravelTime <= 0.0f) 
        m_PlannedTravelTime = 0.001f;

    m_AccStayTime = 0.0f; 

    if (_newState == AGVState::IDLE)
    {
        m_bIdleEventSent = false; 
    }

    if (_newState == AGVState::MOVING)
    {
        if (_from.m_Id != _to.m_Id) 
        {
            m_CurrentLink = MapManager::GetInstance().FindLink(_from.m_Id, _to.m_Id);
            std::cout << "[출발] AGV " << GetNetworkID() << " | " << _from.m_Id << "->" << _to.m_Id 
                      << " | 예약출발: " << _startTime << " | 예약도착: " << _arrivalTime << std::endl;
        }
    }
}

void Robo::UpdateNavigation(float _deltaTime, float _serverTime)
{   
    if (m_State == AGVState::WAIT_REPLAN) 
    {
        std::cout
    << "WAIT_REPLAN "
    << GetNetworkID()
    << std::endl;
return;
    }
        
    
    if (m_State == AGVState::IDLE) 
    {
        m_AccStayTime += _deltaTime;
        if (!m_bIdleEventSent || m_AccStayTime > 5.0f) 
        {
            m_bIdleEventSent = true; 
            m_AccStayTime = 0.0f; 
            
            RobotEvent re = { RobotEventType::IDLE_READY, GetNetworkID(), _serverTime };
            EventManager::GetInstance().Publish(re);
        }
        return;
    }

    if (m_State == AGVState::LOADING || m_State == AGVState::UNLOADING)
    {
        m_AccStayTime += _deltaTime;
        if (m_AccStayTime > 1.0f) 
        {
            m_AccStayTime = 0.f;       
            RobotEventType eType = (m_State == AGVState::LOADING) ? RobotEventType::PICKUP_COMPLETED : RobotEventType::DROP_COMPLETED;
            
            m_State = AGVState::IDLE; 
            RobotEvent re = { eType, GetNetworkID(), _serverTime };
            EventManager::GetInstance().Publish(re);
        }
        return;
    }

    if (m_State == AGVState::MOVING)
    {
        std::cout
<< "[Update]"
<< GetNetworkID()
<< " state="
<< (int)m_State
<< " from "
<< m_FromNode.m_Id
<< " to "
<< m_ToNode.m_Id
<< std::endl;
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
            if (m_CurrentLink.m_Type == 1) // 곡선
            {                
                float t = m_Progress;
                float u = 1.0f - t;
                float tt = t * t; float uu = u * u;
                float uuu = uu * u; float ttt = tt * t;

                m_PosX = (uuu * m_FromNode.m_PosX) + (3.0f * uu * t * m_CurrentLink.m_CX1) + 
                         (3.0f * u * tt * m_CurrentLink.m_CX2) + (ttt * m_ToNode.m_PosX);
                m_PosZ = (uuu * m_FromNode.m_PosZ) + (3.0f * uu * t * m_CurrentLink.m_CZ1) + 
                         (3.0f * u * tt * m_CurrentLink.m_CZ2) + (ttt * m_ToNode.m_PosZ);                                     
            }
            else // 직선
            {
                m_PosX = m_FromNode.m_PosX + (m_ToNode.m_PosX - m_FromNode.m_PosX) * m_Progress;
                m_PosZ = m_FromNode.m_PosZ + (m_ToNode.m_PosZ - m_FromNode.m_PosZ) * m_Progress;                    
            }
        }

        // 주행 완료 처리 (도착)
        if (m_Progress >= 1.0f)
        {
            m_Progress = 1.0f;
            
            m_PosX = m_ToNode.m_PosX;
            m_PosZ = m_ToNode.m_PosZ;

            m_State = AGVState::IDLE; 
            m_CurrentNodeID = m_ToNode.m_Id;

            std::cout << "[도착] AGV " << GetNetworkID() << " | " << m_FromNode.m_Id << "->" << m_ToNode.m_Id 
                      << " | 실제출발: " << m_MoveStartTime << " | 실제도착: " << _serverTime << std::endl;
            
            float scheduledArrivalTime = m_MoveStartTime + m_PlannedTravelTime;
            RobotEvent re = { RobotEventType::MOVING_WAITING_COMPLETED, GetNetworkID(), _serverTime };
            EventManager::GetInstance().Publish(re);
        }
    }
}