#include "Robo.hpp"
#include "header.hpp"
#include <iostream>
#include "Event.hpp"
#include "WarehouseManager.hpp"

Robo::Robo()
{
    SetName("Robo");
    m_ClassID=ClassID::OBJ_AGV;    
    m_Speed=3.8f;
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





void Robo::AssignNextStep(const MapNode& _from, const MapNode& _to, AGVState _newState)
{
    m_FromNode = _from;
    m_ToNode = _to;
    m_Progress = 0.0f;
    m_State = _newState;
}

void Robo::UpdateNavigation(float _deltaTime, float _serverTime)
{
    int a= this->GetNetworkID();
    int b=0;
    auto c = TrafficControlManager::GetInstance().m_ReservationTable;
    
    if(m_State==AGVState::BLOCKED)
    {
        return;
    }

    if (m_State == AGVState::IDLE) 
    {
        m_AccStayTime += _deltaTime;
        if (m_AccStayTime > 2.0f) 
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
        if (m_AccStayTime > m_StayTime)
        {
            m_AccStayTime = 0.f;       
            
            RobotEventType eType = (m_State == AGVState::LOADING) ? RobotEventType::PICKUP_COMPLETED : RobotEventType::DROP_COMPLETED;
            RobotEvent re = { eType, GetNetworkID(),_serverTime };
            EventManager::GetInstance().Publish(re);
            
            // 작업 끝나면 스케줄러가 새 길 줄 때까지 안전하게 쉼
            m_State = AGVState::WAITING; 
        }
        return;
    }

    // 2. 물리적 이동 혹은 대기 
    if (m_State == AGVState::MOVING || m_State == AGVState::WAITING)
    {
        //A*가 4->4를 줬을 때
        if (m_FromNode.m_Id == m_ToNode.m_Id)
        {
            m_Progress += _deltaTime / WaitTime; // WaitTime(예: 1초) 동안 진행도 오름
        }
        else // 정상 이동
        {
            float dist = std::sqrt(std::pow(m_ToNode.m_PosX - m_FromNode.m_PosX, 2) + std::pow(m_ToNode.m_PosZ - m_FromNode.m_PosZ, 2));
            
            m_Progress += (m_Speed / dist) * _deltaTime;

            m_posX = m_FromNode.m_PosX + (m_ToNode.m_PosX - m_FromNode.m_PosX) * m_Progress;
            m_posZ = m_FromNode.m_PosZ + (m_ToNode.m_PosZ - m_FromNode.m_PosZ) * m_Progress;                    
        }

        // 3. 한 칸(링크) 주행 완료 시 관제탑에 보고
        if (m_Progress >= 1.0f)
        {
            m_Progress = 1.0f;
            m_State = AGVState::WAITING;
            
            RobotEvent re = { RobotEventType::MOVING_WAITING_COMPLETED, GetNetworkID(), _serverTime };
            EventManager::GetInstance().Publish(re);
        }
    }
}