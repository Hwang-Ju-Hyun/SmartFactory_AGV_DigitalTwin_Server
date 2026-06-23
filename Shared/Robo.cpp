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



void Robo::AssignNextStep(const MapNode& _from, const MapNode& _to, AGVState _newState,float _serverTime)
{
    m_FromNode = _from;
    m_ToNode = _to;
    m_Progress = 0.0f;
    m_State = _newState;
    
    std::cout
    << "[ASSIGN] "
    << _from.m_Id
    << " -> "
    << _to.m_Id
    << " start="
    << _serverTime
    << std::endl;

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
        
        //저 고수의 조언대로 시작 로그 찍기!
        std::cout << "[STEP START] AGV " << GetNetworkID() 
                  << " | from: " << _from.m_Id 
                  << " | to: " << _to.m_Id 
                  << " | time: " << m_MoveStartTime << std::endl;
    }
}

void Robo::UpdateNavigation(float _deltaTime, float _serverTime)
{    
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
            m_Progress += _deltaTime / 1.0f; // 제자리 대기
        }
        else 
        {
            float dist = std::sqrt(std::pow(m_ToNode.m_PosX - m_FromNode.m_PosX, 2) + std::pow(m_ToNode.m_PosZ - m_FromNode.m_PosZ, 2));
            
            // Zero Division 방지 및 초단거리 텔레포트 차단
            if (dist < 0.001f) 
            {
                // 두 노드가 물리적으로 같은 위치라면 즉시 완료 처리
                m_Progress = 1.0f; 
            }
            else
            {
                m_Progress += (m_Speed / dist) * _deltaTime;
            }

            m_posX = m_FromNode.m_PosX + (m_ToNode.m_PosX - m_FromNode.m_PosX) * m_Progress;
            m_posZ = m_FromNode.m_PosZ + (m_ToNode.m_PosZ - m_FromNode.m_PosZ) * m_Progress;                    
        }

        // 주행 완료 시 관제탑에 보고
        if (m_Progress >= 1.0f)
        {
            m_Progress = 1.0f;
            
            // [상태 변경] 보고하기 전에 먼저 상태를 바꿔버려서, 같은 프레임에서 중복 이벤트가 발생하는 것을 원천 차단!
            m_State = AGVState::IDLE; 
            
            float actualTravelTime = _serverTime - m_MoveStartTime;
            float diff = actualTravelTime - m_PlannedTravelTime;
            
            std::cout << "[STEP END] AGV " << GetNetworkID() 
                      << " | from: " << m_FromNode.m_Id 
                      << " | to: " << m_ToNode.m_Id 
                      << " | start: " << m_MoveStartTime 
                      << " | end: " << _serverTime 
                      << " | actual: " << actualTravelTime 
                      << " | diff: " << diff << std::endl;
            
            RobotEvent re = { RobotEventType::MOVING_WAITING_COMPLETED, GetNetworkID(), _serverTime };
            EventManager::GetInstance().Publish(re);
        }
    }
}