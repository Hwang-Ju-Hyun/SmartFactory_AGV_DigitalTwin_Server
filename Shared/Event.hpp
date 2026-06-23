#pragma once
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>
#include <queue>


// 로봇이 시스템에 알릴 상태 이벤트들
enum class RobotEventType 
{
    IDLE_READY,         // 대기소 도착 후 완전히 쉬고 있음 (새 임무 받을 준비 완료)
    PICKUP_COMPLETED,   // 상차(Load) 완료
    DROP_COMPLETED,      // 하차(Unload) 완료
    MOVING_WAITING_COMPLETED
};

struct RobotEvent 
{
    RobotEventType type;
    uint32_t agvID;
    float timestamp;
};

using EventCallback=std::function<void(const RobotEvent& _eve)>;

class EventManager
{
private:
    EventManager(){}
    std::unordered_map<RobotEventType,std::vector<EventCallback>> m_Listeners;
    std::queue<RobotEvent> m_EventQueue;
public:
    static EventManager& GetInstance()
    {
        static EventManager sInstance;
        return sInstance;
    }
    
    void Subscribe(RobotEventType _type, EventCallback _callback)
    {   
        m_Listeners[_type].push_back(_callback);        
    }

    void Publish(const RobotEvent& _event)
    {
        m_EventQueue.push(_event);        
    }
    void ProcessEvents()
    {
        while(!m_EventQueue.empty())
        {
            RobotEvent event = m_EventQueue.front();
            m_EventQueue.pop();
            auto iter = m_Listeners.find(event.type);
            if(iter != m_Listeners.end())
            {            
                for(const auto& callback:iter->second)
                {
                    callback(event);
                }
            }            
        }
    }
};
