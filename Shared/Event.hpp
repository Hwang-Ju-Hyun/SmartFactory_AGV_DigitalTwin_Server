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
    std::queue<RobotEvent> m_CurrentQueue; // 이번 프레임에서 처리할 큐
    std::queue<RobotEvent> m_NextQueue;    // 다음 프레임을 위해 쌓아두는 큐
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
       //  무조건 다음 프레임 큐에 넣습니다! (이번 프레임 실행 절대 불가)
        m_NextQueue.push(_event);  
    }
    void SwapAndProcessEvents()
    {
        // 1. 큐 스왑: 다음 프레임 대기열을 현재 대기열로 가져옵니다.
        std::swap(m_CurrentQueue, m_NextQueue);

        // 2. 가져온 이벤트들만 깔끔하게 처리합니다.
        // 처리 도중 누군가 Publish()를 호출해도 m_NextQueue로 들어가므로 무한 루프가 방지됩니다.
        while(!m_CurrentQueue.empty())
        {
            RobotEvent event = m_CurrentQueue.front();
            m_CurrentQueue.pop();
            auto iter = m_Listeners.find(event.type);
            if(iter != m_Listeners.end())
            {            
                for(const auto& callback : iter->second)
                {
                    callback(event);
                }
            }            
        }       
    }  
};
