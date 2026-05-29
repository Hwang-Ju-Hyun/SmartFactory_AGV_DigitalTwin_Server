#include "ReplicationManagerServer.hpp"
#include "MemoryStream.hpp"
#include "NetworkManagerServer.hpp"
#include "LinkingContext.hpp"
#include <cassert>

/*
장부(m_Commands)를 쭉 돌면서 "어떤 사물(NetworkID)이 어떤 행동(Action)을 해야 하는지"를 
스트림에 차곡차곡 적는 역할을 합니다.
*/

void ReplicationManagerServer::Write(OutputMemoryStream& _outStream)
{
    if(m_Commands.empty())
    {
        return;
    }
    uint32_t commandCount=m_Commands.size();

    _outStream.Write(commandCount);

    for(auto& com:m_Commands)
    {
        uint32_t networkdID=com.first;        
        ReplicationAction action=com.second;
        uint8_t actionByte=static_cast<uint8_t>(action);
        _outStream.Write(networkdID);
        _outStream.Write(actionByte);

        switch (action)
        {
        case RT_CREATE:
        {
            ObjectPtr obj= NetworkManagerServer::sInstance->GetLinkingContext()->GetObject(networkdID);            
            assert(obj!=nullptr);            
            
            uint32_t classID=obj->GetClassID();
            //uint32_t networkClassID=htonl(classID);
            _outStream.Write(classID);                 
            obj->Write(_outStream);
        }
            break;
        case RT_UPDATE:
        {
            ObjectPtr obj = NetworkManagerServer::sInstance->GetLinkingContext()->GetObject(networkdID);
            assert(obj!=nullptr);                                    
            obj->Write(_outStream);
        }
        break;
        case RT_DESTORY:
        {
            m_ObjectToRemove.push_back(networkdID);
        }
        default:
            break;
        }        
    }
    // 전송이 끝난 명령 처리
    for (auto& pair : m_Commands)
    {
        //교과서 개념 이식: Create 요청을 한 번 보냈으면 다음부턴 Update로 자동 변경!
        if (pair.second == RT_CREATE)
        {
            m_Commands[pair.first] = RT_UPDATE;
        }
    }

    // 삭제가 끝난 객체들은 장부에서 완전히 제거
    for (uint32_t id : m_ObjectToRemove)
    {
        m_Commands.erase(id);
    }
    m_ObjectToRemove.clear();
}