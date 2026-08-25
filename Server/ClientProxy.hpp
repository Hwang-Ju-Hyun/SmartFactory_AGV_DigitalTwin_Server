#pragma once
#include <memory>
#include "TCPSession.hpp"
#include "ReplicationManagerServer.hpp"
//네트워크와 완전히 분리된, 오직 서버에만 존재하는 Object 아바타.


class OutputMemoryStream;

enum class ClientProtocolIdentity
{
    Unknown,
    Unity,
    Robot,
    Vision,
    Rejected
};

class ClientProxy
{
private:
    TCPSessionPtr m_TCPSession;
    uint32_t m_SessionID;
    ClientProtocolIdentity m_ProtocolIdentity = ClientProtocolIdentity::Unknown;
    ReplicationManagerServer m_ReplicationManagerServer;    
public:
    ClientProxy(TCPSessionPtr _session,uint32_t _sessionID);
    ~ClientProxy();
    ReplicationManagerServer& GetReplicationManagerServer(){return m_ReplicationManagerServer;}
public:
    void SetSessionID(uint32_t _sessionID){m_SessionID=_sessionID;}
    const uint32_t GetSessionID()const{return m_SessionID;}
    ClientProtocolIdentity GetProtocolIdentity() const { return m_ProtocolIdentity; }
    bool TryBindProtocolIdentity(ClientProtocolIdentity _identity)
    {
        if (_identity == ClientProtocolIdentity::Unknown)
            return false;
        if (m_ProtocolIdentity == ClientProtocolIdentity::Unknown)
        {
            m_ProtocolIdentity = _identity;
            return true;
        }
        return m_ProtocolIdentity == _identity;
    }
    void RejectProtocol() { m_ProtocolIdentity = ClientProtocolIdentity::Rejected; }
    void SendPacket(OutputMemoryStream& _inStream);  
    TCPSessionPtr GetSession(){return m_TCPSession;}

    friend class TCPSession;
};

using ClientProxyPtr = std::shared_ptr<ClientProxy>;
