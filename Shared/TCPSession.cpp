#include "TCPSession.hpp"
#include "MemoryStream.hpp"

bool TCPSession::ProcessIncomingData()
{
    char buffer[1500];    
    int ReadBytesCount = m_Socket->Receive(buffer,sizeof(buffer));
    if(ReadBytesCount<=0)
        return false;
        
    // 2. 읽어온 데이터를 수신 버퍼 맨 뒤에 이어 붙입니다.
    m_ReceiveBuffer.insert(m_ReceiveBuffer.end(),buffer,buffer+ReadBytesCount);
    while(true)
    {        
        // 조건 A: '전체 길이'를 나타내는 헤더(2바이트)조차 아직 안 왔다면 대기
        if(m_ReceiveBuffer.size()<sizeof(uint16_t))
        {
            break;
        }

        // 버퍼 맨 앞 2바이트를 읽어 이 패킷의 총 길이를 파악합니다.
        uint16_t packetSize = *reinterpret_cast<uint16_t*>(m_ReceiveBuffer.data());

        // 조건 B: 파악한 길이보다 버퍼에 쌓인 데이터가 적다면 대기 (아직 덜 옴)
        if (m_ReceiveBuffer.size() < packetSize) 
        {
            break; 
        }

        //이제 페이로드
        uint32_t payloadSize=packetSize-sizeof(uint16_t);
        
        char* payLoadStart = m_ReceiveBuffer.data()+sizeof(uint16_t);
        
        InputMemoryStream inputStream(payLoadStart,payloadSize);

        //todo :
        //1. 첫 데이터를 UnityPacketType 또는 RobotProtocol::PacketID로 읽어냅니다.
        //함수포인터 델리게이터 형식으로 한 이유는 지금 tcpsession이 clientproxy를 참조를 금하기위해서        
        if(OnPacketReceived)
        {
            OnPacketReceived(inputStream);
        }

        m_ReceiveBuffer.erase(m_ReceiveBuffer.begin(),m_ReceiveBuffer.begin()+packetSize);
    }
    return true;
}

void TCPSession::SendPacket(OutputMemoryStream& _inStream)
{
    uint16_t total_size = static_cast<uint16_t>(sizeof(uint16_t))+_inStream.GetLength();
    OutputMemoryStream finalStream;

    finalStream.Write(total_size);
    finalStream.Write(_inStream.GetBuffer(),_inStream.GetLength());
    m_Socket->Send(finalStream.GetBuffer(),finalStream.GetLength());
}
