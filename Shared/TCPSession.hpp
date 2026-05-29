#pragma once
#include "TCPSocket.hpp"
#include <vector>
#include <functional>

class InputMemoryStream;
class OutputMemoryStream;

class ClientProxy;

class TCPSession
{
private:    
    TCPSocketPtr m_Socket;
    std::vector<char> m_ReceiveBuffer;
public:
    std::function<void(InputMemoryStream&)> OnPacketReceived;
public:
    bool ProcessIncomingData();
    void SendPacket(OutputMemoryStream& _inStream);
    TCPSocketPtr GetSocket(){return m_Socket;}    
    void SetSocket(TCPSocketPtr _tcpSocket){m_Socket=_tcpSocket;}
};

using TCPSessionPtr= std::shared_ptr<TCPSession>;