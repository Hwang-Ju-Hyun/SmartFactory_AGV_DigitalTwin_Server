#include "TCPSocket.hpp"
#include "header.hpp"
#include "SocketAddress.hpp"
#include <iostream>

TCPSocket::TCPSocket(SOCKET _socket) 
    :m_Socket(_socket)
{
}

int TCPSocket::Bind(const SocketAddress& _inAddress)
{
    int result = bind(m_Socket,&_inAddress.m_sockAddr,_inAddress.GetSize());
    if(result==-1)
    {        
        std::cout<<"Bind Error : "<<strerror(errno)<<std::endl;
        return ERROR;
    }
    return result;
}

/*
서버가 사용합니다.
소켓을 "대기 상태"로 만듭니다. inBackLog는 동시에 전화를 걸어온 클라이언트들을 잠시 세워둘 대기석의 크기입니다.
*/

int TCPSocket::Listen(int _inBackLog)
{
    int result = listen(m_Socket,_inBackLog);
    if(result==-1)
    {        
        std::cout<<"Listen Error : "<<strerror(errno)<<std::endl;
        return ERROR;
    }
    return result;
}

/*
클라이언트가 사용합니다.
서버의 주소로 전화를 거는 행위입니다. 내부적으로 3-Way Handshake라는 과정을 거쳐 연결을 확정합니다.
*/
int TCPSocket::Connect(const SocketAddress& _inAddress)
{
    int result = connect(m_Socket,&_inAddress.m_sockAddr,_inAddress.GetSize());
    if(result==-1)
    {        
        std::cout<<"Connect Error : "<<strerror(errno)<<std::endl;
        return ERROR;
    }
    return result;
}

/*
서버가 사용합니다.
대기 중인 클라이언트의 전화를 받는 행위입니다.
중요: 이 함수는 연결된 클라이언트와 통신할 새로운 TCPSocket을 리턴합니다. 기존의 리스닝 소켓은 계속 다른 전화를 기다리고, 실제 대화는 새로 만들어진 소켓으로 진행합니다.
*/

TCPSocketPtr TCPSocket::Accept(SocketAddress& _fromAddress)
{
    socklen_t length = _fromAddress.GetSize();
    int newSocket = accept(m_Socket,&_fromAddress.m_sockAddr,&length);        

    TCPSocketPtr ns = TCPSocketPtr(new TCPSocket(newSocket));

    return ns;
}


/*
Send / Receive
UDP의 SendTo / ReceiveFrom과 달리 주소 인자가 없습니다.
이미 연결이 확정된 상태이므로, "누구에게" 보낼지 매번 적을 필요가 없기 때문입니다.그냥 빨대에 데이터를 밀어 넣으면(Send) 반대편에서 뽑아 쓰는(Receive) 구조입니다.* /
*/

int TCPSocket::Send(const void* _inData,size_t _inLen)
{
    int SendBytes=send(m_Socket,static_cast<const char*>(_inData),_inLen,0);
    if(SendBytes<0)
    {
        std::cout<<"Semd Error : "<<strerror(errno)<<std::endl;
        return ERROR;
    }        
    return SendBytes;
}

int TCPSocket::Receive(void* _outData,size_t _inLen)
{
    int ReadBytesCount=recv(m_Socket,static_cast<char*>(_outData),_inLen,0);
    if( ReadBytesCount<0)
    {
        std::cout<<"Read Error : "<<strerror(errno)<<std::endl;
        return ERROR;
    }        
    return  ReadBytesCount;
}

bool TCPSocket::SetNonBlockingMode(bool _shouldBeNonBlocking)
{
    int flags = fcntl(m_Socket, F_GETFL, 0);
    if (flags == -1) return false;

    if (_shouldBeNonBlocking)
    {
        flags |= O_NONBLOCK; // 논블로킹 켜기
    }
    else
    {
        flags &= ~O_NONBLOCK; // 논블로킹 끄기
    }

    int result = fcntl(m_Socket, F_SETFL, flags);
    return (result != -1);
 }