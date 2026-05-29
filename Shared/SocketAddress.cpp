#include "SocketAddress.hpp"

SocketAddress::SocketAddress(uint32_t _inAddress,uint16_t _inPort)
{
    GetAsSockAddrIn()->sin_family=AF_INET;
    GetAsSockAddrIn()->sin_addr.s_addr=htonl(_inAddress);
    GetAsSockAddrIn()->sin_port=htons(_inPort);
}

SocketAddress::SocketAddress(const sockaddr& _sockAddr)
{
    m_sockAddr=_sockAddr;
}

SocketAddress::SocketAddress()
{
    
}

SocketAddress::~SocketAddress()
{

}

std::string SocketAddress::ToString()
{
    const sockaddr_in* sock=GetAsSockAddrIn();
    char dstBuffer[128];
    inet_ntop(sock->sin_family,&sock->sin_addr,dstBuffer,sizeof(dstBuffer));

    std::string result=dstBuffer;
    return result;
}