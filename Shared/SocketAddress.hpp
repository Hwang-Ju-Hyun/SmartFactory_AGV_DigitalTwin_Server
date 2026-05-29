#pragma once
#include "header.hpp"
#include <memory>
#include <string>

class SocketAddress
{
public:
	SocketAddress(uint32_t _inAddress, uint16_t _inPort);
	SocketAddress();
	SocketAddress(const sockaddr& _inSockAddr);
	~SocketAddress();
private:
	sockaddr m_sockAddr;
public:	
	sockaddr_in* GetAsSockAddrIn() { return reinterpret_cast<sockaddr_in*>(&m_sockAddr); }
	std::string ToString();
	int GetSize() const { return sizeof(sockaddr); }
private:
	friend class UDPSocket;
	friend class TCPSocket;
};

using SocketAddressPtr = std::shared_ptr<SocketAddress>;