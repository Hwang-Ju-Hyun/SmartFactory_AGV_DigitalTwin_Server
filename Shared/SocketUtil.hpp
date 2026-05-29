#pragma once
#include "header.hpp"
#include "TCPSocket.hpp"
#include <vector>

class SocketUtil
{
public:   
    static TCPSocketPtr CreateTCPSocket(int _vIP)
    {
        int s= socket(_vIP,SOCK_STREAM,IPPROTO_TCP);        
        return TCPSocketPtr(new TCPSocket(s));
    }
public:
    static int Select(const std::vector<TCPSocketPtr>* _inReadSet,
						std::vector<TCPSocketPtr>* _outReadSet,
						const std::vector<TCPSocketPtr>* _inWriteSet,
						std::vector<TCPSocketPtr>* _outWriteSet,
						const std::vector<TCPSocketPtr>* _inExceptSet,
						std::vector<TCPSocketPtr>* _outExceptSet,
                        struct timeval* _timeOut);
private:
    static fd_set* FileSetFromVector (fd_set& _outSet,const std::vector<TCPSocketPtr>* _inTCPSockets,int& _ioNaxNfds);
    static void FileVectorFromSet (std::vector<TCPSocketPtr>*_outSockets,const std::vector<TCPSocketPtr>* _inSockets,const fd_set& _inSet);
};