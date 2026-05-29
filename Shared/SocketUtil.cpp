#include "SocketUtil.hpp"

fd_set* SocketUtil::FileSetFromVector(fd_set& _outSet,const std::vector<TCPSocketPtr>* _inTCPSockets,int& _ioNaxNfds)
{
    if(_inTCPSockets)
    {
        FD_ZERO(&_outSet);
        

        for(const auto& tcp_ptr: *_inTCPSockets)
        {
            FD_SET(tcp_ptr->m_Socket,&_outSet);
            if(tcp_ptr->m_Socket>_ioNaxNfds)
            {
                _ioNaxNfds = tcp_ptr->m_Socket;
            }
        }
        return &_outSet;
    }
    return nullptr;
}

void SocketUtil::FileVectorFromSet (std::vector<TCPSocketPtr>*_outSockets,const std::vector<TCPSocketPtr>* _inSockets,const fd_set& _inSet)
{
    if(_outSockets&&_inSockets)
    {
        _outSockets->clear();
        for(auto tcp_socket:*_inSockets)
        {
            if(FD_ISSET(tcp_socket->m_Socket,&_inSet))
            {
                _outSockets->push_back(tcp_socket);
            }
        }
    }
}

int SocketUtil::Select(const std::vector<TCPSocketPtr>* _inReadSet,std::vector<TCPSocketPtr>* _outReadSet,const std::vector<TCPSocketPtr>* _inWriteSet,std::vector<TCPSocketPtr>* _outWriteSet,const std::vector<TCPSocketPtr>* _inExceptSet,std::vector<TCPSocketPtr>* _outExceptSet,struct timeval* _timeOut)
{
    fd_set read,write,except;
    
    int nfds=0;
    
    fd_set* readPtr = FileSetFromVector(read,_inReadSet,nfds);
    fd_set* writePtr = FileSetFromVector(write,_inWriteSet,nfds);
    fd_set* exceptPtr = FileSetFromVector(except,_inExceptSet,nfds);

    int to_Ret = select(nfds+1,readPtr,writePtr,exceptPtr,_timeOut);

    if(to_Ret>0)
    {
        FileVectorFromSet(_outReadSet,_inReadSet,read);
        FileVectorFromSet(_outWriteSet,_inWriteSet,write);
        FileVectorFromSet(_outExceptSet,_inExceptSet,except);
    }
    return to_Ret;
}