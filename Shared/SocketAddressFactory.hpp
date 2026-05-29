#pragma once
#include "SocketAddress.hpp"
#include <string>

class SocketAddressFactory
{
public:
    static SocketAddressPtr CreateIPv4FromString(std::string _inString)
    {
        size_t pos = _inString.find(':');        
        std::string host,port;
        if(pos!=std::string::npos)
        {
            host = _inString.substr(0,pos);
            port = _inString.substr(pos+1);                        
        }
        addrinfo* result=nullptr;
        addrinfo* init_result;                        
        
        int error = getaddrinfo(host.c_str(),port.c_str(),nullptr,&result);
        init_result=result;

        if(error!=0 &&result!=nullptr)
        {
            freeaddrinfo(init_result);
            return nullptr;
        }

        while(!result->ai_addr&&result->ai_next)
        {
            result=result->ai_next;
        }    

        if(result->ai_addr==nullptr)
        {
            freeaddrinfo(init_result);
            return nullptr;
        }
        SocketAddressPtr toRet=std::make_shared<SocketAddress>(*result->ai_addr);
        freeaddrinfo(init_result);
        return toRet;
    }
};