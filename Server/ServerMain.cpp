#include <cassert>
#include <iostream>
#include "SocketAddressFactory.hpp"
#include "SocketAddress.hpp"
#include "TCPSocket.hpp"
#include "SocketUtil.hpp"
#include "NetworkManagerServer.hpp"
#include "ObjectRegistry.hpp"

#include <algorithm>
#include <chrono>
bool g_LOOP=true;

int main(void)
{    
    SocketAddressPtr serverAddr = SocketAddressFactory::CreateIPv4FromString("0.0.0.0:6666");    
    TCPSocketPtr sockServerTcp=SocketUtil::CreateTCPSocket(AF_INET);
    int option = 1;
    setsockopt(sockServerTcp->GetSocket(), SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));
    assert(sockServerTcp->Bind(*serverAddr)!=ERROR);        

    assert(sockServerTcp->Listen()!=ERROR);

    std::cout<<"Server: Checking for data"<<std::endl<<std::endl;
    
    std::vector<TCPSocketPtr> readBlockSockets;
    readBlockSockets.push_back(sockServerTcp);

    std::vector<TCPSocketPtr> readAbleSockets;     

    ObjectRegistry::sInstance->StaticInit();
    
    NetworkManagerServer::sInstance->StaticInit();
    
    const std::chrono::duration<double> kTickDuration(1.0 / 30.0); 
    auto lastUpdateTime = std::chrono::high_resolution_clock::now();
    while(g_LOOP)
    {
        timeval timeoutValue;
        timeoutValue.tv_sec = 0;
        timeoutValue.tv_usec = 3333; // 33.3ms
        int toRet=SocketUtil::Select(&readBlockSockets,&readAbleSockets,nullptr,nullptr,nullptr,nullptr,&timeoutValue);
        
        if(toRet<0)
            continue;            
        
        std::vector<TCPSocketPtr> newSockets;
        std::vector<TCPSocketPtr> closedSockets;
        if(toRet>0)
        {
            for(TCPSocketPtr& socket:readAbleSockets)
            {
                // socket = 알바생이 "여기 불 켜졌어요" 하고 들고 온 카메라 주소
                // sockServerTcp = 우리가 알고 있는 "정문" 카메라 주소
                if(socket==sockServerTcp)
                {
                    // "어? 알바생이 들고 온 카메라 주소가 우리 '정문' 주소랑 똑같네?"
                    // 정문 문손잡이가 덜컹거렸다는 뜻이니까 ➔ "아! 새로운 손님이 접속했구나!"
                    SocketAddress newClientAddr;
                    TCPSocketPtr newClientSock=sockServerTcp->Accept(newClientAddr);
                    if(newClientSock)
                    {   
                        std::cout<<"New Client Connected : "<<newClientAddr.ToString()<<std::endl;
                        newSockets.push_back(newClientSock);
                        
                        //hello packet을 똑바로 주고 받으면 session id 0이 아니라 갱신이 됨
                        NetworkManagerServer::sInstance->OnClientAccepted(newClientSock);
                        
                        
                    }
                }
                else
                {
                    ClientProxyPtr currentClientPtr=nullptr;                
                    int foundIdx=-1;
                    auto proxies = NetworkManagerServer::sInstance->GetPendingProxies();
                    for(int i=0;i<proxies.size();i++)
                    {                    
                        if(proxies[i]->GetSession()->GetSocket()==socket)
                        {
                            currentClientPtr=proxies[i];
                            foundIdx=proxies[i]->GetSessionID();
                            break;
                        }                    
                    }
                    if(currentClientPtr!=nullptr)
                    {                
                        bool isAlive=currentClientPtr->GetSession()->ProcessIncomingData();

                        if(!isAlive)
                        {
                            std::cout<<"Client disconnected"<<std::endl;
                            closedSockets.push_back(socket);
                        }                    
                    }                
                }        
            }
        }
       
        auto now = std::chrono::high_resolution_clock::now();
        float deltaTime = std::chrono::duration<float>(now - lastUpdateTime).count();
        lastUpdateTime = now;

        if (deltaTime > 0.1f) 
        {
            deltaTime = 0.1f; 
        }

        NetworkManagerServer::sInstance->UpdateWorld(deltaTime);

        NetworkManagerServer::sInstance->SendOutgoingReplicationPackets();
        for(const auto& ns:newSockets)
        {
            readBlockSockets.push_back(ns);
        }
        for(const auto& closedSocket:closedSockets)
        {
            readBlockSockets.erase(
                std::remove(readBlockSockets.begin(), readBlockSockets.end(), closedSocket),
                readBlockSockets.end()
            );
        }
    }
    return 0;
}
