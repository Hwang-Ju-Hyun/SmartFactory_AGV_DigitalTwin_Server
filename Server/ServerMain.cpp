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
#include <exception>
#include <limits>
#include <string>
#include <string_view>
#include <utility>
bool g_LOOP=true;

namespace
{
    const char* RunModeName(ServerRunMode runMode)
    {
        switch (runMode)
        {
        case ServerRunMode::AutomaticFleet:
            return "AUTOMATIC_FLEET";
        case ServerRunMode::PhysicalFleet:
            return "PHYSICAL_FLEET";
        case ServerRunMode::PhysicalDemo:
            return "PHYSICAL_DEMO";
        case ServerRunMode::TrajectoryPreview:
            return "TRAJECTORY_PREVIEW";
        case ServerRunMode::TrajectoryRaisedWheel:
            return "TRAJECTORY_RAISED_WHEEL";
        }
        return "UNKNOWN";
    }

    bool IsValidVisionIdentity(std::string_view identity)
    {
        if (identity.empty() ||
            identity.size() > RobotProtocol::kMaxVisionIdentityBytes)
        {
            return false;
        }
        return std::all_of(identity.begin(), identity.end(), [](char value)
        {
            const unsigned char character = static_cast<unsigned char>(value);
            return character >= 0x21 && character <= 0x7e;
        });
    }

    bool TryParsePositiveUInt32(std::string_view text, uint32_t& outValue)
    {
        try
        {
            std::size_t consumed = 0;
            const unsigned long parsed = std::stoul(std::string(text), &consumed, 10);
            if (consumed != text.size() || parsed == 0 ||
                parsed > std::numeric_limits<uint32_t>::max())
            {
                return false;
            }
            outValue = static_cast<uint32_t>(parsed);
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    bool TryParseOptions(int argc, char** argv, ServerRunMode& outRunMode,
                         std::string& outListenAddress,
                         VisionObservationServerConfig& outVisionConfig,
                         bool& outHelpRequested)
    {
        outRunMode = ServerRunMode::AutomaticFleet;
        outListenAddress = "0.0.0.0:6666";
        outVisionConfig = {};
        outHelpRequested = false;
        bool explicitModeSelected = false;

        for (int i = 1; i < argc; ++i)
        {
            const std::string_view argument(argv[i]);
            if (argument == "--physical-demo")
            {
                if (explicitModeSelected)
                {
                    std::cerr << "Only one server run mode may be selected\n";
                    return false;
                }
                outRunMode = ServerRunMode::PhysicalDemo;
                explicitModeSelected = true;
            }
            else if (argument == "--physical-fleet")
            {
                if (explicitModeSelected)
                {
                    std::cerr << "Only one server run mode may be selected\n";
                    return false;
                }
                outRunMode = ServerRunMode::PhysicalFleet;
                explicitModeSelected = true;
            }
            else if (argument == "--trajectory-preview")
            {
                if (explicitModeSelected)
                {
                    std::cerr << "Only one server run mode may be selected\n";
                    return false;
                }
                outRunMode = ServerRunMode::TrajectoryPreview;
                explicitModeSelected = true;
            }
            else if (argument == "--trajectory-raised-wheel")
            {
                if (explicitModeSelected)
                {
                    std::cerr << "Only one server run mode may be selected\n";
                    return false;
                }
                outRunMode = ServerRunMode::TrajectoryRaisedWheel;
                explicitModeSelected = true;
            }
            else if (argument == "--help")
            {
                outHelpRequested = true;
                std::cout << "Usage: AGV_Server [--physical-fleet | --physical-demo | --trajectory-preview | --trajectory-raised-wheel]"
                             " [--listen ADDRESS:PORT] [--vision-observation --vision-calibration-id ID]"
                             " [--vision-source-id ID]\n"
                          << "  no option             Run the TestCase0 automatic simulated world\n"
                          << "  --physical-fleet      Run the real TestCase0 LINE fleet with physical AGV 1\n"
                          << "  --physical-demo       Run one AGV and issue only logical route [1 -> 2]\n"
                          << "  --trajectory-preview  Send motor-locked preview [1 -> 4] only to a preview-capable robot\n"
                          << "  --trajectory-raised-wheel  Send executable 80 mm/s [1 -> 4] only to a command-capable robot\n"
                          << "  --listen ADDRESS:PORT Override the default 0.0.0.0:6666 listener\n"
                          << "  --vision-observation Enable observation-only VisionTracker input (default OFF)\n"
                          << "  --vision-calibration-id ID Require this locked calibration identity\n"
                          << "  --vision-source-id ID Require this non-zero source identity (default 1)\n";
                return false;
            }
            else if (argument == "--listen")
            {
                if (i + 1 >= argc)
                {
                    std::cerr << "--listen requires ADDRESS:PORT\n";
                    return false;
                }
                outListenAddress = argv[++i];
            }
            else if (argument == "--vision-observation")
            {
                outVisionConfig.enabled = true;
            }
            else if (argument == "--vision-calibration-id")
            {
                if (i + 1 >= argc)
                {
                    std::cerr << "--vision-calibration-id requires ID\n";
                    return false;
                }
                outVisionConfig.expectedCalibrationID = argv[++i];
            }
            else if (argument == "--vision-source-id")
            {
                if (i + 1 >= argc ||
                    !TryParsePositiveUInt32(argv[++i], outVisionConfig.expectedSourceID))
                {
                    std::cerr << "--vision-source-id requires a non-zero uint32 value\n";
                    return false;
                }
            }
            else
            {
                std::cerr << "Unknown option: " << argument << "\n"
                          << "Usage: AGV_Server [--physical-fleet | --physical-demo | --trajectory-preview | --trajectory-raised-wheel]"
                             " [--listen ADDRESS:PORT] [--vision-observation --vision-calibration-id ID]"
                             " [--vision-source-id ID]\n";
                return false;
            }
        }

        if (outVisionConfig.enabled &&
            !IsValidVisionIdentity(outVisionConfig.expectedCalibrationID))
        {
            std::cerr << "--vision-observation requires a 1-64 character visible-ASCII"
                         " --vision-calibration-id\n";
            return false;
        }
        if (!outVisionConfig.enabled &&
            !outVisionConfig.expectedCalibrationID.empty())
        {
            std::cerr << "--vision-calibration-id requires --vision-observation\n";
            return false;
        }

        return true;
    }
}

int main(int argc, char** argv)
{
    ServerRunMode runMode;
    std::string listenAddress;
    VisionObservationServerConfig visionConfig;
    bool helpRequested = false;
    if (!TryParseOptions(
            argc, argv, runMode, listenAddress, visionConfig, helpRequested))
        return helpRequested ? 0 : 2;

    std::cout << "[Server] mode=" << RunModeName(runMode)
              << " listen=" << listenAddress
              << " vision=" << (visionConfig.enabled ? "OBSERVATION_ONLY" : "OFF")
              << "\n";

    SocketAddressPtr serverAddr = SocketAddressFactory::CreateIPv4FromString(listenAddress);
    if (!serverAddr)
    {
        std::cerr << "Invalid listen address: " << listenAddress << "\n";
        return 2;
    }
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
    
    try
    {
        NetworkManagerServer::sInstance->StaticInit(
            runMode, std::move(visionConfig));
    }
    catch (const std::exception& error)
    {
        std::cerr << "Server initialization failed: " << error.what() << "\n";
        return 2;
    }
    
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
                            NetworkManagerServer::sInstance->OnClientDisconnected(
                                currentClientPtr.get());
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
