#pragma once
#include "Object.hpp"
#include "TCPSocket.hpp"
#include "ClientProxy.hpp"

class LinkingContext;

class NetworkManagerServer
{
private:
    NetworkManagerServer();
    float m_TotalElapsedServerTime;
public:
    static std::unique_ptr<NetworkManagerServer> sInstance;    
    static void StaticInit();    
public:
    void ProcessPacket(ClientProxy* _cs,InputMemoryStream& _stream);
private:
    void HandleHello_Packet(ClientProxy* _proxy,InputMemoryStream& _instream);
    void SendHello_Packet(ClientProxy* _proxy);
    void SendMap_Packet(ClientProxy* _proxy);    
    void HandleReadyMap_Packet(ClientProxy* _proxy,InputMemoryStream& _instream);
    void HandleReadyObject_Packet(ClientProxy* _proxy,InputMemoryStream& _instream);
private:
    std::vector<ClientProxyPtr> m_PendingProxies;
    // 접속한 클라이언트들을 관리하는 명부 (ID -> 세션 )    
    std::unordered_map<uint32_t,ClientProxy*> m_SessionIdToProxyMap;
    static uint32_t nextSessionID;    
private:
    LinkingContext* m_LinkingContext; 
public: 
    void OnClientAccepted(TCPSocketPtr _tcpSocket);
    std::vector<ClientProxyPtr> GetPendingProxies()const{return m_PendingProxies;}
public:
    LinkingContext* GetLinkingContext()const{return m_LinkingContext;}
    void RegisterObject(ObjectPtr _obj);
    void SendOutgoingReplicationPackets();
private:
    bool m_IsMapReady = false;
    bool m_IsSimulationActive=false;
public:
    void StartSimulation(){m_IsSimulationActive=true;}
    void StopSimulation(){m_IsSimulationActive=false;}
    void UpdateWorld(float _deltaTime);

    //todo 이거 trafficcontrolmanager로 두는게 더 좋아보임
public:
    void ReplanPath(uint32_t _agvID);
    void RequestReplan(uint32_t _agvID);
    float GetTotalElapsedServerTime()const{return m_TotalElapsedServerTime;}
};