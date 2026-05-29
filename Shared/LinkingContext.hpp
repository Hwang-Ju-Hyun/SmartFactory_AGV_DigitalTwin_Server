#pragma once
#include <stdint.h>
#include <unordered_map>
#include "Object.hpp"

class Object;

class LinkingContext
{
public:
    LinkingContext();
    ~LinkingContext();
private:
    static uint32_t m_NextNetworkID;
    std::unordered_map<uint32_t,ObjectPtr> m_NetworkIdToObject;
    std::unordered_map<ObjectPtr,uint32_t> m_ObjectToNetworkId;
public:
    uint32_t GetNetworkID(ObjectPtr _obj);
    ObjectPtr GetObject(uint32_t _networkId);
    void AddObject(ObjectPtr _obj,uint32_t _networkID);
    void RemoveObject(ObjectPtr _inObject);    
    uint32_t GenerateNewNextNeworkID(){return ++m_NextNetworkID;}
    std::unordered_map<uint32_t,ObjectPtr> GetAllObjects(){return m_NetworkIdToObject;}
};