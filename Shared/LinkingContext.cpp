#include "LinkingContext.hpp"
#include "header.hpp"
#include <assert.h>

uint32_t LinkingContext::m_NextNetworkID=0;

LinkingContext::LinkingContext()    
{
}

LinkingContext::~LinkingContext(){}

uint32_t LinkingContext::GetNetworkID(ObjectPtr _obj)
{
    auto iter=m_ObjectToNetworkId.find(_obj);
    
    if(iter!=m_ObjectToNetworkId.end())
    {
        return iter->second;
    }

    return ERROR;
}

ObjectPtr LinkingContext::GetObject(uint32_t _networkId)
{
    auto iter=m_NetworkIdToObject.find(_networkId);

    if(iter!=m_NetworkIdToObject.end())
    {
        return iter->second;
    }   
    return nullptr; 
}

void LinkingContext::AddObject(ObjectPtr _obj,uint32_t _networkID)
{
    m_NetworkIdToObject[_networkID]=_obj;
    m_ObjectToNetworkId[_obj]=_networkID;
}

void LinkingContext::RemoveObject(ObjectPtr _inObject)
{
    uint32_t networkID=m_ObjectToNetworkId[_inObject];
    m_ObjectToNetworkId.erase(_inObject);
    m_NetworkIdToObject.erase(networkID);    
}