#include "MemoryStream.hpp"
#include <iostream>
#include <algorithm>
#include "Map.hpp"

OutputMemoryStream::OutputMemoryStream()

    :m_Buffer(nullptr)
    ,m_Head(0)
    ,m_Capacity(0)
{
    ReallocBuffer(1500);
}

OutputMemoryStream::~OutputMemoryStream()
{
    std::free(m_Buffer);
}

void OutputMemoryStream::ReallocBuffer(uint32_t _inNewLength)
{
    m_Buffer= reinterpret_cast<char*>(std::realloc(m_Buffer,_inNewLength));
    m_Capacity=_inNewLength;
}

void OutputMemoryStream::Write(const void* _inData,uint32_t _inByteCounts)
{
    uint32_t resultHead=m_Head+_inByteCounts;
    if(resultHead>m_Capacity)
    {        
        ReallocBuffer(std::max(m_Capacity*2,resultHead));
    }

    std::memcpy(m_Buffer+m_Head,_inData,_inByteCounts);

    m_Head=resultHead;
}

void OutputMemoryStream::Write(std::vector<int> _inData)
{
    size_t element_Count = _inData.size();
    Write(element_Count);
    for(auto v: _inData)    
        Write(v);    
}

void OutputMemoryStream::Write(const std::vector<Node>& _inData)
{
    uint32_t element_count=static_cast<uint32_t>(_inData.size());
    Write(element_count);
    for(int i=0;i<element_count;i++)
    {
        uint32_t id = _inData[i].m_Id;
        float posX=_inData[i].m_PosX;
        float posY=_inData[i].m_PosY;
        uint8_t type = _inData[i].type;

        Write(id);
        Write(posX);
        Write(posY);
        Write(type);
    }
}

void OutputMemoryStream::Write(const std::vector<Link>& _inData)
{
    uint32_t element_count=static_cast<uint32_t>(_inData.size());
    Write(element_count);
    for(int i=0;i<element_count;i++)
    {
        uint32_t id = _inData[i].m_Id;
        uint32_t FromNodeID =_inData[i].m_FromNodeID;
        uint32_t ToNodeID =_inData[i].m_ToNodeID;        

        Write(id);
        Write(FromNodeID);
        Write(ToNodeID);        
    }
}

InputMemoryStream::InputMemoryStream(char* _inBuffer,uint32_t _inByteCount)
    :m_Buffer(_inBuffer)
    ,m_Capacity(_inByteCount)
    ,m_Head(0)    
{

}

InputMemoryStream::~InputMemoryStream()
{            
}

void InputMemoryStream::Read(void* _outData,uint32_t _inByteCounts)
{
    uint32_t resultHead = m_Head +  _inByteCounts;
    if( resultHead>m_Capacity)
    {
        //todo. . . .
    }

    std::memcpy(_outData,m_Buffer+m_Head,_inByteCounts);

    m_Head=resultHead;
}


void InputMemoryStream::Read(std::vector<int> _outData)
{
    size_t element_Count;
    Read(element_Count);
    _outData.resize(element_Count);
    for(auto& v: _outData)
    {
        Read(v);
    }
}

void InputMemoryStream::Read( std::vector<Node> _outData)
{
    size_t element_Count;
    Read(element_Count);
    _outData.resize(element_Count);
    for(int i=0;i<element_Count;i++)
    {
        Read(_outData[i].m_Id);
        Read(_outData[i].m_PosX);
        Read(_outData[i].m_PosY);
        Read(_outData[i].type);
    }
}

void InputMemoryStream::Read(std::vector<Link> _outData)
{
    size_t element_Count;
    Read(element_Count);
    _outData.resize(element_Count);
    for(int i=0;i<element_Count;i++)
    {
        Read(_outData[i].m_Id);
        Read(_outData[i].m_FromNodeID);
        Read(_outData[i].m_ToNodeID);        
    }
}