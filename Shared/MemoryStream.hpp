#pragma once
#include "header.hpp"
#include <vector>

class Object;

class OutputMemoryStream
{
public:
    OutputMemoryStream();
    ~OutputMemoryStream();
private:
    void ReallocBuffer(uint32_t _inNewLength);
private:
    uint32_t m_Head;
    uint32_t m_Capacity;
    char* m_Buffer;
public:
    const char* GetBuffer()const{return m_Buffer;}
    uint32_t GetLength()const {return m_Head;}    
public:
    void Write(const void* _inData,uint32_t _inByteCounts);    
    void Write(uint32_t _inData){Write(&_inData,sizeof(uint32_t));}
    void Write(int _inData){Write(&_inData,sizeof(int));}
    void Write(size_t _inData){Write(&_inData,sizeof(size_t));}
    void Write(std::vector<int> _inData);
    void Write(uint16_t _inData){Write(&_inData,sizeof(uint16_t));}
    void Write(uint8_t _inData){Write(&_inData,sizeof(uint8_t));}    
};


class InputMemoryStream
{
public:
    InputMemoryStream(char* _inBuffer,uint32_t _inByteCount);
    ~InputMemoryStream();    
private:
    uint32_t m_Head;
    uint32_t m_Capacity;
    char* m_Buffer;
public:
    const char* GetBuffer()const{return m_Buffer;}
    uint32_t GetLength()const {return m_Head;}
public:
    void Read(void* _outData,uint32_t _inByteCounts);
    void Read(uint32_t& _outData){Read(&_outData,sizeof(uint32_t));};
    void Read(int& _outData){Read(&_outData,sizeof(int));}
    void Read(size_t& _outData){Read(&_outData,sizeof(size_t));}
    void Read(uint8_t& _outData){Read(&_outData,sizeof(uint8_t));};
    void Read(uint16_t& _outData){Read(&_outData,sizeof(uint16_t));}
    void Read(std::vector<int> _outData);    
public:
    uint32_t GetRemainDataSize(){return m_Capacity-m_Head;}    
};