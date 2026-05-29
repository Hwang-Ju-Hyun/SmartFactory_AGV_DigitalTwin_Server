#pragma once
#include <iostream>
#include <cstring>
#include <stdint.h>
#include "header.hpp"
#include <memory>

class Object;
class OutputMemoryStream;
class InputMemoryStream;

class Object
{
public:
    Object();
    virtual ~Object(){}
private:
    int m_posX,m_posY;
    char m_Name[10];
    int m_NetworkID;
protected:
    uint32_t m_ClassID= static_cast<uint32_t>(ClassID::OBJ_DEFAULT);
public:
    void SetPosX(int _posX){m_posX=_posX;}
    void AddPosX(int _addX){m_posX+=_addX;}
    void AddPosY(int _addY){m_posY+=_addY;}
    void SetPosY(int _posY){m_posY=_posY;}
    void SetPos(int _x,int _y){m_posX=_x,m_posY=_y;}
    int GetPosX()const{return m_posX;}
    int GetPosY()const{return m_posY;}
    
    void SetName(char* _name){std::strcpy(m_Name,_name);}
    char* GetName(){return m_Name;}

    void SetNetworkID(int _netID){m_NetworkID=_netID;}
    int GetNetworkID()const{return m_NetworkID;}
    
    virtual uint32_t GetClassID(){return m_ClassID;}
public:
    virtual void Write(OutputMemoryStream& _outStream);
    virtual void Read(InputMemoryStream& _outStream);
};

typedef std::shared_ptr<Object> ObjectPtr;