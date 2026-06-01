#pragma once
#include <iostream>
#include <cstring>
#include <stdint.h>
#include "header.hpp"
#include <memory>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class Object;
class OutputMemoryStream;
class InputMemoryStream;

class Object
{
public:
    Object();
    virtual ~Object(){}
private:
    float m_posX,m_posY;
    char m_Name[10];
    int m_NetworkID;
    float m_HeadingAngle;
    glm::quat m_Rotation;
protected:
    uint32_t m_ClassID= static_cast<uint32_t>(ClassID::OBJ_DEFAULT);
public:
    void SetPosX(float _posX){m_posX=_posX;}
    void SetPosY(float _posY){m_posY=_posY;}
    void AddPosX(float _addX){m_posX+=_addX;}
    void AddPosY(float _addY){m_posY+=_addY;}    
    void SetPos(float _x,float _y){m_posX=_x,m_posY=_y;}

    void SetRotation(glm::quat _rot){m_Rotation=_rot;}
    void SetHeadingAngle(float _angle){m_HeadingAngle=_angle;}

    
    float GetPosX()const{return m_posX;}
    float GetPosY()const{return m_posY;}
    glm::quat GetRotation()const{return m_Rotation;}
    float GetHeadingAngle()const{return m_HeadingAngle;}

    
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