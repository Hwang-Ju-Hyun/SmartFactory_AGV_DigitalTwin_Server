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
protected:
    float m_PosX,m_PosZ;
    char m_Name[10];
    int m_NetworkID;
    float m_HeadingAngle;
    glm::quat m_Rotation;
protected:
    uint32_t m_ClassID= static_cast<uint32_t>(ClassID::OBJ_DEFAULT);
public:
    void SetPosX(float _posX){m_PosX=_posX;}
    void SetPosZ(float _posZ){m_PosZ=_posZ;}
    void AddPosX(float _addX){m_PosX+=_addX;}
    void AddPosZ(float _addZ){m_PosZ+=_addZ;}
    void SetPos(float _x,float _z){m_PosX=_x,m_PosZ=_z;}

    void SetRotation(glm::quat _rot){m_Rotation=_rot;}
    void SetHeadingAngle(float _angle){m_HeadingAngle=_angle;}
    
    float GetPosX()const{return m_PosX;}
    float GetPosZ()const{return m_PosZ;}
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