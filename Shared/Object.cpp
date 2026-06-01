#include "Object.hpp"
#include "MemoryStream.hpp"

Object::Object()
    :m_posX(-1)
    ,m_posY(-1)
    ,m_NetworkID(-1)
{
    SetName("None");
}

void Object::Write(OutputMemoryStream& _outStream)
{        
    _outStream.Write(GetPosX());
    _outStream.Write(GetPosY());
    _outStream.Write(m_Rotation.x);
    _outStream.Write(m_Rotation.y);
    _outStream.Write(m_Rotation.z);
    _outStream.Write(m_Rotation.w);
}

void Object::Read(InputMemoryStream& _outStream)
{    
    _outStream.Read(m_posX);
    _outStream.Read(m_posY);
    _outStream.Read(m_Rotation.x);
    _outStream.Read(m_Rotation.y);
    _outStream.Read(m_Rotation.z);
    _outStream.Read(m_Rotation.w);
}