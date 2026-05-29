#pragma once
#include "Object.hpp"

class Robo:public Object
{       
public:
    Robo();
    virtual ~Robo()override{}    

    virtual uint32_t GetClassID()override{return m_ClassID;}
};