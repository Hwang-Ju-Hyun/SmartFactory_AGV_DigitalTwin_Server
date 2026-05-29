#pragma once

#include "Robo.hpp"
#include "Object.hpp"
#include "NetworkManagerServer.hpp"

class RoboServer : public Robo
{
public:
    RoboServer(){}
    ~RoboServer()override;
public:
    static ObjectPtr StaticCreate(){return std::make_shared<RoboServer>();}
};