#pragma once
#include <unordered_map>
#include <memory>
#include "Object.hpp"

typedef ObjectPtr ( *ObjectCreationFunc )();

class ObjectRegistry
{
private:
    ObjectRegistry(){}
    std::unordered_map<uint32_t,ObjectCreationFunc> m_NameToObjectCreationFuncMap;
public:
    static std::unique_ptr<ObjectRegistry> sInstance;
    static void StaticInit();    
    void RegisterCreationFunction(uint32_t _inClassName,ObjectCreationFunc _inCreationFunction);

    // template<typename T>
    // void RegisterCreationFunction()
    // {
    //     assert(m_NameToObjectCreationFuncMap.find(T::kClassID)!=m_NameToObjectCreationFuncMap.end());
    //     m_NameToObjectCreationFuncMap[T::kClassID]=T::CreateInstance;
    // }
    ObjectPtr CreateObject(uint32_t _inClassName);
};