#include "ObjectRegistry.hpp"
#include <cassert>

std::unique_ptr<ObjectRegistry> ObjectRegistry::sInstance=nullptr;

void ObjectRegistry::StaticInit()
{
    sInstance.reset(new ObjectRegistry());
}

void ObjectRegistry::RegisterCreationFunction(uint32_t _inClassName,ObjectCreationFunc _inCreationFunction)
{
    assert(m_NameToObjectCreationFuncMap.find(_inClassName)==m_NameToObjectCreationFuncMap.end());

    m_NameToObjectCreationFuncMap[_inClassName]=_inCreationFunction;
}

ObjectPtr ObjectRegistry::CreateObject(uint32_t _inClassName)
{
    // 1. 맵에서 먼저 찾습니다.
    auto it = m_NameToObjectCreationFuncMap.find(_inClassName);
    
    // 2. 만약 못 찾았다면?
    if (it == m_NameToObjectCreationFuncMap.end())
    {
        // 경고 메시지를 띄우고 깔끔하게 nullptr을 반환하여 뻗는 걸 방지합니다.
        std::cout << "[에러] 등록되지 않은 ClassID를 생성하려고 합니다!" << std::endl;
        return nullptr;
    }

    // 3. 찾았다면 안전하게 함수 포인터를 꺼내서 실행합니다.
    ObjectCreationFunc createFunc = it->second;
    ObjectPtr obj = createFunc();

    return obj;
}