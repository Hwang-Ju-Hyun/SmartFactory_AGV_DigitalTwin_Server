cmake_minimum_required(VERSION 3.15)
project(AGV_FleetControlSystem LANGUAGES CXX)

include(CTest)

# C++20 표준 설정 (네 강점인 최신 C++ 기능 사용을 위해)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 하위 디렉터리 순서대로 추가 (Shared를 먼저 해야 Server가 링킹할 수 있음)
add_subdirectory(Shared)
add_subdirectory(Server)
add_subdirectory(Client)
