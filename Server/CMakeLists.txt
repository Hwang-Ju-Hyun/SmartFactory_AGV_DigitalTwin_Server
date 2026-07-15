# 실행 파일 이름 정의 및 소스 등록
add_executable(AGV_Server
    ServerMain.cpp
    NetworkManagerServer.cpp
    ClientProxy.cpp
    RoboServer.cpp
    ReplicationManagerServer.cpp 
    RoutePlanner.cpp       
    TaskManager.cpp
    RobotSession.cpp
    ESP32RobotController.cpp
    UnityRobotController.cpp
)

#  정적 라이브러리(SharedLib)를 이 실행 파일에 조립(Link)하기!
target_link_libraries(AGV_Server PRIVATE Shared)

add_executable(FakeRobot
    FakeRobotMain.cpp
    FakeRobotApp.cpp
)

target_link_libraries(FakeRobot PRIVATE Shared)
