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

add_executable(TrajectorySmokeTest
    TrajectorySmokeTest.cpp
)

target_link_libraries(TrajectorySmokeTest PRIVATE Shared)

add_executable(TrajectoryPreview
    TrajectoryPreview.cpp
)

target_link_libraries(TrajectoryPreview PRIVATE Shared)

if(BUILD_TESTING)
    add_test(NAME trajectory_smoke COMMAND TrajectorySmokeTest)
    add_test(
        NAME trajectory_testcase03_bezier_preview
        COMMAND TrajectoryPreview
            --scale-mm-per-unit 25
            --spacing-mm 20
            --speed-mm-s 80
            --start-heading-rad 3.14159265359
            --track-width-mm 130
            --require-bezier
            1 4
    )
    set_tests_properties(
        trajectory_testcase03_bezier_preview
        PROPERTIES WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
endif()
