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
    VisionObservationStore.cpp
    VisionObservationRelay.cpp
    PhysicalFleetCorrection.cpp
    MotorFaultDiagnostic.cpp
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

add_executable(VisionObservationTest
    VisionObservationTest.cpp
    VisionObservationStore.cpp
    VisionObservationRelay.cpp
)

target_link_libraries(VisionObservationTest PRIVATE Shared)

add_executable(PhysicalFleetCorrectionTest
    PhysicalFleetCorrectionTest.cpp
    PhysicalFleetCorrection.cpp
)

add_executable(MotorFaultDiagnosticTest
    MotorFaultDiagnosticTest.cpp
    MotorFaultDiagnostic.cpp
)

target_link_libraries(MotorFaultDiagnosticTest PRIVATE Shared)

if(BUILD_TESTING)
    add_test(NAME trajectory_smoke COMMAND TrajectorySmokeTest)
    add_test(NAME vision_observation COMMAND VisionObservationTest)
    add_test(NAME physical_fleet_correction COMMAND PhysicalFleetCorrectionTest)
    add_test(NAME motor_fault_diagnostic COMMAND MotorFaultDiagnosticTest)
    add_test(
        NAME trajectory_testcase0_line_preview
        COMMAND TrajectoryPreview
            --scale-mm-per-unit 50
            --spacing-mm 50
            --speed-mm-s 80
            --start-heading-rad 0
            --track-width-mm 130
            1 2
    )
    set_tests_properties(
        trajectory_testcase0_line_preview
        PROPERTIES WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
endif()
