#include "UnityRobotController.hpp"

UnityRobotController::UnityRobotController() = default;

UnityRobotController::~UnityRobotController() = default;

void UnityRobotController::FollowRoute(const RoutePacket& routePacket)
{
    // 유니티 로봇에게 경로를 전달하는 로직 구현
}

void UnityRobotController::CancelRoute()
{
    // 유니티 로봇에게 경로 취소를 전달하는 로직 구현
}

StatusPacket UnityRobotController::GetStatus()
{
    // 유니티 로봇의 상태를 가져오는 로직 구현
}

bool UnityRobotController::IsArrived() const
{
    // 유니티 로봇이 목적지에 도착했는지 확인하는 로직 구현
}

void UnityRobotController::Update(float dt)
{
    // 유니티 로봇을 업데이트하는 로직 구현
}
