#pragma once

#include <algorithm>
#include <cmath>

namespace AGVKinematics
{
    constexpr float MAX_SPEED = 4.0f;
    constexpr float MAX_ACCEL = 6.0f;
    constexpr float MAX_DECEL = 6.0f;
    constexpr float MAX_ANGULAR_SPEED = 3.14159265f;
    constexpr float MAX_ANGULAR_ACCEL = 6.2831853f;
    constexpr float HEADING_KP = 4.0f;
    constexpr float SLOW_TURN_ANGLE = 0.2617994f;
    constexpr float HARD_TURN_ANGLE = 0.7853982f;
    constexpr float PIVOT_TURN_ANGLE = 1.2217305f;

    inline float EstimateStopToStopTravelTime(float distance)
    {
        if (distance <= 0.001f) 
            return 0.0f;

        const float accelDistance = (MAX_SPEED * MAX_SPEED) / (2.0f * MAX_ACCEL); //최고속도까지 걸리는데 필요한 거리
        const float decelDistance = (MAX_SPEED * MAX_SPEED) / (2.0f * MAX_DECEL); //최고속도에서 완전정지까지 필요한 거리

        if (accelDistance + decelDistance <= distance) 
        {
            const float cruiseDistance = distance - accelDistance - decelDistance;
            return (MAX_SPEED / MAX_ACCEL) + (cruiseDistance / MAX_SPEED) + (MAX_SPEED / MAX_DECEL);
        }

        const float peakVelocity = std::sqrt((2.0f * distance * MAX_ACCEL * MAX_DECEL) / (MAX_ACCEL + MAX_DECEL));
        return (peakVelocity / MAX_ACCEL) + (peakVelocity / MAX_DECEL);
    }

    inline float StopLimitedSpeed(float remainingDistance)
    {
        if (remainingDistance <= 0.0f) return 0.0f;
        return std::sqrt(2.0f * MAX_DECEL * remainingDistance);
    }

    inline float MoveVelocityTowards(float current, float target, float dt)
    {
        const float maxUp = MAX_ACCEL * dt;
        const float maxDown = MAX_DECEL * dt;
        const float delta = target - current;

        if (delta > 0.0f) return current + std::min(delta, maxUp);
        return current + std::max(delta, -maxDown);
    }

    inline float HeadingSpeedScale(float angleError)
    {
        angleError = std::abs(angleError);
        if (angleError <= SLOW_TURN_ANGLE) return 1.0f;
        if (angleError >= PIVOT_TURN_ANGLE) return 0.0f;

        if (angleError <= HARD_TURN_ANGLE)
        {
            const float t = (angleError - SLOW_TURN_ANGLE) / (HARD_TURN_ANGLE - SLOW_TURN_ANGLE);
            return 1.0f + (0.25f - 1.0f) * t;
        }

        const float t = (angleError - HARD_TURN_ANGLE) / (PIVOT_TURN_ANGLE - HARD_TURN_ANGLE);
        return 0.25f * (1.0f - t);
    }

    inline float MoveAngularVelocityTowards(float current, float target, float dt)
    {
        const float maxStep = MAX_ANGULAR_ACCEL * dt;
        const float delta = target - current;
        return current + std::clamp(delta, -maxStep, maxStep);
    }

    inline float EstimateTurnInPlaceTime(float angleError)
    {
        angleError = std::abs(angleError);
        if (angleError <= 0.001f) return 0.0f;

        const float accelAngle = (MAX_ANGULAR_SPEED * MAX_ANGULAR_SPEED) / (2.0f * MAX_ANGULAR_ACCEL);
        if (2.0f * accelAngle <= angleError)
        {
            const float cruiseAngle = angleError - (2.0f * accelAngle);
            return (2.0f * MAX_ANGULAR_SPEED / MAX_ANGULAR_ACCEL) + (cruiseAngle / MAX_ANGULAR_SPEED);
        }

        const float peakAngularVelocity = std::sqrt(angleError * MAX_ANGULAR_ACCEL);
        return 2.0f * peakAngularVelocity / MAX_ANGULAR_ACCEL;
    }

    inline float NormalizeAngle(float angle)
    {
        constexpr float PI = 3.14159265f;
        constexpr float TWO_PI = PI * 2.0f;

        while (angle > PI) angle -= TWO_PI;
        while (angle < -PI) angle += TWO_PI;
        return angle;
    }

    inline float MoveAngleTowards(float current, float target, float dt)
    {
        const float delta = NormalizeAngle(target - current);
        const float maxStep = MAX_ANGULAR_SPEED * dt;
        const float step = std::clamp(delta, -maxStep, maxStep);
        return NormalizeAngle(current + step);
    }
}
