#include "PhysicalFleetDiagnostics.hpp"

#include <cmath>
#include <numbers>

namespace
{
    constexpr float kRadiansToDegrees =
        180.0f / std::numbers::pi_v<float>;

    float NormalizeAngle(float angle)
    {
        return std::remainder(angle, 2.0f * std::numbers::pi_v<float>);
    }
}

PhysicalFleetPoseDiagnostic CalculatePhysicalFleetPoseDiagnostic(
    const PhysicalFleetPoseDiagnosticInput& input)
{
    PhysicalFleetPoseDiagnostic diagnostic;
    diagnostic.actualXMm = input.actualXMm;
    diagnostic.actualZMm = input.actualZMm;
    diagnostic.actualHeadingDeg = input.actualHeadingRad * kRadiansToDegrees;
    diagnostic.targetXMm = input.targetXMm;
    diagnostic.targetZMm = input.targetZMm;
    diagnostic.targetHeadingDeg =
        input.expectedHeadingRad * kRadiansToDegrees;
    diagnostic.deltaXMm = input.targetXMm - input.actualXMm;
    diagnostic.deltaZMm = input.targetZMm - input.actualZMm;

    const float forwardX = std::cos(input.expectedHeadingRad);
    const float forwardZ = std::sin(input.expectedHeadingRad);
    const float leftX = -forwardZ;
    const float leftZ = forwardX;
    diagnostic.longitudinalErrorMm =
        diagnostic.deltaXMm * forwardX + diagnostic.deltaZMm * forwardZ;
    diagnostic.lateralErrorMm =
        diagnostic.deltaXMm * leftX + diagnostic.deltaZMm * leftZ;
    diagnostic.positionErrorMm =
        std::hypot(diagnostic.deltaXMm, diagnostic.deltaZMm);
    diagnostic.headingErrorDeg = NormalizeAngle(
        input.expectedHeadingRad - input.actualHeadingRad) *
        kRadiansToDegrees;
    return diagnostic;
}

PhysicalFleetErrorReduction CalculatePhysicalFleetErrorReduction(
    const PhysicalFleetPoseDiagnostic& before,
    const PhysicalFleetPoseDiagnostic& after)
{
    PhysicalFleetErrorReduction reduction;
    reduction.positionErrorReductionMm =
        before.positionErrorMm - after.positionErrorMm;
    reduction.absoluteHeadingErrorReductionDeg =
        std::abs(before.headingErrorDeg) - std::abs(after.headingErrorDeg);
    return reduction;
}

const char* NodeCorrectionActionName(
    RobotProtocol::NodeCorrectionAction action)
{
    switch (action)
    {
    case RobotProtocol::NodeCorrectionAction::DRIVE_FORWARD:
        return "DRIVE_FORWARD";
    case RobotProtocol::NodeCorrectionAction::TURN_CW:
        return "TURN_CW";
    case RobotProtocol::NodeCorrectionAction::TURN_CCW:
        return "TURN_CCW";
    }
    return "UNKNOWN";
}
