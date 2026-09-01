#include "PhysicalFleetCorrection.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace
{
    float NormalizeAngle(float angle)
    {
        return std::remainder(angle, 2.0f * std::numbers::pi_v<float>);
    }

    bool IsFinite(const PhysicalFleetCorrectionInput& input)
    {
        return std::isfinite(input.actualXMm) &&
            std::isfinite(input.actualZMm) &&
            std::isfinite(input.actualHeadingRad) &&
            std::isfinite(input.targetXMm) &&
            std::isfinite(input.targetZMm) &&
            std::isfinite(input.expectedArrivalHeadingRad);
    }

    PhysicalFleetCorrectionDecision MakeTurnDecision(
        PhysicalFleetCorrectionDecision decision)
    {
        const float absoluteHeadingError = std::abs(decision.headingErrorRad);
        decision.action = decision.headingErrorRad < 0.0f
            ? PhysicalFleetCorrectionAction::TURN_CW
            : PhysicalFleetCorrectionAction::TURN_CCW;
        decision.magnitude = std::min(
            absoluteHeadingError,
            PhysicalFleetCorrectionPolicy::kMaximumTurnRad);
        return decision;
    }
}

PhysicalFleetCorrectionDecision DecidePhysicalFleetCorrection(
    const PhysicalFleetCorrectionInput& input,
    PhysicalFleetCorrectionGoal goal)
{
    PhysicalFleetCorrectionDecision decision;
    if (!IsFinite(input))
        return decision;

    const float deltaX = input.targetXMm - input.actualXMm;
    const float deltaZ = input.targetZMm - input.actualZMm;
    decision.positionErrorMm = std::hypot(deltaX, deltaZ);
    if (!std::isfinite(decision.positionErrorMm))
        return PhysicalFleetCorrectionDecision{};

    if (decision.positionErrorMm >
        PhysicalFleetCorrectionPolicy::kRejectDistanceMm)
    {
        return decision;
    }

    if (goal != PhysicalFleetCorrectionGoal::HEADING_ONLY &&
        decision.positionErrorMm >
        PhysicalFleetCorrectionPolicy::kPositionToleranceMm)
    {
        const float targetBearingRad = std::atan2(deltaZ, deltaX);
        decision.headingErrorRad = NormalizeAngle(
            targetBearingRad - input.actualHeadingRad);
        if (!std::isfinite(decision.headingErrorRad))
            return PhysicalFleetCorrectionDecision{};

        if (std::abs(decision.headingErrorRad) >
            PhysicalFleetCorrectionPolicy::kHeadingToleranceRad)
        {
            return MakeTurnDecision(decision);
        }

        decision.action = PhysicalFleetCorrectionAction::DRIVE_FORWARD;
        decision.magnitude = std::min(
            decision.positionErrorMm,
            PhysicalFleetCorrectionPolicy::kMaximumDriveMm);
        return decision;
    }

    decision.headingErrorRad = NormalizeAngle(
        input.expectedArrivalHeadingRad - input.actualHeadingRad);
    if (!std::isfinite(decision.headingErrorRad))
        return PhysicalFleetCorrectionDecision{};
    if (goal == PhysicalFleetCorrectionGoal::HEADING_ONLY &&
        decision.positionErrorMm >
            PhysicalFleetCorrectionPolicy::kMaximumHeadingAlignmentDriftMm)
    {
        return decision;
    }
    if (std::abs(decision.headingErrorRad) >
        PhysicalFleetCorrectionPolicy::kHeadingToleranceRad)
    {
        return MakeTurnDecision(decision);
    }

    decision.action = PhysicalFleetCorrectionAction::ACCEPT;
    return decision;
}
