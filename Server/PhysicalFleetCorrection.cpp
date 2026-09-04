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
        PhysicalFleetCorrectionDecision decision,
        float headingErrorRad)
    {
        const float absoluteHeadingError = std::abs(headingErrorRad);
        decision.action = headingErrorRad < 0.0f
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

    decision.targetBearingRad = std::atan2(deltaZ, deltaX);
    decision.targetBearingErrorRad = NormalizeAngle(
        decision.targetBearingRad - input.actualHeadingRad);
    decision.arrivalHeadingErrorRad = NormalizeAngle(
        input.expectedArrivalHeadingRad - input.actualHeadingRad);
    if (!std::isfinite(decision.targetBearingRad) ||
        !std::isfinite(decision.targetBearingErrorRad) ||
        !std::isfinite(decision.arrivalHeadingErrorRad))
    {
        return PhysicalFleetCorrectionDecision{};
    }

    const float positionExitMm =
        goal == PhysicalFleetCorrectionGoal::START_POSE_STRICT
            ? PhysicalFleetCorrectionPolicy::kStartPosePositionToleranceMm
            : PhysicalFleetCorrectionPolicy::kPositionCorrectionExitMm;
    if (goal != PhysicalFleetCorrectionGoal::HEADING_ONLY &&
        decision.positionErrorMm >
        positionExitMm)
    {
        if (std::abs(decision.targetBearingErrorRad) >
            PhysicalFleetCorrectionPolicy::kHeadingToleranceRad)
        {
            return MakeTurnDecision(
                decision, decision.targetBearingErrorRad);
        }

        decision.action = PhysicalFleetCorrectionAction::DRIVE_FORWARD;
        decision.magnitude = std::min(
            decision.positionErrorMm,
            PhysicalFleetCorrectionPolicy::kMaximumDriveMm);
        return decision;
    }

    if (goal == PhysicalFleetCorrectionGoal::HEADING_ONLY &&
        decision.positionErrorMm >
            PhysicalFleetCorrectionPolicy::kMaximumFinalPositionErrorMm)
    {
        return decision;
    }
    if (std::abs(decision.arrivalHeadingErrorRad) >
        PhysicalFleetCorrectionPolicy::kHeadingToleranceRad)
    {
        return MakeTurnDecision(
            decision, decision.arrivalHeadingErrorRad);
    }

    decision.action = PhysicalFleetCorrectionAction::ACCEPT;
    return decision;
}

PhysicalFleetCoarsePoseDisposition ClassifyPhysicalFleetCoarsePose(
    float positionErrorMm,
    float arrivalHeadingErrorRad)
{
    if (!std::isfinite(positionErrorMm) ||
        !std::isfinite(arrivalHeadingErrorRad) ||
        positionErrorMm < 0.0f ||
        positionErrorMm > PhysicalFleetCorrectionPolicy::kRejectDistanceMm)
    {
        return PhysicalFleetCoarsePoseDisposition::REJECT;
    }
    if (positionErrorMm >
        PhysicalFleetCorrectionPolicy::kPositionCorrectionEntryMm)
    {
        return PhysicalFleetCoarsePoseDisposition::CORRECT_POSITION;
    }
    if (std::abs(arrivalHeadingErrorRad) <=
        PhysicalFleetCorrectionPolicy::kHeadingToleranceRad)
    {
        return PhysicalFleetCoarsePoseDisposition::ACCEPT;
    }
    return PhysicalFleetCoarsePoseDisposition::CORRECT_HEADING;
}

PhysicalFleetCorrectionGoal SelectPhysicalFleetCorrectionGoal(
    bool positionToleranceReached,
    float positionErrorMm)
{
    if (positionToleranceReached)
    {
        return positionErrorMm >
                PhysicalFleetCorrectionPolicy::kPositionCorrectionEntryMm
            ? PhysicalFleetCorrectionGoal::POSITION_AND_HEADING
            : PhysicalFleetCorrectionGoal::HEADING_ONLY;
    }

    return positionErrorMm <=
            PhysicalFleetCorrectionPolicy::kPositionCorrectionExitMm
        ? PhysicalFleetCorrectionGoal::HEADING_ONLY
        : PhysicalFleetCorrectionGoal::POSITION_AND_HEADING;
}

PhysicalFleetPreDepartureDecision DecidePhysicalFleetPreDepartureAlignment(
    float positionErrorMm,
    float actualHeadingRad,
    float targetHeadingRad,
    uint8_t completedAttempts)
{
    PhysicalFleetPreDepartureDecision result;
    if (!std::isfinite(positionErrorMm) || positionErrorMm < 0.0f ||
        positionErrorMm >
            PhysicalFleetCorrectionPolicy::kMaximumFinalPositionErrorMm ||
        !std::isfinite(actualHeadingRad) ||
        !std::isfinite(targetHeadingRad))
    {
        return result;
    }

    result.headingErrorRad = NormalizeAngle(
        targetHeadingRad - actualHeadingRad);
    if (!std::isfinite(result.headingErrorRad))
        return PhysicalFleetPreDepartureDecision{};

    if (std::abs(result.headingErrorRad) <=
        PhysicalFleetCorrectionPolicy::kHeadingToleranceRad)
    {
        result.action = PhysicalFleetCorrectionAction::ACCEPT;
        return result;
    }

    if (completedAttempts >= PhysicalFleetCorrectionPolicy::
            kMaximumPreDepartureAlignmentAttempts)
    {
        result.attemptLimitReached = true;
        return result;
    }

    result.action = result.headingErrorRad < 0.0f
        ? PhysicalFleetCorrectionAction::TURN_CW
        : PhysicalFleetCorrectionAction::TURN_CCW;
    result.magnitude = std::min(
        std::abs(result.headingErrorRad),
        PhysicalFleetCorrectionPolicy::kMaximumTurnRad);
    return result;
}

PhysicalFleetPreDepartureTransition
DecidePhysicalFleetPreDepartureRecoveryTransition(
    const PhysicalFleetPreDepartureRecoveryInput& input)
{
    if (!std::isfinite(input.positionErrorMm) ||
        !std::isfinite(input.turnPositionIncreaseMm) ||
        input.positionErrorMm < 0.0f ||
        input.positionErrorMm >
            PhysicalFleetCorrectionPolicy::kRejectDistanceMm)
    {
        return PhysicalFleetPreDepartureTransition::SAFE_STOP;
    }

    if (input.objective ==
        PhysicalFleetPreDepartureObjective::RECENTER_POSITION)
    {
        return input.positionErrorMm <=
                PhysicalFleetCorrectionPolicy::kPositionCorrectionExitMm
            ? PhysicalFleetPreDepartureTransition::RETURN_TO_ALIGNMENT
            : PhysicalFleetPreDepartureTransition::STAY;
    }

    if (input.positionErrorMm <=
        PhysicalFleetCorrectionPolicy::kMaximumFinalPositionErrorMm)
    {
        return PhysicalFleetPreDepartureTransition::STAY;
    }

    const bool completedTurn =
        input.completedAction == PhysicalFleetCorrectionAction::TURN_CW ||
        input.completedAction == PhysicalFleetCorrectionAction::TURN_CCW;
    if (input.completedFreshCommandMeasurement && completedTurn &&
        input.turnPositionIncreaseMm <=
            PhysicalFleetCorrectionPolicy::kMaximumTurnPositionIncreaseMm &&
        input.completedRecoveries < PhysicalFleetCorrectionPolicy::
            kMaximumPreDepartureRecenterRecoveries)
    {
        return PhysicalFleetPreDepartureTransition::ENTER_RECENTER;
    }
    return PhysicalFleetPreDepartureTransition::SAFE_STOP;
}

PhysicalFleetProgressResult CheckPhysicalFleetCorrectionProgress(
    const PhysicalFleetProgressCheck& check)
{
    PhysicalFleetProgressResult result;
    const bool isTurn =
        check.action == PhysicalFleetCorrectionAction::TURN_CW ||
        check.action == PhysicalFleetCorrectionAction::TURN_CCW;
    if (isTurn &&
        check.afterPositionErrorMm - check.beforePositionErrorMm >
            PhysicalFleetCorrectionPolicy::kMaximumTurnPositionIncreaseMm)
    {
        result.reason =
            PhysicalFleetNonConvergenceReason::TURN_POSITION_SPIKE;
        return result;
    }

    bool improved = false;
    if (check.action == PhysicalFleetCorrectionAction::DRIVE_FORWARD)
    {
        improved =
            check.beforePositionErrorMm - check.afterPositionErrorMm >=
            PhysicalFleetCorrectionPolicy::kMinimumPositionProgressMm;
    }
    else if (isTurn)
    {
        improved =
            check.beforeObjectiveHeadingErrorRad -
                check.afterObjectiveHeadingErrorRad >=
            PhysicalFleetCorrectionPolicy::kMinimumHeadingProgressRad;
    }
    else
    {
        result.reason =
            PhysicalFleetNonConvergenceReason::ERROR_NOT_DECREASING;
        return result;
    }

    result.consecutiveNonImprovingPrimitives = improved
        ? 0
        : static_cast<uint8_t>(
              check.consecutiveNonImprovingPrimitives + 1);
    if (result.consecutiveNonImprovingPrimitives >=
        PhysicalFleetCorrectionPolicy::
            kMaximumConsecutiveNonImprovingPrimitives)
    {
        result.reason =
            PhysicalFleetNonConvergenceReason::ERROR_NOT_DECREASING;
    }
    return result;
}

PhysicalFleetNonConvergenceReason CheckPhysicalFleetTurnCommand(
    PhysicalFleetCorrectionAction action,
    float magnitudeRad,
    bool hasPreviousTurn,
    PhysicalFleetCorrectionAction previousTurnAction,
    uint8_t consecutiveSameDirectionTurns,
    float cumulativeTurnRad)
{
    const bool isTurn =
        action == PhysicalFleetCorrectionAction::TURN_CW ||
        action == PhysicalFleetCorrectionAction::TURN_CCW;
    if (!isTurn)
        return PhysicalFleetNonConvergenceReason::NONE;

    const uint8_t nextSameDirectionTurns =
        hasPreviousTurn && previousTurnAction == action
            ? static_cast<uint8_t>(consecutiveSameDirectionTurns + 1)
            : 1;
    if (nextSameDirectionTurns >
        PhysicalFleetCorrectionPolicy::
            kMaximumConsecutiveSameDirectionTurns)
    {
        return PhysicalFleetNonConvergenceReason::
            SAME_DIRECTION_TURN_LIMIT;
    }
    if (cumulativeTurnRad + magnitudeRad >
        PhysicalFleetCorrectionPolicy::kMaximumCumulativeTurnRad)
    {
        return PhysicalFleetNonConvergenceReason::CUMULATIVE_TURN_LIMIT;
    }
    return PhysicalFleetNonConvergenceReason::NONE;
}

const char* PhysicalFleetNonConvergenceReasonName(
    PhysicalFleetNonConvergenceReason reason)
{
    switch (reason)
    {
    case PhysicalFleetNonConvergenceReason::NONE:
        return "NONE";
    case PhysicalFleetNonConvergenceReason::ERROR_NOT_DECREASING:
        return "ERROR_NOT_DECREASING";
    case PhysicalFleetNonConvergenceReason::SAME_DIRECTION_TURN_LIMIT:
        return "SAME_DIRECTION_TURN_LIMIT";
    case PhysicalFleetNonConvergenceReason::CUMULATIVE_TURN_LIMIT:
        return "CUMULATIVE_TURN_LIMIT";
    case PhysicalFleetNonConvergenceReason::TURN_POSITION_SPIKE:
        return "TURN_POSITION_SPIKE";
    }
    return "UNKNOWN";
}
