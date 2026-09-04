#pragma once

#include <cstdint>

enum class PhysicalFleetCorrectionAction
{
    ACCEPT,
    DRIVE_FORWARD,
    TURN_CW,
    TURN_CCW,
    REJECT
};

enum class PhysicalFleetCorrectionGoal
{
    START_POSE_STRICT,
    POSITION_AND_HEADING,
    HEADING_ONLY
};

enum class PhysicalFleetCoarsePoseDisposition
{
    ACCEPT,
    CORRECT_POSITION,
    CORRECT_HEADING,
    REJECT
};

struct PhysicalFleetCorrectionInput
{
    float actualXMm = 0.0f;
    float actualZMm = 0.0f;
    float actualHeadingRad = 0.0f;
    float targetXMm = 0.0f;
    float targetZMm = 0.0f;
    float expectedArrivalHeadingRad = 0.0f;
};

struct PhysicalFleetCorrectionDecision
{
    PhysicalFleetCorrectionAction action =
        PhysicalFleetCorrectionAction::REJECT;

    // DRIVE_FORWARD uses millimetres. TURN_CW/TURN_CCW use radians.
    // ACCEPT and REJECT use canonical zero.
    float magnitude = 0.0f;
    float positionErrorMm = 0.0f;
    float targetBearingRad = 0.0f;
    float targetBearingErrorRad = 0.0f;
    float arrivalHeadingErrorRad = 0.0f;
};

struct PhysicalFleetPreDepartureDecision
{
    PhysicalFleetCorrectionAction action =
        PhysicalFleetCorrectionAction::REJECT;
    float magnitude = 0.0f;
    float headingErrorRad = 0.0f;
    bool attemptLimitReached = false;
};

enum class PhysicalFleetPreDepartureObjective
{
    ALIGN_HEADING,
    RECENTER_POSITION
};

enum class PhysicalFleetPreDepartureTransition
{
    STAY,
    ENTER_RECENTER,
    RETURN_TO_ALIGNMENT,
    SAFE_STOP
};

struct PhysicalFleetPreDepartureRecoveryInput
{
    PhysicalFleetPreDepartureObjective objective =
        PhysicalFleetPreDepartureObjective::ALIGN_HEADING;
    float positionErrorMm = 0.0f;
    bool completedFreshCommandMeasurement = false;
    PhysicalFleetCorrectionAction completedAction =
        PhysicalFleetCorrectionAction::REJECT;
    float turnPositionIncreaseMm = 0.0f;
    uint8_t completedRecoveries = 0;
};

enum class PhysicalFleetNonConvergenceReason
{
    NONE,
    ERROR_NOT_DECREASING,
    SAME_DIRECTION_TURN_LIMIT,
    CUMULATIVE_TURN_LIMIT,
    TURN_POSITION_SPIKE
};

struct PhysicalFleetProgressCheck
{
    PhysicalFleetCorrectionAction action =
        PhysicalFleetCorrectionAction::REJECT;
    float beforePositionErrorMm = 0.0f;
    float afterPositionErrorMm = 0.0f;
    float beforeObjectiveHeadingErrorRad = 0.0f;
    float afterObjectiveHeadingErrorRad = 0.0f;
    uint8_t consecutiveNonImprovingPrimitives = 0;
};

struct PhysicalFleetProgressResult
{
    PhysicalFleetNonConvergenceReason reason =
        PhysicalFleetNonConvergenceReason::NONE;
    uint8_t consecutiveNonImprovingPrimitives = 0;
};

namespace PhysicalFleetCorrectionPolicy
{
    inline constexpr uint8_t kMaximumPrimitivesPerNode = 8;
    inline constexpr float kStartPosePositionToleranceMm = 20.0f;
    inline constexpr float kPositionCorrectionEntryMm = 40.0f;
    inline constexpr float kPositionCorrectionExitMm = 35.0f;
    inline constexpr float kMaximumFinalPositionErrorMm =
        kPositionCorrectionEntryMm;
    inline constexpr float kHeadingToleranceRad =
        0.17453292519943295f; // 10 degrees; avoids unreliable floor micro-turns
    inline constexpr float kRejectDistanceMm = 200.0f;
    inline constexpr float kMaximumDriveMm = 120.0f;
    inline constexpr float kMaximumTurnRad =
        1.5707963267948966f; // 90 degrees
    inline constexpr float kMinimumPositionProgressMm = 2.0f;
    inline constexpr float kMinimumHeadingProgressRad =
        0.03490658503988659f; // 2 degrees
    inline constexpr uint8_t kMaximumConsecutiveNonImprovingPrimitives = 2;
    inline constexpr uint8_t kMaximumConsecutiveSameDirectionTurns = 2;
    inline constexpr float kMaximumCumulativeTurnRad =
        6.2831853071795865f; // one full revolution
    inline constexpr float kMaximumTurnPositionIncreaseMm = 25.0f;
    inline constexpr uint8_t kMaximumPreDepartureAlignmentAttempts = 2;
    inline constexpr uint8_t kMaximumPreDepartureRecenterRecoveries = 1;
    inline constexpr uint8_t kMaximumPreDepartureRecenterPrimitives = 3;
    inline constexpr float kMaximumPreDepartureRecenterTurnRad =
        3.1415926535897932f; // bounded independently; global limit also applies
}

PhysicalFleetCorrectionDecision DecidePhysicalFleetCorrection(
    const PhysicalFleetCorrectionInput& input,
    PhysicalFleetCorrectionGoal goal =
        PhysicalFleetCorrectionGoal::POSITION_AND_HEADING);
bool IsPhysicalFleetInitialDeparturePoseAccepted(
    const PhysicalFleetCorrectionInput& input);
PhysicalFleetCoarsePoseDisposition ClassifyPhysicalFleetCoarsePose(
    float positionErrorMm,
    float arrivalHeadingErrorRad);
PhysicalFleetCorrectionGoal SelectPhysicalFleetCorrectionGoal(
    bool positionToleranceReached,
    float positionErrorMm);
PhysicalFleetPreDepartureDecision DecidePhysicalFleetPreDepartureAlignment(
    float positionErrorMm,
    float actualHeadingRad,
    float targetHeadingRad,
    uint8_t completedAttempts);
PhysicalFleetPreDepartureTransition
DecidePhysicalFleetPreDepartureRecoveryTransition(
    const PhysicalFleetPreDepartureRecoveryInput& input);
PhysicalFleetProgressResult CheckPhysicalFleetCorrectionProgress(
    const PhysicalFleetProgressCheck& check);
PhysicalFleetNonConvergenceReason CheckPhysicalFleetTurnCommand(
    PhysicalFleetCorrectionAction action,
    float magnitudeRad,
    bool hasPreviousTurn,
    PhysicalFleetCorrectionAction previousTurnAction,
    uint8_t consecutiveSameDirectionTurns,
    float cumulativeTurnRad);
const char* PhysicalFleetNonConvergenceReasonName(
    PhysicalFleetNonConvergenceReason reason);
