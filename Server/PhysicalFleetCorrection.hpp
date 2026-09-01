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
    POSITION_AND_HEADING,
    HEADING_ONLY
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

    // Signed shortest angular error used for this decision. While position is
    // outside tolerance this points toward the target; once position is
    // accepted it points toward expectedArrivalHeadingRad.
    float headingErrorRad = 0.0f;
};

namespace PhysicalFleetCorrectionPolicy
{
    inline constexpr uint8_t kMaximumPrimitivesPerNode = 8;
    inline constexpr float kPositionToleranceMm = 20.0f;
    inline constexpr float kHeadingToleranceRad =
        0.17453292519943295f; // 10 degrees; avoids unreliable floor micro-turns
    inline constexpr float kRejectDistanceMm = 200.0f;
    inline constexpr float kMaximumHeadingAlignmentDriftMm = 75.0f;
    inline constexpr float kMaximumDriveMm = 120.0f;
    inline constexpr float kMaximumTurnRad =
        1.5707963267948966f; // 90 degrees
}

PhysicalFleetCorrectionDecision DecidePhysicalFleetCorrection(
    const PhysicalFleetCorrectionInput& input,
    PhysicalFleetCorrectionGoal goal =
        PhysicalFleetCorrectionGoal::POSITION_AND_HEADING);
