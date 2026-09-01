#include "PhysicalFleetCorrection.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numbers>

namespace
{
    constexpr float kDegreesToRadians =
        std::numbers::pi_v<float> / 180.0f;

    [[noreturn]] void Fail(const char* expression, int line)
    {
        std::cerr << "PhysicalFleetCorrectionTest failed at line " << line
                  << ": " << expression << "\n";
        std::exit(1);
    }

#define REQUIRE(expression) \
    do { if (!(expression)) Fail(#expression, __LINE__); } while (false)

    bool NearlyEqual(float lhs, float rhs, float epsilon = 0.0001f)
    {
        return std::abs(lhs - rhs) <= epsilon;
    }

    PhysicalFleetCorrectionInput AtOrigin()
    {
        return {};
    }

    void TestAcceptsPoseWithinTolerance()
    {
        auto input = AtOrigin();
        input.actualXMm = 19.0f;
        input.actualHeadingRad = 4.0f * kDegreesToRadians;

        const auto decision = DecidePhysicalFleetCorrection(input);
        REQUIRE(decision.action == PhysicalFleetCorrectionAction::ACCEPT);
        REQUIRE(decision.magnitude == 0.0f);
        REQUIRE(NearlyEqual(decision.positionErrorMm, 19.0f));
        REQUIRE(NearlyEqual(
            decision.headingErrorRad, -4.0f * kDegreesToRadians));
    }

    void TestToleranceAndRejectBoundariesAreInclusive()
    {
        auto input = AtOrigin();
        input.actualXMm =
            PhysicalFleetCorrectionPolicy::kPositionToleranceMm;
        input.actualHeadingRad =
            PhysicalFleetCorrectionPolicy::kHeadingToleranceRad;

        const auto accepted = DecidePhysicalFleetCorrection(input);
        REQUIRE(accepted.action == PhysicalFleetCorrectionAction::ACCEPT);

        input = AtOrigin();
        input.targetXMm =
            PhysicalFleetCorrectionPolicy::kRejectDistanceMm;
        const auto boundedDrive = DecidePhysicalFleetCorrection(input);
        REQUIRE(boundedDrive.action ==
                PhysicalFleetCorrectionAction::DRIVE_FORWARD);
        REQUIRE(NearlyEqual(
            boundedDrive.magnitude,
            PhysicalFleetCorrectionPolicy::kMaximumDriveMm));
    }

    void TestTurnsTowardPositionBeforeDriving()
    {
        auto input = AtOrigin();
        input.targetZMm = 100.0f;

        const auto ccw = DecidePhysicalFleetCorrection(input);
        REQUIRE(ccw.action == PhysicalFleetCorrectionAction::TURN_CCW);
        REQUIRE(NearlyEqual(
            ccw.magnitude,
            PhysicalFleetCorrectionPolicy::kMaximumTurnRad));
        REQUIRE(NearlyEqual(ccw.positionErrorMm, 100.0f));

        input.targetZMm = -100.0f;
        const auto cw = DecidePhysicalFleetCorrection(input);
        REQUIRE(cw.action == PhysicalFleetCorrectionAction::TURN_CW);
        REQUIRE(NearlyEqual(
            cw.magnitude,
            PhysicalFleetCorrectionPolicy::kMaximumTurnRad));
    }

    void TestCapsTurnAtNinetyDegrees()
    {
        auto input = AtOrigin();
        input.targetXMm = -100.0f;

        const auto decision = DecidePhysicalFleetCorrection(input);
        REQUIRE(decision.action == PhysicalFleetCorrectionAction::TURN_CCW);
        REQUIRE(NearlyEqual(
            decision.magnitude,
            PhysicalFleetCorrectionPolicy::kMaximumTurnRad));
        REQUIRE(NearlyEqual(
            std::abs(decision.headingErrorRad),
            std::numbers::pi_v<float>));
    }

    void TestDrivesWhenAlignedAndCapsDistance()
    {
        auto input = AtOrigin();
        input.targetXMm = 150.0f;

        const auto decision = DecidePhysicalFleetCorrection(input);
        REQUIRE(decision.action ==
                PhysicalFleetCorrectionAction::DRIVE_FORWARD);
        REQUIRE(NearlyEqual(
            decision.magnitude,
            PhysicalFleetCorrectionPolicy::kMaximumDriveMm));
        REQUIRE(NearlyEqual(decision.positionErrorMm, 150.0f));
        REQUIRE(NearlyEqual(decision.headingErrorRad, 0.0f));
    }

    void TestRecordedEightDegreeBearingErrorDrivesInsteadOfMicroTurning()
    {
        auto input = AtOrigin();
        input.targetXMm = 96.2116f;
        input.actualHeadingRad = -8.01f * kDegreesToRadians;

        const auto decision = DecidePhysicalFleetCorrection(input);
        REQUIRE(decision.action ==
                PhysicalFleetCorrectionAction::DRIVE_FORWARD);
        REQUIRE(NearlyEqual(decision.magnitude, 96.2116f));
        REQUIRE(NearlyEqual(
            decision.headingErrorRad,
            8.01f * kDegreesToRadians,
            0.0002f));
    }

    void TestCorrectsArrivalHeadingAfterPosition()
    {
        auto input = AtOrigin();
        input.expectedArrivalHeadingRad = 45.0f * kDegreesToRadians;

        const auto ccw = DecidePhysicalFleetCorrection(input);
        REQUIRE(ccw.action == PhysicalFleetCorrectionAction::TURN_CCW);
        REQUIRE(NearlyEqual(ccw.magnitude, 45.0f * kDegreesToRadians));

        input.expectedArrivalHeadingRad = -45.0f * kDegreesToRadians;
        const auto cw = DecidePhysicalFleetCorrection(input);
        REQUIRE(cw.action == PhysicalFleetCorrectionAction::TURN_CW);
        REQUIRE(NearlyEqual(cw.magnitude, 45.0f * kDegreesToRadians));
    }

    void TestHeadingOnlyGoalDoesNotReturnToPositionCorrection()
    {
        auto input = AtOrigin();
        input.actualXMm = 50.7381f;

        const auto strict = DecidePhysicalFleetCorrection(input);
        REQUIRE(strict.action == PhysicalFleetCorrectionAction::TURN_CCW);

        const auto headingOnly = DecidePhysicalFleetCorrection(
            input,
            PhysicalFleetCorrectionGoal::HEADING_ONLY);
        REQUIRE(headingOnly.action == PhysicalFleetCorrectionAction::ACCEPT);

        input.actualHeadingRad = -32.0083f * kDegreesToRadians;
        const auto remainingHeading = DecidePhysicalFleetCorrection(
            input,
            PhysicalFleetCorrectionGoal::HEADING_ONLY);
        REQUIRE(remainingHeading.action ==
                PhysicalFleetCorrectionAction::TURN_CCW);
        REQUIRE(NearlyEqual(
            remainingHeading.magnitude,
            32.0083f * kDegreesToRadians,
            0.0002f));

        input.actualXMm = 62.7304f;
        input.actualHeadingRad = -2.48727f * kDegreesToRadians;
        const auto aligned = DecidePhysicalFleetCorrection(
            input,
            PhysicalFleetCorrectionGoal::HEADING_ONLY);
        REQUIRE(aligned.action == PhysicalFleetCorrectionAction::ACCEPT);
        REQUIRE(aligned.magnitude == 0.0f);
        REQUIRE(NearlyEqual(aligned.positionErrorMm, 62.7304f));
        REQUIRE(NearlyEqual(
            aligned.headingErrorRad,
            2.48727f * kDegreesToRadians,
            0.0002f));

        input.actualXMm =
            PhysicalFleetCorrectionPolicy::kMaximumHeadingAlignmentDriftMm +
            0.01f;
        const auto excessiveDrift = DecidePhysicalFleetCorrection(
            input,
            PhysicalFleetCorrectionGoal::HEADING_ONLY);
        REQUIRE(excessiveDrift.action ==
                PhysicalFleetCorrectionAction::REJECT);
    }

    void TestUsesShortestWrappedHeadingError()
    {
        auto input = AtOrigin();
        input.actualHeadingRad = 179.0f * kDegreesToRadians;
        input.expectedArrivalHeadingRad = -179.0f * kDegreesToRadians;

        const auto decision = DecidePhysicalFleetCorrection(input);
        REQUIRE(decision.action == PhysicalFleetCorrectionAction::ACCEPT);
        REQUIRE(NearlyEqual(
            decision.headingErrorRad, 2.0f * kDegreesToRadians, 0.0002f));
    }

    void TestRejectsExcessiveDistance()
    {
        auto input = AtOrigin();
        input.targetXMm = 200.01f;

        const auto decision = DecidePhysicalFleetCorrection(input);
        REQUIRE(decision.action == PhysicalFleetCorrectionAction::REJECT);
        REQUIRE(decision.magnitude == 0.0f);
        REQUIRE(NearlyEqual(decision.positionErrorMm, 200.01f));
    }

    void TestRecordedRecoveryRetainsTwoFinalPrimitives()
    {
        REQUIRE(
            PhysicalFleetCorrectionPolicy::kMaximumPrimitivesPerNode == 8);

        // The 2026-08-31 floor run used six primitives to get this close to
        // node 2. It still needed one short drive followed by one final
        // heading correction, so both must fit inside the bounded session.
        auto input = AtOrigin();
        input.actualXMm = 324.0f;
        input.actualZMm = -13.0f;
        input.actualHeadingRad = 28.0f * kDegreesToRadians;
        input.targetXMm = 350.0f;

        const auto finalDrive = DecidePhysicalFleetCorrection(input);
        REQUIRE(finalDrive.action ==
                PhysicalFleetCorrectionAction::DRIVE_FORWARD);
        REQUIRE(finalDrive.positionErrorMm >
                PhysicalFleetCorrectionPolicy::kPositionToleranceMm);

        input.actualXMm = input.targetXMm;
        input.actualZMm = input.targetZMm;
        const auto finalHeading = DecidePhysicalFleetCorrection(input);
        REQUIRE(finalHeading.action == PhysicalFleetCorrectionAction::TURN_CW);
    }

    void TestRejectsEveryNonFiniteInput()
    {
        const float invalidValues[]{
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::infinity()
        };

        for (float invalid : invalidValues)
        {
            for (int field = 0; field < 6; ++field)
            {
                auto input = AtOrigin();
                float* fields[]{
                    &input.actualXMm,
                    &input.actualZMm,
                    &input.actualHeadingRad,
                    &input.targetXMm,
                    &input.targetZMm,
                    &input.expectedArrivalHeadingRad
                };
                *fields[field] = invalid;

                const auto decision = DecidePhysicalFleetCorrection(input);
                REQUIRE(decision.action ==
                        PhysicalFleetCorrectionAction::REJECT);
                REQUIRE(decision.magnitude == 0.0f);
            }
        }

        auto overflowingHeading = AtOrigin();
        overflowingHeading.actualHeadingRad =
            std::numeric_limits<float>::max();
        overflowingHeading.expectedArrivalHeadingRad =
            -std::numeric_limits<float>::max();
        REQUIRE(DecidePhysicalFleetCorrection(overflowingHeading).action ==
                PhysicalFleetCorrectionAction::REJECT);
    }
}

int main()
{
    TestAcceptsPoseWithinTolerance();
    TestToleranceAndRejectBoundariesAreInclusive();
    TestTurnsTowardPositionBeforeDriving();
    TestCapsTurnAtNinetyDegrees();
    TestDrivesWhenAlignedAndCapsDistance();
    TestRecordedEightDegreeBearingErrorDrivesInsteadOfMicroTurning();
    TestCorrectsArrivalHeadingAfterPosition();
    TestHeadingOnlyGoalDoesNotReturnToPositionCorrection();
    TestUsesShortestWrappedHeadingError();
    TestRejectsExcessiveDistance();
    TestRecordedRecoveryRetainsTwoFinalPrimitives();
    TestRejectsEveryNonFiniteInput();
    std::cout << "Physical fleet correction tests passed\n";
    return 0;
}
