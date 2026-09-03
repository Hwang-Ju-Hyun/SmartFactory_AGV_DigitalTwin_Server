#include "PhysicalFleetDiagnostics.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <numbers>
#include <string>

namespace
{
    [[noreturn]] void Fail(const char* expression, int line)
    {
        std::cerr << "PhysicalFleetDiagnosticsTest failed at line " << line
                  << ": " << expression << "\n";
        std::exit(1);
    }

#define REQUIRE(expression) \
    do { if (!(expression)) Fail(#expression, __LINE__); } while (false)

    bool NearlyEqual(float lhs, float rhs, float epsilon = 0.001f)
    {
        return std::abs(lhs - rhs) <= epsilon;
    }

    void TestEastboundProjectionAndReduction()
    {
        constexpr float degreesToRadians =
            std::numbers::pi_v<float> / 180.0f;
        PhysicalFleetPoseDiagnosticInput input;
        input.actualXMm = 100.0f;
        input.actualZMm = 30.0f;
        input.actualHeadingRad = -5.0f * degreesToRadians;
        input.targetXMm = 130.0f;
        input.targetZMm = 50.0f;
        input.expectedHeadingRad = 10.0f * degreesToRadians;

        const auto before = CalculatePhysicalFleetPoseDiagnostic(input);
        REQUIRE(NearlyEqual(before.deltaXMm, 30.0f));
        REQUIRE(NearlyEqual(before.deltaZMm, 20.0f));
        REQUIRE(NearlyEqual(before.positionErrorMm, std::hypot(30.0f, 20.0f)));
        REQUIRE(NearlyEqual(before.headingErrorDeg, 15.0f));

        input.actualXMm = 125.0f;
        input.actualZMm = 48.0f;
        input.actualHeadingRad = 8.0f * degreesToRadians;
        const auto after = CalculatePhysicalFleetPoseDiagnostic(input);
        const auto reduction = CalculatePhysicalFleetErrorReduction(
            before, after);
        REQUIRE(reduction.positionErrorReductionMm > 30.0f);
        REQUIRE(NearlyEqual(
            reduction.absoluteHeadingErrorReductionDeg, 13.0f));
    }

    void TestCardinalEdgeProjectionAndHeadingWrap()
    {
        PhysicalFleetPoseDiagnosticInput northbound;
        northbound.actualXMm = 0.0f;
        northbound.actualZMm = 0.0f;
        northbound.targetXMm = 20.0f;
        northbound.targetZMm = 30.0f;
        northbound.expectedHeadingRad = std::numbers::pi_v<float> / 2.0f;
        const auto projected =
            CalculatePhysicalFleetPoseDiagnostic(northbound);
        REQUIRE(NearlyEqual(projected.longitudinalErrorMm, 30.0f));
        REQUIRE(NearlyEqual(projected.lateralErrorMm, -20.0f));

        constexpr float degreesToRadians =
            std::numbers::pi_v<float> / 180.0f;
        northbound.actualHeadingRad = 179.0f * degreesToRadians;
        northbound.expectedHeadingRad = -179.0f * degreesToRadians;
        const auto wrapped = CalculatePhysicalFleetPoseDiagnostic(northbound);
        REQUIRE(NearlyEqual(wrapped.headingErrorDeg, 2.0f));
    }

    void TestActionNames()
    {
        REQUIRE(std::string(NodeCorrectionActionName(
                    RobotProtocol::NodeCorrectionAction::DRIVE_FORWARD)) ==
                "DRIVE_FORWARD");
        REQUIRE(std::string(NodeCorrectionActionName(
                    RobotProtocol::NodeCorrectionAction::TURN_CW)) ==
                "TURN_CW");
        REQUIRE(std::string(NodeCorrectionActionName(
                    RobotProtocol::NodeCorrectionAction::TURN_CCW)) ==
                "TURN_CCW");
    }
}

int main()
{
    TestEastboundProjectionAndReduction();
    TestCardinalEdgeProjectionAndHeadingWrap();
    TestActionNames();
    std::cout << "Physical fleet diagnostics tests passed\n";
    return 0;
}
