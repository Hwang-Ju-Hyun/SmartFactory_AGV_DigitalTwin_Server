#pragma once

#include "Protocol.hpp"

struct PhysicalFleetPoseDiagnosticInput
{
    float actualXMm = 0.0f;
    float actualZMm = 0.0f;
    float actualHeadingRad = 0.0f;
    float targetXMm = 0.0f;
    float targetZMm = 0.0f;
    float expectedHeadingRad = 0.0f;
};

struct PhysicalFleetPoseDiagnostic
{
    float actualXMm = 0.0f;
    float actualZMm = 0.0f;
    float actualHeadingDeg = 0.0f;
    float targetXMm = 0.0f;
    float targetZMm = 0.0f;
    float targetHeadingDeg = 0.0f;
    float deltaXMm = 0.0f;
    float deltaZMm = 0.0f;
    float longitudinalErrorMm = 0.0f;
    float lateralErrorMm = 0.0f;
    float positionErrorMm = 0.0f;
    float headingErrorDeg = 0.0f;
};

struct PhysicalFleetErrorReduction
{
    float positionErrorReductionMm = 0.0f;
    float absoluteHeadingErrorReductionDeg = 0.0f;
};

PhysicalFleetPoseDiagnostic CalculatePhysicalFleetPoseDiagnostic(
    const PhysicalFleetPoseDiagnosticInput& input);
PhysicalFleetErrorReduction CalculatePhysicalFleetErrorReduction(
    const PhysicalFleetPoseDiagnostic& before,
    const PhysicalFleetPoseDiagnostic& after);
const char* NodeCorrectionActionName(
    RobotProtocol::NodeCorrectionAction action);
