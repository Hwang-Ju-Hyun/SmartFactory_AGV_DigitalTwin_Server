#pragma once

#include "Protocol.hpp"
#include <cstdint>
#include <string>

enum class MotorFaultDiagnosticOperation : uint8_t
{
    NONE = 0,
    TRAJECTORY_DRIVE = 1,
    TRAJECTORY_TURN = 2,
    CORRECTION_DRIVE = 3,
    CORRECTION_TURN = 4
};

enum class MotorFaultMotionMode : uint8_t
{
    NONE = 0,
    FORWARD = 1,
    TURN_CW = 2,
    TURN_CCW = 3
};

enum class MotorFaultMotionProfile : uint8_t
{
    NORMAL = 0,
    PHYSICAL_FLEET = 1,
    CORRECTION = 2
};

struct WheelMismatchDiagnostic
{
    MotorFaultDiagnosticOperation operation =
        MotorFaultDiagnosticOperation::NONE;
    MotorFaultMotionMode motionMode = MotorFaultMotionMode::NONE;
    MotorFaultMotionProfile motionProfile = MotorFaultMotionProfile::NORMAL;
    int32_t leftProgress = 0;
    int32_t rightProgress = 0;
    int32_t leftTarget = 0;
    int32_t rightTarget = 0;
};

enum class MotorFaultDiagnosticConsumeResult
{
    NOT_DIAGNOSTIC,
    WHEEL_MISMATCH_STARTED,
    IN_PROGRESS,
    COMPLETE,
    MALFORMED
};

class MotorFaultDiagnosticDecoder
{
public:
    MotorFaultDiagnosticConsumeResult Consume(
        RobotProtocol::ErrorCode errorCode,
        uint32_t detail,
        WheelMismatchDiagnostic& outDiagnostic);
    void Reset();

private:
    bool m_Active = false;
    uint8_t m_NextRecordIndex = 0;
    WheelMismatchDiagnostic m_Diagnostic;
};

const char* MotorFaultDiagnosticOperationName(
    MotorFaultDiagnosticOperation operation);
const char* MotorFaultMotionModeName(MotorFaultMotionMode mode);
const char* MotorFaultMotionProfileName(MotorFaultMotionProfile profile);
std::string FormatWheelMismatchDiagnostic(
    const WheelMismatchDiagnostic& diagnostic);
