#include "MotorFaultDiagnostic.hpp"

#include <array>
#include <sstream>

namespace
{
    constexpr std::array<RobotProtocol::MotorFaultDiagnosticTag, 5>
        kExpectedTags = {
            RobotProtocol::MotorFaultDiagnosticTag::CONTEXT,
            RobotProtocol::MotorFaultDiagnosticTag::LEFT_PROGRESS,
            RobotProtocol::MotorFaultDiagnosticTag::RIGHT_PROGRESS,
            RobotProtocol::MotorFaultDiagnosticTag::LEFT_TARGET,
            RobotProtocol::MotorFaultDiagnosticTag::RIGHT_TARGET
        };

    bool IsDiagnosticTag(uint8_t tag)
    {
        return tag >= static_cast<uint8_t>(kExpectedTags.front()) &&
            tag <= static_cast<uint8_t>(kExpectedTags.back());
    }

    int32_t DecodeSigned24(uint32_t detail)
    {
        uint32_t value = detail & 0x00FFFFFFu;
        if ((value & 0x00800000u) != 0)
            value |= 0xFF000000u;
        return static_cast<int32_t>(value);
    }

    bool IsValidContext(const WheelMismatchDiagnostic& diagnostic)
    {
        const uint8_t operation = static_cast<uint8_t>(diagnostic.operation);
        const uint8_t mode = static_cast<uint8_t>(diagnostic.motionMode);
        const uint8_t profile = static_cast<uint8_t>(diagnostic.motionProfile);
        return operation >= 1 && operation <= 4 &&
            mode >= 1 && mode <= 3 &&
            profile <= 2;
    }
}

MotorFaultDiagnosticConsumeResult MotorFaultDiagnosticDecoder::Consume(
    RobotProtocol::ErrorCode errorCode,
    uint32_t detail,
    WheelMismatchDiagnostic& outDiagnostic)
{
    if (errorCode != RobotProtocol::ErrorCode::MOTOR_FAULT)
    {
        Reset();
        return MotorFaultDiagnosticConsumeResult::NOT_DIAGNOSTIC;
    }

    if (detail == RobotProtocol::kWheelMismatchLegacyDetail)
    {
        Reset();
        m_Active = true;
        return MotorFaultDiagnosticConsumeResult::WHEEL_MISMATCH_STARTED;
    }

    const uint8_t tag = static_cast<uint8_t>(detail >> 24);
    if (!IsDiagnosticTag(tag))
    {
        Reset();
        return MotorFaultDiagnosticConsumeResult::NOT_DIAGNOSTIC;
    }

    if (!m_Active || m_NextRecordIndex >= kExpectedTags.size() ||
        tag != static_cast<uint8_t>(kExpectedTags[m_NextRecordIndex]))
    {
        Reset();
        return MotorFaultDiagnosticConsumeResult::MALFORMED;
    }

    switch (kExpectedTags[m_NextRecordIndex])
    {
    case RobotProtocol::MotorFaultDiagnosticTag::CONTEXT:
        if (((detail >> 16) & 0xFFu) !=
            RobotProtocol::kMotorFaultDiagnosticVersion)
        {
            Reset();
            return MotorFaultDiagnosticConsumeResult::MALFORMED;
        }
        m_Diagnostic.operation = static_cast<MotorFaultDiagnosticOperation>(
            (detail >> 8) & 0xFFu);
        m_Diagnostic.motionMode = static_cast<MotorFaultMotionMode>(
            (detail >> 4) & 0x0Fu);
        m_Diagnostic.motionProfile = static_cast<MotorFaultMotionProfile>(
            detail & 0x0Fu);
        if (!IsValidContext(m_Diagnostic))
        {
            Reset();
            return MotorFaultDiagnosticConsumeResult::MALFORMED;
        }
        break;
    case RobotProtocol::MotorFaultDiagnosticTag::LEFT_PROGRESS:
        m_Diagnostic.leftProgress = DecodeSigned24(detail);
        break;
    case RobotProtocol::MotorFaultDiagnosticTag::RIGHT_PROGRESS:
        m_Diagnostic.rightProgress = DecodeSigned24(detail);
        break;
    case RobotProtocol::MotorFaultDiagnosticTag::LEFT_TARGET:
        m_Diagnostic.leftTarget = DecodeSigned24(detail);
        break;
    case RobotProtocol::MotorFaultDiagnosticTag::RIGHT_TARGET:
        m_Diagnostic.rightTarget = DecodeSigned24(detail);
        break;
    }

    ++m_NextRecordIndex;
    if (m_NextRecordIndex != kExpectedTags.size())
        return MotorFaultDiagnosticConsumeResult::IN_PROGRESS;

    outDiagnostic = m_Diagnostic;
    Reset();
    return MotorFaultDiagnosticConsumeResult::COMPLETE;
}

void MotorFaultDiagnosticDecoder::Reset()
{
    m_Active = false;
    m_NextRecordIndex = 0;
    m_Diagnostic = {};
}

const char* MotorFaultDiagnosticOperationName(
    MotorFaultDiagnosticOperation operation)
{
    switch (operation)
    {
    case MotorFaultDiagnosticOperation::TRAJECTORY_DRIVE:
        return "TRAJECTORY_DRIVE";
    case MotorFaultDiagnosticOperation::TRAJECTORY_TURN:
        return "TRAJECTORY_TURN";
    case MotorFaultDiagnosticOperation::CORRECTION_DRIVE:
        return "CORRECTION_DRIVE";
    case MotorFaultDiagnosticOperation::CORRECTION_TURN:
        return "CORRECTION_TURN";
    default:
        return "UNKNOWN";
    }
}

const char* MotorFaultMotionModeName(MotorFaultMotionMode mode)
{
    switch (mode)
    {
    case MotorFaultMotionMode::FORWARD:
        return "FORWARD";
    case MotorFaultMotionMode::TURN_CW:
        return "TURN_CW";
    case MotorFaultMotionMode::TURN_CCW:
        return "TURN_CCW";
    default:
        return "UNKNOWN";
    }
}

const char* MotorFaultMotionProfileName(MotorFaultMotionProfile profile)
{
    switch (profile)
    {
    case MotorFaultMotionProfile::NORMAL:
        return "NORMAL";
    case MotorFaultMotionProfile::PHYSICAL_FLEET:
        return "PHYSICAL_FLEET";
    case MotorFaultMotionProfile::CORRECTION:
        return "CORRECTION";
    default:
        return "UNKNOWN";
    }
}

std::string FormatWheelMismatchDiagnostic(
    const WheelMismatchDiagnostic& diagnostic)
{
    std::ostringstream output;
    output << "WHEEL_MISMATCH"
           << " operation="
           << MotorFaultDiagnosticOperationName(diagnostic.operation)
           << " mode=" << MotorFaultMotionModeName(diagnostic.motionMode)
           << " profile="
           << MotorFaultMotionProfileName(diagnostic.motionProfile)
           << " targetLeft=" << diagnostic.leftTarget
           << " targetRight=" << diagnostic.rightTarget
           << " left=" << diagnostic.leftProgress
           << " right=" << diagnostic.rightProgress;
    return output.str();
}
