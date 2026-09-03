#include "MotorFaultDiagnostic.hpp"

#include <cstdint>
#include <iostream>

namespace
{
    int g_Failures = 0;

    void Check(bool condition, const char* message)
    {
        if (condition) return;
        ++g_Failures;
        std::cerr << "[MotorFaultDiagnosticTest] FAIL: " << message << "\n";
    }

    MotorFaultDiagnosticConsumeResult Consume(
        MotorFaultDiagnosticDecoder& decoder,
        uint32_t detail,
        WheelMismatchDiagnostic& diagnostic)
    {
        return decoder.Consume(
            RobotProtocol::ErrorCode::MOTOR_FAULT, detail, diagnostic);
    }

    void TestCorrectionDriveSnapshot()
    {
        MotorFaultDiagnosticDecoder decoder;
        WheelMismatchDiagnostic diagnostic;

        Check(Consume(decoder, 65539u, diagnostic) ==
                  MotorFaultDiagnosticConsumeResult::WHEEL_MISMATCH_STARTED,
              "legacy WHEEL_MISMATCH did not start a diagnostic sequence");
        Check(Consume(decoder, 0xD0010312u, diagnostic) ==
                  MotorFaultDiagnosticConsumeResult::IN_PROGRESS,
              "CORRECTION_DRIVE context was rejected");
        Check(Consume(decoder, 0xD10000CDu, diagnostic) ==
                  MotorFaultDiagnosticConsumeResult::IN_PROGRESS,
              "left progress was rejected");
        Check(Consume(decoder, 0xD2000075u, diagnostic) ==
                  MotorFaultDiagnosticConsumeResult::IN_PROGRESS,
              "right progress was rejected");
        Check(Consume(decoder, 0xD30000D5u, diagnostic) ==
                  MotorFaultDiagnosticConsumeResult::IN_PROGRESS,
              "left target was rejected");
        Check(Consume(decoder, 0xD40000CBu, diagnostic) ==
                  MotorFaultDiagnosticConsumeResult::COMPLETE,
              "right target did not complete the diagnostic sequence");

        Check(diagnostic.operation ==
                  MotorFaultDiagnosticOperation::CORRECTION_DRIVE,
              "operation was not decoded as CORRECTION_DRIVE");
        Check(diagnostic.motionMode == MotorFaultMotionMode::FORWARD,
              "motion mode was not decoded as FORWARD");
        Check(diagnostic.motionProfile ==
                  MotorFaultMotionProfile::CORRECTION,
              "motion profile was not decoded as CORRECTION");
        Check(diagnostic.leftProgress == 205 &&
                  diagnostic.rightProgress == 117,
              "wheel progress counts changed during decoding");
        Check(diagnostic.leftTarget == 213 &&
                  diagnostic.rightTarget == 203,
              "wheel target counts changed during decoding");
        Check(FormatWheelMismatchDiagnostic(diagnostic) ==
                  "WHEEL_MISMATCH operation=CORRECTION_DRIVE mode=FORWARD "
                  "profile=CORRECTION targetLeft=213 targetRight=203 "
                  "left=205 right=117",
              "Server diagnostic log text is incomplete or ambiguous");
    }

    void TestSignedCountsAndSequenceGuards()
    {
        MotorFaultDiagnosticDecoder decoder;
        WheelMismatchDiagnostic diagnostic;

        Check(Consume(decoder, 0xD1000001u, diagnostic) ==
                  MotorFaultDiagnosticConsumeResult::MALFORMED,
              "orphan tagged detail was accepted");
        Check(Consume(decoder, 65539u, diagnostic) ==
                  MotorFaultDiagnosticConsumeResult::WHEEL_MISMATCH_STARTED,
              "second legacy WHEEL_MISMATCH did not start decoding");
        Check(Consume(decoder, 0xD0010432u, diagnostic) ==
                  MotorFaultDiagnosticConsumeResult::IN_PROGRESS,
              "CORRECTION_TURN context was rejected");
        Check(Consume(decoder, 0xD1FFFFFBu, diagnostic) ==
                  MotorFaultDiagnosticConsumeResult::IN_PROGRESS,
              "negative signed left progress was rejected");
        Check(Consume(decoder, 0xD2000007u, diagnostic) ==
                  MotorFaultDiagnosticConsumeResult::IN_PROGRESS,
              "right progress was rejected in signed-count test");
        Check(Consume(decoder, 0xD3FFFFF5u, diagnostic) ==
                  MotorFaultDiagnosticConsumeResult::IN_PROGRESS,
              "negative signed left target was rejected");
        Check(Consume(decoder, 0xD400000Du, diagnostic) ==
                  MotorFaultDiagnosticConsumeResult::COMPLETE,
              "signed-count sequence did not complete");
        Check(diagnostic.leftProgress == -5 &&
                  diagnostic.rightProgress == 7 &&
                  diagnostic.leftTarget == -11 &&
                  diagnostic.rightTarget == 13,
              "signed 24-bit counts were decoded incorrectly");

        Check(Consume(decoder, 65539u, diagnostic) ==
                  MotorFaultDiagnosticConsumeResult::WHEEL_MISMATCH_STARTED,
              "sequence-order test did not start");
        Check(Consume(decoder, 0xD1000001u, diagnostic) ==
                  MotorFaultDiagnosticConsumeResult::MALFORMED,
              "out-of-order diagnostic tag was accepted");
    }
}

int main()
{
    TestCorrectionDriveSnapshot();
    TestSignedCountsAndSequenceGuards();

    if (g_Failures != 0)
    {
        std::cerr << "[MotorFaultDiagnosticTest] FAILED checks="
                  << g_Failures << "\n";
        return 1;
    }

    std::cout << "[MotorFaultDiagnosticTest] PASS WHEEL_MISMATCH tagged diagnostics\n";
    return 0;
}
