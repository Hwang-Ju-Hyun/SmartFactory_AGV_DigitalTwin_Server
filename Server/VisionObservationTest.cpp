#include "PacketSerializer.hpp"
#include "VisionObservationStore.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#define REQUIRE(expression)                                                     \
    do                                                                          \
    {                                                                           \
        if (!(expression))                                                      \
            throw std::runtime_error(                                           \
                std::string("requirement failed: ") + #expression);            \
    } while (false)

namespace
{
    template <typename Payload, typename Writer, typename Reader>
    Payload RoundTrip(const Payload& source, Writer writer, Reader reader)
    {
        OutputMemoryStream output;
        REQUIRE(writer(output, source));
        InputMemoryStream input(
            const_cast<char*>(output.GetBuffer()), output.GetLength());
        Payload decoded;
        REQUIRE(reader(input, decoded));
        REQUIRE(input.GetRemainDataSize() == 0);
        return decoded;
    }

    RobotProtocol::VisionObservationPayload MakeMeasuredPayload()
    {
        RobotProtocol::VisionObservationPayload payload;
        payload.sourceTimestampUs = 0x123456789abcdef0ULL;
        payload.reportedAgeMs = 80;
        payload.state = RobotProtocol::VisionTrackingState::MEASURED;
        payload.pose = RobotProtocol::VisionPose{100.0f, 150.0f, -45.0f};
        payload.calibrationID = "calibration-2026-08";
        payload.verificationState =
            RobotProtocol::VisionVerificationState::VERIFIED;
        payload.quality.qualityFields =
            RobotProtocol::VISION_QUALITY_DECISION_MARGIN |
            RobotProtocol::VISION_QUALITY_CALIBRATION_RMS_ERROR |
            RobotProtocol::VISION_QUALITY_VERIFICATION_REFERENCE_COUNT |
            RobotProtocol::VISION_QUALITY_VERIFICATION_RMS_ERROR |
            RobotProtocol::VISION_QUALITY_VERIFICATION_MAX_ERROR |
            RobotProtocol::VISION_QUALITY_VERIFICATION_COVERAGE |
            RobotProtocol::VISION_QUALITY_VERIFICATION_AGE;
        payload.quality.decisionMargin = 42.0f;
        payload.quality.calibrationRmsErrorMm = 1.5f;
        payload.quality.verificationReferenceCount = 6;
        payload.quality.verificationRmsErrorMm = 2.0f;
        payload.quality.verificationMaxErrorMm = 3.0f;
        payload.quality.verificationCoverageRatio = 0.75f;
        payload.quality.verificationAgeMs = 20;
        return payload;
    }

    VisionObservationStoreConfig MakeStoreConfig()
    {
        VisionObservationStoreConfig config;
        config.map.millimetersPerServerUnit = 50.0;
        config.map.localOriginServerX = 50.0;
        config.map.localOriginServerZ = -36.0;
        config.map.minimumServerX = 50.0;
        config.map.maximumServerX = 66.0;
        config.map.minimumServerZ = -36.0;
        config.map.maximumServerZ = -28.0;
        config.map.allowedMarginMillimeters = 100.0;
        config.expectedCalibrationID = "calibration-2026-08";
        config.expectedMapContractID = "dd2c1523295b02ee";
        config.expectedPoseContractID = "f84eb43ebb6cf7ff";
        config.expectedSourceID = 1;
        return config;
    }

    VisionObservationInput MakeStoreInput(uint32_t agvID, uint32_t sequence)
    {
        const auto wire = MakeMeasuredPayload();
        VisionObservationInput input;
        input.agvID = agvID;
        input.sourceID = 1;
        input.sessionID = 77;
        input.sequence = sequence;
        input.sourceTimestampMicroseconds = wire.sourceTimestampUs;
        input.reportedAgeMilliseconds = wire.reportedAgeMs;
        input.state = wire.state;
        input.pose = VisionMetricPose{
            wire.pose->xMm, wire.pose->zMm, wire.pose->headingDeg};
        input.calibrationID = wire.calibrationID;
        input.mapContractID = "dd2c1523295b02ee";
        input.poseContractID = "f84eb43ebb6cf7ff";
        input.verificationState = wire.verificationState;
        input.quality = wire.quality;
        return input;
    }

    bool IsRegisteredAgv(uint32_t agvID)
    {
        return agvID == 1 || agvID == 2;
    }

    void TestSerializerRoundTrips()
    {
        RobotProtocol::VisionHelloPayload hello;
        hello.protocolVersion = RobotProtocol::kProtocolVersion;
        hello.sourceID = 1;
        hello.sessionID = 0xfedcba9876543210ULL;
        hello.mapContractID = "dd2c1523295b02ee";
        hello.poseContractID = "f84eb43ebb6cf7ff";
        const auto helloDecoded = RoundTrip(
            hello,
            RobotProtocol::WriteVisionHelloPayload,
            RobotProtocol::ReadVisionHelloPayload);
        REQUIRE(helloDecoded.sessionID == hello.sessionID);
        REQUIRE(helloDecoded.mapContractID == hello.mapContractID);

        RobotProtocol::VisionHelloAckPayload ack;
        ack.accepted = 1;
        ack.rejectionReason =
            RobotProtocol::VisionHelloRejectionReason::NONE;
        ack.sourceID = hello.sourceID;
        ack.sessionID = hello.sessionID;
        const auto ackDecoded = RoundTrip(
            ack,
            RobotProtocol::WriteVisionHelloAckPayload,
            RobotProtocol::ReadVisionHelloAckPayload);
        REQUIRE(ackDecoded.accepted == 1);
        REQUIRE(ackDecoded.sessionID == hello.sessionID);

        const auto measured = MakeMeasuredPayload();
        const auto measuredDecoded = RoundTrip(
            measured,
            RobotProtocol::WriteVisionObservationPayload,
            RobotProtocol::ReadVisionObservationPayload);
        REQUIRE(measuredDecoded.pose.has_value());
        REQUIRE(measuredDecoded.sourceTimestampUs == measured.sourceTimestampUs);
        REQUIRE(measuredDecoded.quality.verificationReferenceCount == 6);

        auto held = measured;
        held.state = RobotProtocol::VisionTrackingState::HELD;
        held.reportedAgeMs = 200;
        held.verificationState = RobotProtocol::VisionVerificationState::STALE;
        const auto heldDecoded = RoundTrip(
            held,
            RobotProtocol::WriteVisionObservationPayload,
            RobotProtocol::ReadVisionObservationPayload);
        REQUIRE(heldDecoded.pose.has_value());
        REQUIRE(heldDecoded.state == RobotProtocol::VisionTrackingState::HELD);

        auto lost = measured;
        lost.state = RobotProtocol::VisionTrackingState::LOST;
        lost.pose.reset();
        lost.verificationState =
            RobotProtocol::VisionVerificationState::REFERENCES_MISSING;
        const auto lostDecoded = RoundTrip(
            lost,
            RobotProtocol::WriteVisionObservationPayload,
            RobotProtocol::ReadVisionObservationPayload);
        REQUIRE(!lostDecoded.pose.has_value());
    }

    void TestSerializerRejectsMalformedPayloads()
    {
        const auto measured = MakeMeasuredPayload();
        OutputMemoryStream output;
        REQUIRE(RobotProtocol::WriteVisionObservationPayload(output, measured));

        std::vector<char> trailing(
            output.GetBuffer(), output.GetBuffer() + output.GetLength());
        trailing.push_back('\0');
        InputMemoryStream trailingInput(trailing.data(), trailing.size());
        RobotProtocol::VisionObservationPayload decoded;
        REQUIRE(!RobotProtocol::ReadVisionObservationPayload(
            trailingInput, decoded));

        std::vector<char> truncated(
            output.GetBuffer(), output.GetBuffer() + output.GetLength() - 1);
        InputMemoryStream truncatedInput(truncated.data(), truncated.size());
        REQUIRE(!RobotProtocol::ReadVisionObservationPayload(
            truncatedInput, decoded));

        auto invalidStatePose = measured;
        invalidStatePose.state = RobotProtocol::VisionTrackingState::LOST;
        REQUIRE(!RobotProtocol::WriteVisionObservationPayload(
            output, invalidStatePose));

        REQUIRE(RobotProtocol::IsKnownVisionPacketID(
            static_cast<uint16_t>(RobotProtocol::PacketID::VISION_HELLO)));
        REQUIRE(!RobotProtocol::IsKnownRobotPacketID(
            static_cast<uint16_t>(RobotProtocol::PacketID::VISION_HELLO)));
    }

    void TestGoldenMeasuredBodyLayout()
    {
        RobotProtocol::VisionObservationPayload payload;
        payload.sourceTimestampUs = 0x0102030405060708ULL;
        payload.reportedAgeMs = 3;
        payload.state = RobotProtocol::VisionTrackingState::MEASURED;
        payload.pose = RobotProtocol::VisionPose{1.0f, -2.0f, 90.0f};
        payload.calibrationID = "cal";
        payload.verificationState =
            RobotProtocol::VisionVerificationState::VERIFIED;

        OutputMemoryStream body;
        RobotProtocol::WritePacketBodyHeader(
            body, RobotProtocol::PacketID::VISION_OBSERVATION, 1, 2);
        REQUIRE(RobotProtocol::WriteVisionObservationPayload(body, payload));

        const std::vector<uint8_t> expected{
            0x5a, 0x02,
            0x01, 0x00, 0x00, 0x00,
            0x02, 0x00, 0x00, 0x00,
            0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
            0x03, 0x00, 0x00, 0x00,
            0x01,
            0x00, 0x00, 0x80, 0x3f,
            0x00, 0x00, 0x00, 0xc0,
            0x00, 0x00, 0xb4, 0x42,
            0x03, 0x00, 0x63, 0x61, 0x6c,
            0x01,
            0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00
        };
        REQUIRE(body.GetLength() == expected.size());
        REQUIRE(std::equal(
            expected.begin(), expected.end(),
            reinterpret_cast<const uint8_t*>(body.GetBuffer())));

        InputMemoryStream input(
            const_cast<char*>(body.GetBuffer()), body.GetLength());
        RobotProtocol::PacketBodyHeader header;
        REQUIRE(RobotProtocol::ReadPacketBodyHeader(input, header));
        REQUIRE(header.packetID == static_cast<uint16_t>(
            RobotProtocol::PacketID::VISION_OBSERVATION));
        REQUIRE(header.agvID == 1 && header.sequence == 2);
        RobotProtocol::VisionObservationPayload decoded;
        REQUIRE(RobotProtocol::ReadVisionObservationPayload(input, decoded));

        OutputMemoryStream lostBody;
        RobotProtocol::WritePacketBodyHeader(
            lostBody, RobotProtocol::PacketID::VISION_OBSERVATION, 1, 3);
        payload.state = RobotProtocol::VisionTrackingState::LOST;
        payload.pose.reset();
        payload.verificationState =
            RobotProtocol::VisionVerificationState::REFERENCES_MISSING;
        REQUIRE(RobotProtocol::WriteVisionObservationPayload(
            lostBody, payload));
        REQUIRE(lostBody.GetLength() + 3 * sizeof(float) == body.GetLength());
    }

    void TestObservationStoreValidationAndIsolation()
    {
        VisionObservationStore store(MakeStoreConfig());
        auto first = MakeStoreInput(1, 1);
        REQUIRE(store.TryStore(first, 1000, 1000, IsRegisteredAgv) ==
               VisionObservationStoreResult::ACCEPTED);
        REQUIRE(store.GetLatest(1).has_value());
        REQUIRE(store.GetLatest(1)->observation.pose.has_value());

        REQUIRE(store.TryStore(first, 1001, 1001, IsRegisteredAgv) ==
               VisionObservationStoreResult::DUPLICATE_OR_OUT_OF_ORDER_SEQUENCE);

        auto unknown = MakeStoreInput(99, 2);
        REQUIRE(store.TryStore(unknown, 1002, 1002, IsRegisteredAgv) ==
               VisionObservationStoreResult::UNKNOWN_AGV);
        auto secondAgv = MakeStoreInput(2, 2);
        REQUIRE(store.TryStore(secondAgv, 1002, 1002, IsRegisteredAgv) ==
               VisionObservationStoreResult::ACCEPTED);
        REQUIRE(store.Size() == 2);

        auto stale = MakeStoreInput(1, 3);
        stale.reportedAgeMilliseconds = 101;
        REQUIRE(store.TryStore(stale, 1003, 1003, IsRegisteredAgv) ==
               VisionObservationStoreResult::STALE_REPORTED_AGE);

        auto nonFinite = MakeStoreInput(1, 3);
        nonFinite.pose->xMillimeters =
            std::numeric_limits<float>::quiet_NaN();
        REQUIRE(store.TryStore(nonFinite, 1003, 1003, IsRegisteredAgv) ==
               VisionObservationStoreResult::NON_FINITE_POSE);

        auto outside = MakeStoreInput(1, 3);
        outside.pose->xMillimeters = 1000.1f;
        REQUIRE(store.TryStore(outside, 1003, 1003, IsRegisteredAgv) ==
               VisionObservationStoreResult::OUT_OF_MAP);

        auto wrongContract = MakeStoreInput(1, 3);
        wrongContract.mapContractID = "wrong-map";
        REQUIRE(store.TryStore(wrongContract, 1003, 1003, IsRegisteredAgv) ==
               VisionObservationStoreResult::MAP_CONTRACT_ID_MISMATCH);

        auto lost = MakeStoreInput(1, 3);
        lost.state = RobotProtocol::VisionTrackingState::LOST;
        lost.pose.reset();
        lost.reportedAgeMilliseconds = 9999;
        lost.verificationState =
            RobotProtocol::VisionVerificationState::REFERENCES_MISSING;
        REQUIRE(store.TryStore(lost, 1003, 1003, IsRegisteredAgv) ==
               VisionObservationStoreResult::ACCEPTED);
        REQUIRE(!store.GetLatest(1)->observation.pose.has_value());

        auto newSession = MakeStoreInput(1, 1);
        newSession.sessionID = 78;
        REQUIRE(store.TryStore(newSession, 1004, 1004, IsRegisteredAgv) ==
               VisionObservationStoreResult::ACCEPTED);
        REQUIRE(store.IsLatestWithinReceiveTimeout(1, 1504));
        REQUIRE(!store.IsLatestWithinReceiveTimeout(1, 1505));
    }

    void TestObservationStoreBoundaries()
    {
        {
            VisionObservationStore store(MakeStoreConfig());
            auto input = MakeStoreInput(1, 0);
            REQUIRE(store.TryStore(input, 1000, 1000, IsRegisteredAgv) ==
                    VisionObservationStoreResult::INVALID_SEQUENCE);
            input.sequence = 1;
            input.sourceID = 2;
            REQUIRE(store.TryStore(input, 1000, 1000, IsRegisteredAgv) ==
                    VisionObservationStoreResult::SOURCE_ID_MISMATCH);
            input.sourceID = 1;
            input.sessionID = 0;
            REQUIRE(store.TryStore(input, 1000, 1000, IsRegisteredAgv) ==
                    VisionObservationStoreResult::INVALID_SESSION_ID);
        }

        {
            VisionObservationStore store(MakeStoreConfig());
            auto input = MakeStoreInput(1, 1);
            input.calibrationID = "wrong-calibration";
            REQUIRE(store.TryStore(input, 1000, 1000, IsRegisteredAgv) ==
                    VisionObservationStoreResult::CALIBRATION_ID_MISMATCH);
            input = MakeStoreInput(1, 1);
            input.poseContractID = "wrong-pose";
            REQUIRE(store.TryStore(input, 1000, 1000, IsRegisteredAgv) ==
                    VisionObservationStoreResult::POSE_CONTRACT_ID_MISMATCH);
            input = MakeStoreInput(1, 1);
            input.verificationState =
                RobotProtocol::VisionVerificationState::STALE;
            REQUIRE(store.TryStore(input, 1000, 1000, IsRegisteredAgv) ==
                    VisionObservationStoreResult::MEASURED_NOT_VERIFIED);
        }

        {
            VisionObservationStore store(MakeStoreConfig());
            auto held = MakeStoreInput(1, 1);
            held.state = RobotProtocol::VisionTrackingState::HELD;
            held.verificationState = RobotProtocol::VisionVerificationState::STALE;
            held.reportedAgeMilliseconds = 200;
            REQUIRE(store.TryStore(held, 1000, 1000, IsRegisteredAgv) ==
                    VisionObservationStoreResult::ACCEPTED);
            held.sequence = 2;
            held.reportedAgeMilliseconds = 201;
            REQUIRE(store.TryStore(held, 1001, 1001, IsRegisteredAgv) ==
                    VisionObservationStoreResult::STALE_REPORTED_AGE);
            REQUIRE(store.GetLatest(1)->observation.sequence == 1);
        }

        {
            VisionObservationStore store(MakeStoreConfig());
            auto input = MakeStoreInput(1, 1);
            REQUIRE(store.TryStore(input, 1001, 1000, IsRegisteredAgv) ==
                    VisionObservationStoreResult::RECEIVE_TIME_IN_FUTURE);
            REQUIRE(store.TryStore(input, 1000, 1501, IsRegisteredAgv) ==
                    VisionObservationStoreResult::STALE_RECEIVE_TIME);
        }

        {
            VisionObservationStore store(MakeStoreConfig());
            auto input = MakeStoreInput(1, 1);
            input.pose->headingDegrees = -180.0f;
            input.pose->xMillimeters = -100.0f;
            input.pose->zMillimeters = -100.0f;
            REQUIRE(store.TryStore(input, 1000, 1000, IsRegisteredAgv) ==
                    VisionObservationStoreResult::ACCEPTED);
            input.sequence = 2;
            input.pose->headingDegrees = 179.999f;
            input.pose->xMillimeters = 900.0f;
            input.pose->zMillimeters = 500.0f;
            REQUIRE(store.TryStore(input, 1001, 1001, IsRegisteredAgv) ==
                    VisionObservationStoreResult::ACCEPTED);
            input.sequence = 3;
            input.pose->headingDegrees = 180.0f;
            REQUIRE(store.TryStore(input, 1002, 1002, IsRegisteredAgv) ==
                    VisionObservationStoreResult::HEADING_OUT_OF_RANGE);
        }

        {
            VisionObservationStore store(MakeStoreConfig());
            auto input = MakeStoreInput(1, 1);
            input.quality.qualityFields = RobotProtocol::VISION_QUALITY_NONE;
            input.quality.decisionMargin = 1.0f;
            REQUIRE(store.TryStore(input, 1000, 1000, IsRegisteredAgv) ==
                    VisionObservationStoreResult::INVALID_QUALITY_METADATA);
            input = MakeStoreInput(1, 1);
            input.quality.verificationCoverageRatio =
                std::numeric_limits<float>::infinity();
            REQUIRE(store.TryStore(input, 1000, 1000, IsRegisteredAgv) ==
                    VisionObservationStoreResult::INVALID_QUALITY_METADATA);
            input = MakeStoreInput(1, 1);
            input.quality.verificationMaxErrorMm = 1.0f;
            input.quality.verificationRmsErrorMm = 2.0f;
            REQUIRE(store.TryStore(input, 1000, 1000, IsRegisteredAgv) ==
                    VisionObservationStoreResult::INVALID_QUALITY_METADATA);
        }
    }
}

int main()
{
    TestSerializerRoundTrips();
    TestSerializerRejectsMalformedPayloads();
    TestGoldenMeasuredBodyLayout();
    TestObservationStoreValidationAndIsolation();
    TestObservationStoreBoundaries();
    std::cout << "Vision observation tests passed\n";
    return 0;
}
