#include "VisionObservationRelay.hpp"

#include "MemoryStream.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <numbers>

namespace
{
    bool IsPoseTrackingState(RobotProtocol::VisionTrackingState state)
    {
        return state == RobotProtocol::VisionTrackingState::MEASURED ||
               state == RobotProtocol::VisionTrackingState::HELD;
    }

    uint32_t ClampReceiveAge(uint64_t ageMilliseconds)
    {
        return static_cast<uint32_t>(std::min<uint64_t>(
            ageMilliseconds,
            std::numeric_limits<uint32_t>::max()));
    }

    void WriteUInt32LittleEndian(OutputMemoryStream& stream, uint32_t value)
    {
        const uint8_t bytes[sizeof(uint32_t)]{
            static_cast<uint8_t>(value & 0xffU),
            static_cast<uint8_t>((value >> 8U) & 0xffU),
            static_cast<uint8_t>((value >> 16U) & 0xffU),
            static_cast<uint8_t>((value >> 24U) & 0xffU)
        };
        stream.Write(bytes, sizeof(bytes));
    }

    void WriteFloatLittleEndian(OutputMemoryStream& stream, float value)
    {
        static_assert(sizeof(float) == sizeof(uint32_t));
        WriteUInt32LittleEndian(stream, std::bit_cast<uint32_t>(value));
    }
}

UnityVisionObservationPayload BuildUnityVisionObservationPayload(
    const StoredVisionObservation& stored,
    const VisionObservationStoreConfig& config,
    uint64_t nowServerMilliseconds)
{
    UnityVisionObservationPayload payload;
    payload.agvID = stored.observation.agvID;
    payload.transportSequence = stored.observation.sequence;

    const bool receiveTimeIsValid =
        stored.receivedAtServerMilliseconds <= nowServerMilliseconds;
    const uint64_t receiveAge = receiveTimeIsValid
        ? nowServerMilliseconds - stored.receivedAtServerMilliseconds
        : std::numeric_limits<uint64_t>::max();
    payload.serverReceiveAgeMilliseconds = ClampReceiveAge(receiveAge);

    const bool isFresh =
        receiveTimeIsValid &&
        receiveAge <= config.maximumReceiveAgeMilliseconds;
    const bool hasUsablePose =
        isFresh &&
        IsPoseTrackingState(stored.observation.state) &&
        stored.observation.pose.has_value() &&
        config.map.IsValid();
    if (!hasUsablePose)
        return payload;

    const VisionMetricPose& metricPose = *stored.observation.pose;
    if (!std::isfinite(metricPose.xMillimeters) ||
        !std::isfinite(metricPose.zMillimeters) ||
        !std::isfinite(metricPose.headingDegrees))
    {
        return payload;
    }

    const double serverX =
        config.map.localOriginServerX +
        static_cast<double>(metricPose.xMillimeters) /
            config.map.millimetersPerServerUnit;
    const double serverZ =
        config.map.localOriginServerZ +
        static_cast<double>(metricPose.zMillimeters) /
            config.map.millimetersPerServerUnit;
    const double headingRadians =
        static_cast<double>(metricPose.headingDegrees) *
        std::numbers::pi_v<double> / 180.0;
    if (!std::isfinite(serverX) ||
        !std::isfinite(serverZ) ||
        !std::isfinite(headingRadians))
    {
        return payload;
    }

    payload.trackingState = stored.observation.state;
    payload.poseValid = true;
    payload.serverX = static_cast<float>(serverX);
    payload.serverZ = static_cast<float>(serverZ);
    payload.headingRadians = static_cast<float>(headingRadians);
    return payload;
}

bool WriteUnityVisionObservationPacket(
    OutputMemoryStream& stream,
    const UnityVisionObservationPayload& payload)
{
    const bool stateIsPose = IsPoseTrackingState(payload.trackingState);
    const bool poseIsFinite =
        std::isfinite(payload.serverX) &&
        std::isfinite(payload.serverZ) &&
        std::isfinite(payload.headingRadians);
    const bool canonicalLostPose =
        payload.serverX == 0.0f &&
        payload.serverZ == 0.0f &&
        payload.headingRadians == 0.0f;

    if (payload.agvID == 0 || payload.transportSequence == 0 ||
        (payload.poseValid && (!stateIsPose || !poseIsFinite)) ||
        (!payload.poseValid &&
         (payload.trackingState != RobotProtocol::VisionTrackingState::LOST ||
          !canonicalLostPose)))
    {
        return false;
    }

    stream.Write(static_cast<uint8_t>(UPT_VISION_OBSERVATION));
    WriteUInt32LittleEndian(stream, payload.agvID);
    WriteUInt32LittleEndian(stream, payload.transportSequence);
    stream.Write(static_cast<uint8_t>(payload.trackingState));
    stream.Write(static_cast<uint8_t>(payload.poseValid ? 1U : 0U));
    WriteFloatLittleEndian(stream, payload.serverX);
    WriteFloatLittleEndian(stream, payload.serverZ);
    WriteFloatLittleEndian(stream, payload.headingRadians);
    WriteUInt32LittleEndian(stream, payload.serverReceiveAgeMilliseconds);
    return true;
}
