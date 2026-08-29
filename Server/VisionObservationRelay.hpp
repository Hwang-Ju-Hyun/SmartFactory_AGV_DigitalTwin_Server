#pragma once

#include "VisionObservationStore.hpp"

#include <cstdint>

class OutputMemoryStream;

// Fixed-layout payload sent on the Unity legacy channel after the one-byte
// UPT_VISION_OBSERVATION discriminator.  poseValid is deliberately explicit:
// LOST and Server receive-timeout snapshots carry canonical zero pose fields.
struct UnityVisionObservationPayload
{
    uint32_t agvID = 0;
    uint32_t transportSequence = 0;
    RobotProtocol::VisionTrackingState trackingState =
        RobotProtocol::VisionTrackingState::LOST;
    bool poseValid = false;
    float serverX = 0.0f;
    float serverZ = 0.0f;
    float headingRadians = 0.0f;
    uint32_t serverReceiveAgeMilliseconds = 0;
};

constexpr uint32_t kUnityVisionObservationPacketBytes =
    sizeof(uint8_t) +
    sizeof(uint32_t) +
    sizeof(uint32_t) +
    sizeof(uint8_t) +
    sizeof(uint8_t) +
    sizeof(float) * 3U +
    sizeof(uint32_t);

// Converts local Vision millimetres to the active Server map coordinate frame.
// The stored observation remains read-only and no authoritative AGV state is
// referenced or changed by this helper.
UnityVisionObservationPayload BuildUnityVisionObservationPayload(
    const StoredVisionObservation& stored,
    const VisionObservationStoreConfig& config,
    uint64_t nowServerMilliseconds);

// Writes the complete Unity packet body (one-byte type plus fixed payload).
// TCPSession adds the outer uint16 frame size.
bool WriteUnityVisionObservationPacket(
    OutputMemoryStream& stream,
    const UnityVisionObservationPayload& payload);
