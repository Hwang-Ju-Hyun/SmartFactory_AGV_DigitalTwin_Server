#pragma once

#include <cmath>
#include <cstdint>
#include <numbers>
#include <optional>
#include <string>

struct PhysicalFleetHeadingAnchor
{
    float headingRad = 0.0f;
    uint32_t visionSequence = 0;
};

struct ResolvedPhysicalFleetHeading
{
    float headingRad = 0.0f;
    bool usedVision = false;
    uint32_t visionSequence = 0;
};

struct PhysicalFleetVisionHeadingCandidate
{
    uint32_t agvID = 0;
    uint32_t nodeID = 0;
    uint32_t sourceID = 0;
    uint64_t sessionID = 0;
    uint32_t visionSequence = 0;
    uint64_t receivedAtMilliseconds = 0;
    float headingRad = 0.0f;
    std::string calibrationID;
    bool measuredAndVerified = false;
};

inline std::optional<PhysicalFleetHeadingAnchor>
ValidatePhysicalFleetVisionHeading(
    const PhysicalFleetVisionHeadingCandidate& candidate,
    uint32_t expectedAgvID,
    uint32_t expectedNodeID,
    const std::string& expectedCalibrationID,
    uint32_t currentSourceID,
    uint64_t currentSessionID,
    uint64_t nowMilliseconds,
    uint64_t maximumAgeMilliseconds)
{
    if (!candidate.measuredAndVerified ||
        candidate.agvID != expectedAgvID ||
        candidate.nodeID != expectedNodeID ||
        candidate.sourceID == 0 ||
        candidate.sourceID != currentSourceID ||
        candidate.sessionID == 0 ||
        candidate.sessionID != currentSessionID ||
        candidate.visionSequence == 0 ||
        candidate.calibrationID != expectedCalibrationID ||
        candidate.receivedAtMilliseconds > nowMilliseconds ||
        nowMilliseconds - candidate.receivedAtMilliseconds >
            maximumAgeMilliseconds ||
        !std::isfinite(candidate.headingRad))
    {
        return std::nullopt;
    }
    return PhysicalFleetHeadingAnchor{
        candidate.headingRad,
        candidate.visionSequence
    };
}

inline ResolvedPhysicalFleetHeading ResolvePhysicalFleetHeading(
    float nominalHeadingRad,
    const std::optional<PhysicalFleetHeadingAnchor>& visionAnchor)
{
    ResolvedPhysicalFleetHeading resolved;
    if (visionAnchor.has_value() &&
        visionAnchor->visionSequence != 0 &&
        std::isfinite(visionAnchor->headingRad))
    {
        resolved.headingRad = std::remainder(
            visionAnchor->headingRad,
            2.0f * std::numbers::pi_v<float>);
        resolved.usedVision = true;
        resolved.visionSequence = visionAnchor->visionSequence;
        return resolved;
    }

    resolved.headingRad = std::remainder(
        nominalHeadingRad,
        2.0f * std::numbers::pi_v<float>);
    return resolved;
}
