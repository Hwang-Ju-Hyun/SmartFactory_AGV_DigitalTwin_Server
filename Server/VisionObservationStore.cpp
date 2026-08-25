#include "VisionObservationStore.hpp"

#include "Map.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace
{
    constexpr uint16_t kKnownVisionQualityFields =
        RobotProtocol::VISION_QUALITY_DECISION_MARGIN |
        RobotProtocol::VISION_QUALITY_CALIBRATION_RMS_ERROR |
        RobotProtocol::VISION_QUALITY_VERIFICATION_REFERENCE_COUNT |
        RobotProtocol::VISION_QUALITY_VERIFICATION_RMS_ERROR |
        RobotProtocol::VISION_QUALITY_VERIFICATION_MAX_ERROR |
        RobotProtocol::VISION_QUALITY_VERIFICATION_COVERAGE |
        RobotProtocol::VISION_QUALITY_VERIFICATION_AGE;

    bool IsIdentityValid(std::string_view identity)
    {
        if (identity.empty() ||
            identity.size() > RobotProtocol::kMaxVisionIdentityBytes)
            return false;

        return std::all_of(identity.begin(), identity.end(), [](char value)
        {
            const unsigned char character = static_cast<unsigned char>(value);
            return character >= 0x21 && character <= 0x7e;
        });
    }

    void ValidateExpectedIdentity(std::string_view identity,
                                  std::string_view fieldName)
    {
        if (!IsIdentityValid(identity))
        {
            throw std::invalid_argument(
                std::string(fieldName) +
                " must contain 1-" +
                std::to_string(RobotProtocol::kMaxVisionIdentityBytes) +
                " visible ASCII characters");
        }
    }

    bool HasQualityField(uint16_t fields,
                         RobotProtocol::VisionQualityField field)
    {
        return (fields & static_cast<uint16_t>(field)) != 0;
    }

    bool IsNonNegativeFinite(float value)
    {
        return std::isfinite(value) && value >= 0.0f;
    }

    bool IsQualityMetadataValid(
        const RobotProtocol::VisionQualityMetadata& quality)
    {
        if ((quality.qualityFields &
             static_cast<uint16_t>(~kKnownVisionQualityFields)) != 0)
        {
            return false;
        }

        const bool hasVerificationRms = HasQualityField(
            quality.qualityFields,
            RobotProtocol::VISION_QUALITY_VERIFICATION_RMS_ERROR);
        const bool hasVerificationMax = HasQualityField(
            quality.qualityFields,
            RobotProtocol::VISION_QUALITY_VERIFICATION_MAX_ERROR);
        if ((!HasQualityField(
                 quality.qualityFields,
                 RobotProtocol::VISION_QUALITY_DECISION_MARGIN) &&
             quality.decisionMargin != 0.0f) ||
            (!HasQualityField(
                 quality.qualityFields,
                 RobotProtocol::VISION_QUALITY_CALIBRATION_RMS_ERROR) &&
             quality.calibrationRmsErrorMm != 0.0f) ||
            (!HasQualityField(
                 quality.qualityFields,
                 RobotProtocol::VISION_QUALITY_VERIFICATION_REFERENCE_COUNT) &&
             quality.verificationReferenceCount != 0) ||
            (!hasVerificationRms && quality.verificationRmsErrorMm != 0.0f) ||
            (!hasVerificationMax && quality.verificationMaxErrorMm != 0.0f) ||
            (!HasQualityField(
                 quality.qualityFields,
                 RobotProtocol::VISION_QUALITY_VERIFICATION_COVERAGE) &&
             quality.verificationCoverageRatio != 0.0f) ||
            (!HasQualityField(
                 quality.qualityFields,
                 RobotProtocol::VISION_QUALITY_VERIFICATION_AGE) &&
             quality.verificationAgeMs != 0))
        {
            return false;
        }

        if (!IsNonNegativeFinite(quality.decisionMargin) ||
            !IsNonNegativeFinite(quality.calibrationRmsErrorMm) ||
            !IsNonNegativeFinite(quality.verificationRmsErrorMm) ||
            !IsNonNegativeFinite(quality.verificationMaxErrorMm))
        {
            return false;
        }

        if (!std::isfinite(quality.verificationCoverageRatio) ||
            quality.verificationCoverageRatio < 0.0f ||
            quality.verificationCoverageRatio > 1.0f)
        {
            return false;
        }

        if (hasVerificationRms && hasVerificationMax &&
            quality.verificationMaxErrorMm < quality.verificationRmsErrorMm)
        {
            return false;
        }

        return true;
    }
}

VisionMapCoordinateContract VisionMapCoordinateContract::FromCanonicalMap(
    double millimetersPerServerUnit,
    double localOriginServerX,
    double localOriginServerZ,
    double allowedMarginMillimeters)
{
    VisionMapCoordinateContract contract;
    contract.millimetersPerServerUnit = millimetersPerServerUnit;
    contract.localOriginServerX = localOriginServerX;
    contract.localOriginServerZ = localOriginServerZ;
    contract.allowedMarginMillimeters = allowedMarginMillimeters;

    const auto nodes = MapManager::GetInstance().GetNodes();
    if (nodes.empty())
        throw std::invalid_argument("canonical map must contain at least one node");

    contract.minimumServerX = std::numeric_limits<double>::infinity();
    contract.maximumServerX = -std::numeric_limits<double>::infinity();
    contract.minimumServerZ = std::numeric_limits<double>::infinity();
    contract.maximumServerZ = -std::numeric_limits<double>::infinity();

    for (const auto& [nodeID, node] : nodes)
    {
        (void)nodeID;
        if (!std::isfinite(node.m_PosX) || !std::isfinite(node.m_PosZ))
            throw std::invalid_argument("canonical map contains a non-finite node");

        contract.minimumServerX = std::min(
            contract.minimumServerX, static_cast<double>(node.m_PosX));
        contract.maximumServerX = std::max(
            contract.maximumServerX, static_cast<double>(node.m_PosX));
        contract.minimumServerZ = std::min(
            contract.minimumServerZ, static_cast<double>(node.m_PosZ));
        contract.maximumServerZ = std::max(
            contract.maximumServerZ, static_cast<double>(node.m_PosZ));
    }

    if (!contract.IsValid())
        throw std::invalid_argument("canonical map coordinate contract is invalid");

    return contract;
}

bool VisionMapCoordinateContract::IsValid() const
{
    return std::isfinite(millimetersPerServerUnit) &&
           millimetersPerServerUnit > 0.0 &&
           std::isfinite(localOriginServerX) &&
           std::isfinite(localOriginServerZ) &&
           std::isfinite(minimumServerX) &&
           std::isfinite(maximumServerX) &&
           std::isfinite(minimumServerZ) &&
           std::isfinite(maximumServerZ) &&
           minimumServerX <= maximumServerX &&
           minimumServerZ <= maximumServerZ &&
           std::isfinite(allowedMarginMillimeters) &&
           allowedMarginMillimeters >= 0.0;
}

bool VisionMapCoordinateContract::ContainsLocalMillimeters(
    double xMillimeters,
    double zMillimeters) const
{
    if (!IsValid() || !std::isfinite(xMillimeters) ||
        !std::isfinite(zMillimeters))
    {
        return false;
    }

    const double serverX =
        localOriginServerX + xMillimeters / millimetersPerServerUnit;
    const double serverZ =
        localOriginServerZ + zMillimeters / millimetersPerServerUnit;
    const double marginServerUnits =
        allowedMarginMillimeters / millimetersPerServerUnit;

    return serverX >= minimumServerX - marginServerUnits &&
           serverX <= maximumServerX + marginServerUnits &&
           serverZ >= minimumServerZ - marginServerUnits &&
           serverZ <= maximumServerZ + marginServerUnits;
}

VisionObservationStore::VisionObservationStore(
    VisionObservationStoreConfig config)
    : m_Config(std::move(config))
{
    if (!m_Config.map.IsValid())
        throw std::invalid_argument("Vision map coordinate contract is invalid");

    ValidateExpectedIdentity(
        m_Config.expectedCalibrationID, "expectedCalibrationID");
    ValidateExpectedIdentity(
        m_Config.expectedMapContractID, "expectedMapContractID");
    ValidateExpectedIdentity(
        m_Config.expectedPoseContractID, "expectedPoseContractID");

    if (m_Config.expectedSourceID.has_value() &&
        *m_Config.expectedSourceID == 0)
    {
        throw std::invalid_argument("expectedSourceID must not be zero");
    }

    if (m_Config.maximumMeasuredReportedAgeMilliseconds >
        m_Config.maximumHeldReportedAgeMilliseconds)
    {
        throw std::invalid_argument(
            "MEASURED age limit must not exceed HELD age limit");
    }
}

VisionObservationStoreResult VisionObservationStore::TryStore(
    const VisionObservationInput& observation,
    uint64_t receivedAtServerMilliseconds,
    uint64_t nowServerMilliseconds,
    const IsRegisteredAgvCallback& isRegisteredAgv)
{
    if (!isRegisteredAgv)
        return VisionObservationStoreResult::AGV_REGISTRATION_CHECK_UNAVAILABLE;
    if (!isRegisteredAgv(observation.agvID))
        return VisionObservationStoreResult::UNKNOWN_AGV;

    if (observation.sourceID == 0)
        return VisionObservationStoreResult::INVALID_SOURCE_ID;
    if (m_Config.expectedSourceID.has_value() &&
        observation.sourceID != *m_Config.expectedSourceID)
    {
        return VisionObservationStoreResult::SOURCE_ID_MISMATCH;
    }
    if (observation.sessionID == 0)
        return VisionObservationStoreResult::INVALID_SESSION_ID;
    if (observation.sequence == 0)
        return VisionObservationStoreResult::INVALID_SEQUENCE;

    switch (observation.state)
    {
    case RobotProtocol::VisionTrackingState::MEASURED:
    case RobotProtocol::VisionTrackingState::HELD:
    case RobotProtocol::VisionTrackingState::LOST:
        break;
    default:
        return VisionObservationStoreResult::INVALID_TRACKING_STATE;
    }

    switch (observation.verificationState)
    {
    case RobotProtocol::VisionVerificationState::UNKNOWN:
    case RobotProtocol::VisionVerificationState::VERIFIED:
    case RobotProtocol::VisionVerificationState::AWAITING_VERIFICATION:
    case RobotProtocol::VisionVerificationState::REFERENCES_MISSING:
    case RobotProtocol::VisionVerificationState::MISMATCH:
    case RobotProtocol::VisionVerificationState::STALE:
    case RobotProtocol::VisionVerificationState::INVALID:
        break;
    default:
        return VisionObservationStoreResult::INVALID_VERIFICATION_STATE;
    }

    if (observation.state == RobotProtocol::VisionTrackingState::MEASURED &&
        observation.verificationState !=
            RobotProtocol::VisionVerificationState::VERIFIED)
    {
        return VisionObservationStoreResult::MEASURED_NOT_VERIFIED;
    }

    if (!IsIdentityValid(observation.calibrationID))
        return VisionObservationStoreResult::INVALID_CALIBRATION_ID;
    if (observation.calibrationID != m_Config.expectedCalibrationID)
        return VisionObservationStoreResult::CALIBRATION_ID_MISMATCH;
    if (!IsIdentityValid(observation.mapContractID))
        return VisionObservationStoreResult::INVALID_MAP_CONTRACT_ID;
    if (observation.mapContractID != m_Config.expectedMapContractID)
        return VisionObservationStoreResult::MAP_CONTRACT_ID_MISMATCH;
    if (!IsIdentityValid(observation.poseContractID))
        return VisionObservationStoreResult::INVALID_POSE_CONTRACT_ID;
    if (observation.poseContractID != m_Config.expectedPoseContractID)
        return VisionObservationStoreResult::POSE_CONTRACT_ID_MISMATCH;

    const bool mustHavePose =
        observation.state == RobotProtocol::VisionTrackingState::MEASURED ||
        observation.state == RobotProtocol::VisionTrackingState::HELD;
    if (observation.pose.has_value() != mustHavePose)
        return VisionObservationStoreResult::INVALID_STATE_POSE_COMBINATION;

    if (observation.state == RobotProtocol::VisionTrackingState::MEASURED &&
        observation.reportedAgeMilliseconds >
            m_Config.maximumMeasuredReportedAgeMilliseconds)
    {
        return VisionObservationStoreResult::STALE_REPORTED_AGE;
    }
    if (observation.state == RobotProtocol::VisionTrackingState::HELD &&
        observation.reportedAgeMilliseconds >
            m_Config.maximumHeldReportedAgeMilliseconds)
    {
        return VisionObservationStoreResult::STALE_REPORTED_AGE;
    }
    // LOST is pose-free by contract.  Its reported age is diagnostic and is
    // intentionally not interpreted as the age of a current pose.

    if (receivedAtServerMilliseconds > nowServerMilliseconds)
        return VisionObservationStoreResult::RECEIVE_TIME_IN_FUTURE;
    if (nowServerMilliseconds - receivedAtServerMilliseconds >
        m_Config.maximumReceiveAgeMilliseconds)
    {
        return VisionObservationStoreResult::STALE_RECEIVE_TIME;
    }

    if (observation.pose.has_value())
    {
        const VisionMetricPose& pose = *observation.pose;
        if (!std::isfinite(pose.xMillimeters) ||
            !std::isfinite(pose.zMillimeters) ||
            !std::isfinite(pose.headingDegrees))
        {
            return VisionObservationStoreResult::NON_FINITE_POSE;
        }

        if (pose.headingDegrees < -180.0f || pose.headingDegrees >= 180.0f)
            return VisionObservationStoreResult::HEADING_OUT_OF_RANGE;

        if (!m_Config.map.ContainsLocalMillimeters(
                pose.xMillimeters, pose.zMillimeters))
        {
            return VisionObservationStoreResult::OUT_OF_MAP;
        }
    }

    if (!IsQualityMetadataValid(observation.quality))
        return VisionObservationStoreResult::INVALID_QUALITY_METADATA;

    const StreamKey streamKey{observation.sourceID, observation.sessionID};
    const auto sequenceIt = m_LastSequenceByStream.find(streamKey);
    if (sequenceIt != m_LastSequenceByStream.end() &&
        observation.sequence <= sequenceIt->second)
    {
        return VisionObservationStoreResult::DUPLICATE_OR_OUT_OF_ORDER_SEQUENCE;
    }

    m_LastSequenceByStream[streamKey] = observation.sequence;
    m_LatestByAgv[observation.agvID] = {
        observation,
        receivedAtServerMilliseconds
    };
    return VisionObservationStoreResult::ACCEPTED;
}

std::optional<StoredVisionObservation> VisionObservationStore::GetLatest(
    uint32_t agvID) const
{
    const auto it = m_LatestByAgv.find(agvID);
    if (it == m_LatestByAgv.end())
        return std::nullopt;
    return it->second;
}

bool VisionObservationStore::IsLatestWithinReceiveTimeout(
    uint32_t agvID,
    uint64_t nowServerMilliseconds) const
{
    const auto it = m_LatestByAgv.find(agvID);
    if (it == m_LatestByAgv.end())
        return false;

    const uint64_t receivedAt = it->second.receivedAtServerMilliseconds;
    if (receivedAt > nowServerMilliseconds)
        return false;

    return nowServerMilliseconds - receivedAt <=
           m_Config.maximumReceiveAgeMilliseconds;
}

std::size_t VisionObservationStore::StreamKeyHash::operator()(
    const StreamKey& key) const noexcept
{
    const std::size_t sourceHash = std::hash<uint32_t>{}(key.sourceID);
    const std::size_t sessionHash = std::hash<uint64_t>{}(key.sessionID);
    return sourceHash ^
           (sessionHash + static_cast<std::size_t>(0x9e3779b9U) +
            (sourceHash << 6U) + (sourceHash >> 2U));
}
