#pragma once

#include "Protocol.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>

// Coordinates on the Vision wire are local millimetres.  This contract keeps
// the conversion back to the active Server map explicit, so a packet is never
// checked against hard-coded pixel or Unity coordinates.
struct VisionMapCoordinateContract
{
    double millimetersPerServerUnit = 50.0;
    double localOriginServerX = 50.0;
    double localOriginServerZ = -36.0;
    double minimumServerX = std::numeric_limits<double>::quiet_NaN();
    double maximumServerX = std::numeric_limits<double>::quiet_NaN();
    double minimumServerZ = std::numeric_limits<double>::quiet_NaN();
    double maximumServerZ = std::numeric_limits<double>::quiet_NaN();
    double allowedMarginMillimeters = 100.0;

    // Reads the node-centre bounds from the active Shared/MapData.json through
    // MapManager.  Tests can instead inject explicit bounds in this struct.
    static VisionMapCoordinateContract FromCanonicalMap(
        double millimetersPerServerUnit = 50.0,
        double localOriginServerX = 50.0,
        double localOriginServerZ = -36.0,
        double allowedMarginMillimeters = 100.0);

    bool IsValid() const;
    bool ContainsLocalMillimeters(double xMillimeters,
                                  double zMillimeters) const;
};

struct VisionMetricPose
{
    float xMillimeters = 0.0f;
    float zMillimeters = 0.0f;
    float headingDegrees = 0.0f;
};

// A transport-neutral observation.  Network code maps the deserialized
// RobotProtocol payload into this value and supplies the Server receive time.
// sourceTimestampMicroseconds is sender-local diagnostic data only; this store
// deliberately never compares it with the Server clock.
struct VisionObservationInput
{
    uint32_t agvID = 0;
    uint32_t sourceID = 0;
    uint64_t sessionID = 0;
    uint32_t sequence = 0;
    uint64_t sourceTimestampMicroseconds = 0;
    uint32_t reportedAgeMilliseconds = 0;
    RobotProtocol::VisionTrackingState state =
        RobotProtocol::VisionTrackingState::LOST;
    std::optional<VisionMetricPose> pose;
    std::string calibrationID;
    std::string mapContractID;
    std::string poseContractID;
    RobotProtocol::VisionVerificationState verificationState =
        RobotProtocol::VisionVerificationState::UNKNOWN;
    RobotProtocol::VisionQualityMetadata quality;
};

struct StoredVisionObservation
{
    VisionObservationInput observation;
    uint64_t receivedAtServerMilliseconds = 0;
};

struct VisionObservationStoreConfig
{
    VisionMapCoordinateContract map;
    std::string expectedCalibrationID;
    std::string expectedMapContractID;
    std::string expectedPoseContractID;
    std::optional<uint32_t> expectedSourceID;

    // These mirror the initial VisionTracker MEASURED/HELD policy.  They are
    // injected configuration, not accuracy claims.
    uint32_t maximumMeasuredReportedAgeMilliseconds = 100;
    uint32_t maximumHeldReportedAgeMilliseconds = 200;
    uint64_t maximumReceiveAgeMilliseconds = 500;
};

enum class VisionObservationStoreResult
{
    ACCEPTED,
    AGV_REGISTRATION_CHECK_UNAVAILABLE,
    UNKNOWN_AGV,
    INVALID_SOURCE_ID,
    SOURCE_ID_MISMATCH,
    INVALID_SESSION_ID,
    INVALID_SEQUENCE,
    INVALID_TRACKING_STATE,
    INVALID_VERIFICATION_STATE,
    MEASURED_NOT_VERIFIED,
    INVALID_CALIBRATION_ID,
    CALIBRATION_ID_MISMATCH,
    INVALID_MAP_CONTRACT_ID,
    MAP_CONTRACT_ID_MISMATCH,
    INVALID_POSE_CONTRACT_ID,
    POSE_CONTRACT_ID_MISMATCH,
    INVALID_STATE_POSE_COMBINATION,
    NON_FINITE_POSE,
    HEADING_OUT_OF_RANGE,
    INVALID_QUALITY_METADATA,
    OUT_OF_MAP,
    STALE_REPORTED_AGE,
    RECEIVE_TIME_IN_FUTURE,
    STALE_RECEIVE_TIME,
    DUPLICATE_OR_OUT_OF_ORDER_SEQUENCE
};

// Observation-only storage.  This class has no reference to AGV world state,
// RoutePlanner, reservations, TaskManager, or robot controllers.
class VisionObservationStore
{
public:
    using IsRegisteredAgvCallback = std::function<bool(uint32_t)>;

    explicit VisionObservationStore(VisionObservationStoreConfig config);

    // receivedAtServerMilliseconds and nowServerMilliseconds must be from the
    // same Server-owned monotonic clock.  A new authenticated session may reset
    // sequence numbering; Network code remains responsible for authenticating
    // which source/session is currently connected.
    VisionObservationStoreResult TryStore(
        const VisionObservationInput& observation,
        uint64_t receivedAtServerMilliseconds,
        uint64_t nowServerMilliseconds,
        const IsRegisteredAgvCallback& isRegisteredAgv);

    std::optional<StoredVisionObservation> GetLatest(uint32_t agvID) const;

    // This reports packet receive recency only.  It does not turn HELD into a
    // fresh measurement and it does not make LOST contain a pose.
    bool IsLatestWithinReceiveTimeout(uint32_t agvID,
                                      uint64_t nowServerMilliseconds) const;

    std::size_t Size() const { return m_LatestByAgv.size(); }
    const VisionObservationStoreConfig& GetConfig() const { return m_Config; }

private:
    struct StreamKey
    {
        uint32_t sourceID = 0;
        uint64_t sessionID = 0;

        bool operator==(const StreamKey&) const = default;
    };

    struct StreamKeyHash
    {
        std::size_t operator()(const StreamKey& key) const noexcept;
    };

    VisionObservationStoreConfig m_Config;
    std::unordered_map<uint32_t, StoredVisionObservation> m_LatestByAgv;
    std::unordered_map<StreamKey, uint32_t, StreamKeyHash>
        m_LastSequenceByStream;
};
