#include "MemoryStream.hpp"
#include "PacketSerializer.hpp"
#include "TrajectoryBuilder.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace
{
    struct PreviewOptions
    {
        float millimetersPerMapUnit = std::numeric_limits<float>::quiet_NaN();
        float spacingMm = 20.0f;
        float speedMmPerSecond = 80.0f;
        float startHeadingRad = std::numeric_limits<float>::quiet_NaN();
        float cornerStopThresholdRad = 0.35f;
        float trackWidthMm = std::numeric_limits<float>::quiet_NaN();
        bool requireBezier = false;
        std::vector<uint32_t> nodeIDs;
    };

    void PrintUsage(const char* executable)
    {
        std::cerr
            << "Usage: " << executable
            << " --scale-mm-per-unit VALUE --start-heading-rad VALUE"
               " [--spacing-mm VALUE] [--speed-mm-s VALUE]"
               " [--corner-stop-rad VALUE] [--track-width-mm VALUE]"
               " [--require-bezier] NODE_ID NODE_ID [NODE_ID ...]\n";
    }

    bool ParseFloat(const char* value, float& output)
    {
        if (!value) return false;
        errno = 0;
        char* end = nullptr;
        const float parsed = std::strtof(value, &end);
        if (errno != 0 || end == value || *end != '\0' || !std::isfinite(parsed))
            return false;
        output = parsed;
        return true;
    }

    bool ParseNodeID(const char* value, uint32_t& output)
    {
        if (!value || value[0] == '-') return false;
        errno = 0;
        char* end = nullptr;
        const unsigned long parsed = std::strtoul(value, &end, 10);
        if (errno != 0 || end == value || *end != '\0' || parsed == 0 ||
            parsed > std::numeric_limits<uint32_t>::max())
        {
            return false;
        }
        output = static_cast<uint32_t>(parsed);
        return true;
    }

    bool ParseArguments(int argc, char** argv, PreviewOptions& options)
    {
        for (int i = 1; i < argc; ++i)
        {
            const std::string argument = argv[i];
            auto parseOption = [&](float& destination) -> bool
            {
                if (i + 1 >= argc || !ParseFloat(argv[++i], destination))
                {
                    std::cerr << "[TrajectoryPreview] Invalid value for "
                              << argument << "\n";
                    return false;
                }
                return true;
            };

            if (argument == "--scale-mm-per-unit")
            {
                if (!parseOption(options.millimetersPerMapUnit)) return false;
            }
            else if (argument == "--spacing-mm")
            {
                if (!parseOption(options.spacingMm)) return false;
            }
            else if (argument == "--speed-mm-s")
            {
                if (!parseOption(options.speedMmPerSecond)) return false;
            }
            else if (argument == "--start-heading-rad")
            {
                if (!parseOption(options.startHeadingRad)) return false;
            }
            else if (argument == "--corner-stop-rad")
            {
                if (!parseOption(options.cornerStopThresholdRad)) return false;
            }
            else if (argument == "--track-width-mm")
            {
                if (!parseOption(options.trackWidthMm)) return false;
            }
            else if (argument == "--require-bezier")
            {
                options.requireBezier = true;
            }
            else if (!argument.empty() && argument.front() == '-')
            {
                std::cerr << "[TrajectoryPreview] Unknown option: " << argument << "\n";
                return false;
            }
            else
            {
                uint32_t nodeID = 0;
                if (!ParseNodeID(argv[i], nodeID))
                {
                    std::cerr << "[TrajectoryPreview] Invalid node ID: "
                              << argument << "\n";
                    return false;
                }
                options.nodeIDs.push_back(nodeID);
            }
        }

        if (!std::isfinite(options.millimetersPerMapUnit) ||
            !std::isfinite(options.startHeadingRad) ||
            options.nodeIDs.size() < 2)
        {
            std::cerr << "[TrajectoryPreview] Scale, trusted start heading, and at least"
                         " two node IDs are required.\n";
            return false;
        }
        return true;
    }

    std::string FormatRoute(const std::vector<uint32_t>& nodeIDs)
    {
        std::ostringstream stream;
        for (size_t i = 0; i < nodeIDs.size(); ++i)
        {
            if (i != 0) stream << " -> ";
            stream << nodeIDs[i];
        }
        return stream.str();
    }

    std::string FormatFlags(uint8_t flags)
    {
        if (flags == RobotProtocol::TRAJECTORY_FLAG_NONE) return "NONE";

        std::ostringstream stream;
        auto append = [&](const char* name)
        {
            if (stream.tellp() > 0) stream << '|';
            stream << name;
        };
        if ((flags & RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY) != 0)
            append("NODE");
        if ((flags & RobotProtocol::TRAJECTORY_FLAG_STOP) != 0)
            append("STOP");
        if ((flags & RobotProtocol::TRAJECTORY_FLAG_ROTATE_IN_PLACE) != 0)
            append("ROTATE");
        if ((flags & RobotProtocol::TRAJECTORY_FLAG_FINAL) != 0)
            append("FINAL");
        return stream.str();
    }

    float NormalizeAngle(float angle)
    {
        constexpr float pi = 3.14159265358979323846f;
        while (angle > pi) angle -= 2.0f * pi;
        while (angle < -pi) angle += 2.0f * pi;
        return angle;
    }
}

int main(int argc, char** argv)
{
    PreviewOptions preview;
    if (!ParseArguments(argc, argv, preview))
    {
        PrintUsage(argv[0]);
        return 2;
    }

    auto& map = MapManager::GetInstance();
    size_t lineCount = 0;
    size_t bezierCount = 0;
    for (size_t i = 0; i + 1 < preview.nodeIDs.size(); ++i)
    {
        const MapLink& link = map.FindLink(preview.nodeIDs[i], preview.nodeIDs[i + 1]);
        if (link.m_Type == 0) ++lineCount;
        else if (link.m_Type == 1) ++bezierCount;
        else
        {
            std::cerr << "[TrajectoryPreview] Missing or unsupported directed link "
                      << preview.nodeIDs[i] << " -> " << preview.nodeIDs[i + 1]
                      << "\n";
            return 1;
        }
    }
    if (preview.requireBezier && bezierCount == 0)
    {
        std::cerr << "[TrajectoryPreview] Expected at least one Bezier link\n";
        return 1;
    }

    TrajectoryBuildOptions buildOptions;
    buildOptions.hasTrustedStartHeading = true;
    buildOptions.startHeadingRad = preview.startHeadingRad;
    buildOptions.millimetersPerMapUnit = preview.millimetersPerMapUnit;
    buildOptions.spacingMm = preview.spacingMm;
    buildOptions.cruiseSpeedMmPerSecond = preview.speedMmPerSecond;
    buildOptions.cornerStopThresholdRad = preview.cornerStopThresholdRad;

    RobotProtocol::TrajectoryCommandPayload trajectory;
    std::string error;
    if (!TrajectoryBuilder::Build(preview.nodeIDs, 1, buildOptions,
                                  trajectory, error))
    {
        std::cerr << "[TrajectoryPreview] Build failed: " << error << "\n";
        return 1;
    }

    OutputMemoryStream encoded;
    if (!RobotProtocol::WriteTrajectoryCommandPayload(encoded, trajectory))
    {
        std::cerr << "[TrajectoryPreview] Serialization failed\n";
        return 1;
    }

    float sampledLengthMm = 0.0f;
    float minimumSampledTurnRadiusMm = std::numeric_limits<float>::infinity();
    for (size_t i = 1; i < trajectory.waypoints.size(); ++i)
    {
        const auto& previous = trajectory.waypoints[i - 1];
        const auto& current = trajectory.waypoints[i];
        const float segmentLength = std::hypot(
            current.forwardMm - previous.forwardMm,
            current.leftMm - previous.leftMm);
        sampledLengthMm += segmentLength;

        const float headingChange = std::abs(NormalizeAngle(
            current.headingRad - previous.headingRad));
        if (segmentLength > 0.001f && headingChange > 0.001f)
        {
            minimumSampledTurnRadiusMm = std::min(
                minimumSampledTurnRadiusMm, segmentLength / headingChange);
        }
    }

    std::cout << "\n[TrajectoryPreview] HARDWARE-FREE PREVIEW: no TCP, no motor output\n"
              << "[TrajectoryPreview] route=" << FormatRoute(preview.nodeIDs) << "\n"
              << "[TrajectoryPreview] links LINE=" << lineCount
              << " BEZIER=" << bezierCount << "\n"
              << "[TrajectoryPreview] scale=" << preview.millimetersPerMapUnit
              << " mm/map-unit (temporary until physical calibration)\n"
              << "[TrajectoryPreview] assumedStartHeading=" << preview.startHeadingRad
              << " rad (geometry-only; not a physical calibration)\n"
              << "[TrajectoryPreview] spacing=" << preview.spacingMm
              << " mm speed=" << preview.speedMmPerSecond << " mm/s\n"
              << "[TrajectoryPreview] waypoints=" << trajectory.waypoints.size()
              << " sampledLength=" << sampledLengthMm
              << " mm payloadBytes=" << encoded.GetLength() << "\n\n";

    if (std::isfinite(minimumSampledTurnRadiusMm))
    {
        std::cout << "[TrajectoryPreview] minimumSampledTurnRadius="
                  << minimumSampledTurnRadiusMm << " mm\n";
        if (std::isfinite(preview.trackWidthMm) && preview.trackWidthMm > 0.0f &&
            minimumSampledTurnRadiusMm < preview.trackWidthMm * 0.5f)
        {
            std::cout
                << "[TrajectoryPreview] WARNING: radius is smaller than half the"
                   " track width; differential drive requires inner-wheel reversal"
                   " or a different physical scale/path.\n";
        }
        std::cout << "\n";
    }

    std::cout << std::fixed << std::setprecision(3)
              << " idx  forward(mm)   left(mm)  heading(rad)  speed(mm/s)  node  flags\n";
    for (size_t i = 0; i < trajectory.waypoints.size(); ++i)
    {
        const auto& waypoint = trajectory.waypoints[i];
        std::cout << std::setw(4) << i
                  << std::setw(13) << waypoint.forwardMm
                  << std::setw(11) << waypoint.leftMm
                  << std::setw(14) << waypoint.headingRad
                  << std::setw(13) << waypoint.targetSpeedMmPerSecond
                  << std::setw(6) << waypoint.nodeID << "  "
                  << FormatFlags(waypoint.flags) << "\n";
    }

    return 0;
}
