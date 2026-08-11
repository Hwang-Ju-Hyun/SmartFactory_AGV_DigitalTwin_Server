#include "FakeRobotApp.hpp"
#include <cerrno>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

namespace
{
    bool ParsePositiveUInt32(const char* text, uint32_t& outValue)
    {
        if (!text || *text == '\0' || *text == '-')
            return false;

        errno = 0;
        char* end = nullptr;
        const unsigned long parsed = std::strtoul(text, &end, 10);
        if (errno == ERANGE || *end != '\0' || parsed == 0 ||
            parsed > std::numeric_limits<uint32_t>::max())
            return false;

        outValue = static_cast<uint32_t>(parsed);
        return true;
    }

    void PrintUsage(const char* executable)
    {
        std::cerr << "Usage: " << executable
                  << " [SERVER_ADDRESS] [AGV_ID]"
                  << " [--expect-trajectory-preview START FINAL --deadline-ms MS]\n";
    }
}

int main(int argc, char** argv)
{
    std::string serverAddress = argc >= 2 ? argv[1] : "127.0.0.1:6666";
    uint32_t agvID = argc >= 3 ? static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 10)) : 1;
    FakeRobotTrajectoryPreviewExpectation expectation;
    bool sawExpectation = false;
    bool sawDeadline = false;

    for (int index = 3; index < argc; ++index)
    {
        const std::string option = argv[index];
        if (option == "--expect-trajectory-preview")
        {
            if (sawExpectation || index + 2 >= argc ||
                !ParsePositiveUInt32(argv[index + 1], expectation.startNodeID) ||
                !ParsePositiveUInt32(argv[index + 2], expectation.finalNodeID))
            {
                PrintUsage(argv[0]);
                return 2;
            }
            sawExpectation = true;
            index += 2;
        }
        else if (option == "--deadline-ms")
        {
            uint32_t deadlineMs = 0;
            if (sawDeadline || index + 1 >= argc ||
                !ParsePositiveUInt32(argv[index + 1], deadlineMs))
            {
                PrintUsage(argv[0]);
                return 2;
            }
            expectation.deadline = std::chrono::milliseconds(deadlineMs);
            sawDeadline = true;
            ++index;
        }
        else
        {
            PrintUsage(argv[0]);
            return 2;
        }
    }

    if (sawExpectation != sawDeadline)
    {
        PrintUsage(argv[0]);
        return 2;
    }
    expectation.enabled = sawExpectation;

    FakeRobotApp app(serverAddress, agvID, expectation);
    return app.Run();
}
