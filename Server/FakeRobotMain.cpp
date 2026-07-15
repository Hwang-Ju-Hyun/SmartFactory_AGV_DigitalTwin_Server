#include "FakeRobotApp.hpp"
#include <cstdlib>
#include <string>

int main(int argc, char** argv)
{
    std::string serverAddress = argc >= 2 ? argv[1] : "127.0.0.1:6666";
    uint32_t agvID = argc >= 3 ? static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 10)) : 1;

    FakeRobotApp app(serverAddress, agvID);
    return app.Run();
}
