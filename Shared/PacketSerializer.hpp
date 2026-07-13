#pragma once
#include "MemoryStream.hpp"
#include "Protocol.hpp"
#include "SharedPackets.hpp"
#include <vector>

namespace RobotProtocol
{
    struct RouteCommandPayload
    {
        uint32_t routeID = 0;
        std::vector<RouteNodeTime> nodes;
    };

    bool IsKnownPacketID(uint16_t rawPacketID);
    bool PeekPacketID(InputMemoryStream& inStream, PacketID& outPacketID);

    void WritePacketBodyHeader(OutputMemoryStream& outStream, PacketID packetID, uint32_t agvID, uint32_t sequence);
    bool ReadPacketBodyHeader(InputMemoryStream& inStream, PacketBodyHeader& outHeader);

    void WriteHelloPayload(OutputMemoryStream& outStream, const HelloPayload& payload);
    bool ReadHelloPayload(InputMemoryStream& inStream, HelloPayload& outPayload);

    void WriteHelloAckPayload(OutputMemoryStream& outStream, const HelloAckPayload& payload);
    bool ReadHelloAckPayload(InputMemoryStream& inStream, HelloAckPayload& outPayload);

    void WriteStatusPayload(OutputMemoryStream& outStream, const StatusPayload& payload);
    bool ReadStatusPayload(InputMemoryStream& inStream, StatusPayload& outPayload);

    void WriteRouteCommandPayload(OutputMemoryStream& outStream, const RouteCommandPayload& payload);
    bool ReadRouteCommandPayload(InputMemoryStream& inStream, RouteCommandPayload& outPayload);

    void WriteArrivedPayload(OutputMemoryStream& outStream, const ArrivedPayload& payload);
    bool ReadArrivedPayload(InputMemoryStream& inStream, ArrivedPayload& outPayload);

    void WriteErrorPayload(OutputMemoryStream& outStream, const ErrorPayload& payload);
    bool ReadErrorPayload(InputMemoryStream& inStream, ErrorPayload& outPayload);

    void WriteTimePayload(OutputMemoryStream& outStream, const TimePayload& payload);
    bool ReadTimePayload(InputMemoryStream& inStream, TimePayload& outPayload);
}
