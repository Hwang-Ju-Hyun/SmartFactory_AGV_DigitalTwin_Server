#pragma once
#include "MemoryStream.hpp"
#include <vector>
#include <cstdint>

struct RoutePacket
{
    uint32_t agvID;
    std::vector<uint32_t> linkIDs;

    // 쓰는 함수 (Server -> Robot 보낼 때)
    void Serialize(OutputMemoryStream& _stream) const
    {
        _stream.Write(agvID);
        _stream.Write(static_cast<uint16_t>(linkIDs.size()));
        for (uint32_t linkID : linkIDs)
        {
            _stream.Write(linkID);
        }
    }

    // 읽는 함수 (Robot -> Server에서 파싱할 때, 유니티에서 쓸 예정)
    void Deserialize(InputMemoryStream& _stream)
    {
        _stream.Read(agvID);
        
        uint16_t linkCount;
        _stream.Read(linkCount);
        
        for (uint16_t i = 0; i < linkCount; ++i)
        {
            uint32_t linkID;
            _stream.Read(linkID);
            linkIDs.push_back(linkID);
        }
    }
};

// 2. 도착 보고 패킷 (PT_ARRIVED)
struct ArrivedPacket
{
    uint32_t agvID;
    uint32_t currentNodeID;

    void Serialize(OutputMemoryStream& out) const
    {
        out.Write(agvID);
        out.Write(currentNodeID);
    }
    void Deserialize(InputMemoryStream& in)
    {
        in.Read(agvID);
        in.Read(currentNodeID);
    }
};

// 3. 상태 보고 패킷 (PT_STATUS) - 100ms마다 발송
struct StatusPacket
{
    uint32_t agvID;
    uint32_t currentLinkID;
    float progress; // 현재 링크에서의 진행률 (0.0 ~ 1.0)
    float x;
    float z;
    float heading;  // 현재 바라보는 각도 (라디안)
    float velocity; // 현재 주행 속도
    float battery;  // 배터리 잔량 (0.0 ~ 100.0)

    void Serialize(OutputMemoryStream& out) const
    {
        out.Write(agvID);
        out.Write(currentLinkID);
        out.Write(progress);
        out.Write(x);
        out.Write(z);
        out.Write(heading);
        out.Write(velocity);
        out.Write(battery);
    }
    void Deserialize(InputMemoryStream& in)
    {
        in.Read(agvID);
        in.Read(currentLinkID);
        in.Read(progress);
        in.Read(x);
        in.Read(z);
        in.Read(heading);
        in.Read(velocity);
        in.Read(battery);
    }
};

// 4. 경로 취소 패킷 (PT_CANCEL_ROUTE)
struct CancelRoutePacket
{
    uint32_t agvID;

    void Serialize(OutputMemoryStream& out) const { out.Write(agvID); }
    void Deserialize(InputMemoryStream& in) { in.Read(agvID); }
};