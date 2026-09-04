#pragma once

#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

class OutputMemoryStream;

enum class UnityCargoState : uint8_t
{
    UNLOADED = 0,
    LOADED = 1
};

struct UnityCargoStatePayload
{
    uint32_t sequence = 0;
    uint32_t agvID = 0;
    uint32_t taskID = 0;
    uint32_t cargoID = 0;
    uint32_t nodeID = 0;
    UnityCargoState state = UnityCargoState::UNLOADED;
};

class UnityCargoStateStore
{
public:
    std::optional<UnityCargoStatePayload> MarkLoaded(
        uint32_t _agvID, uint32_t _nodeID);
    std::optional<UnityCargoStatePayload> MarkUnloaded(
        uint32_t _agvID, uint32_t _nodeID);
    std::vector<UnityCargoStatePayload> GetSnapshot() const;

private:
    uint32_t m_NextSequence = 1;
    uint32_t m_NextTaskID = 1;
    uint32_t m_NextCargoID = 1;
    std::unordered_map<uint32_t, UnityCargoStatePayload> m_StateByAgv;
};

bool WriteUnityCargoStatePacket(
    OutputMemoryStream& _stream,
    const UnityCargoStatePayload& _payload);
