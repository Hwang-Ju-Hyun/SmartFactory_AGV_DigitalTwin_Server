#include "UnityCargoState.hpp"

#include "MemoryStream.hpp"
#include "header.hpp"
#include <algorithm>

std::optional<UnityCargoStatePayload> UnityCargoStateStore::MarkLoaded(
    uint32_t _agvID, uint32_t _nodeID)
{
    if (_agvID == 0 || _nodeID == 0)
        return std::nullopt;

    const auto existing = m_StateByAgv.find(_agvID);
    if (existing != m_StateByAgv.end() &&
        existing->second.state == UnityCargoState::LOADED)
    {
        return std::nullopt;
    }

    UnityCargoStatePayload payload;
    payload.sequence = m_NextSequence++;
    payload.agvID = _agvID;
    payload.taskID = m_NextTaskID++;
    payload.cargoID = m_NextCargoID++;
    payload.nodeID = _nodeID;
    payload.state = UnityCargoState::LOADED;
    m_StateByAgv[_agvID] = payload;
    return payload;
}

std::optional<UnityCargoStatePayload> UnityCargoStateStore::MarkUnloaded(
    uint32_t _agvID, uint32_t _nodeID)
{
    if (_agvID == 0 || _nodeID == 0)
        return std::nullopt;

    const auto existing = m_StateByAgv.find(_agvID);
    if (existing == m_StateByAgv.end() ||
        existing->second.state != UnityCargoState::LOADED)
    {
        return std::nullopt;
    }

    UnityCargoStatePayload payload = existing->second;
    payload.sequence = m_NextSequence++;
    payload.nodeID = _nodeID;
    payload.state = UnityCargoState::UNLOADED;
    existing->second = payload;
    return payload;
}

std::vector<UnityCargoStatePayload> UnityCargoStateStore::GetSnapshot() const
{
    std::vector<UnityCargoStatePayload> snapshot;
    snapshot.reserve(m_StateByAgv.size());
    for (const auto& [agvID, payload] : m_StateByAgv)
    {
        (void)agvID;
        snapshot.push_back(payload);
    }
    std::sort(snapshot.begin(), snapshot.end(),
              [](const auto& lhs, const auto& rhs)
              {
                  return lhs.agvID < rhs.agvID;
              });
    return snapshot;
}

bool WriteUnityCargoStatePacket(
    OutputMemoryStream& _stream,
    const UnityCargoStatePayload& _payload)
{
    if (_payload.sequence == 0 || _payload.agvID == 0 ||
        _payload.taskID == 0 || _payload.cargoID == 0 ||
        _payload.nodeID == 0 ||
        (_payload.state != UnityCargoState::UNLOADED &&
         _payload.state != UnityCargoState::LOADED))
    {
        return false;
    }

    _stream.Write(static_cast<uint8_t>(UPT_CARGO_STATE));
    _stream.Write(_payload.sequence);
    _stream.Write(_payload.agvID);
    _stream.Write(_payload.taskID);
    _stream.Write(_payload.cargoID);
    _stream.Write(_payload.nodeID);
    _stream.Write(static_cast<uint8_t>(_payload.state));
    return true;
}
