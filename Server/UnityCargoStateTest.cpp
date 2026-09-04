#include "UnityCargoState.hpp"

#include "MemoryStream.hpp"
#include "header.hpp"
#include <cstdlib>
#include <iostream>

namespace
{
    void Require(bool condition, const char* message)
    {
        if (!condition)
        {
            std::cerr << "FAILED: " << message << "\n";
            std::exit(1);
        }
    }
}

int main()
{
    UnityCargoStateStore store;

    const auto loaded = store.MarkLoaded(3, 120);
    Require(loaded.has_value(), "first pickup must create cargo state");
    Require(loaded->state == UnityCargoState::LOADED,
            "pickup state must be LOADED");
    Require(loaded->taskID != 0 && loaded->cargoID != 0,
            "pickup must allocate stable identifiers");
    Require(!store.MarkLoaded(3, 120).has_value(),
            "duplicate pickup must be idempotent");

    OutputMemoryStream stream;
    Require(WriteUnityCargoStatePacket(stream, *loaded),
            "valid cargo state must serialize");
    Require(stream.GetLength() == 22,
            "cargo packet body must remain exactly 22 bytes");
    InputMemoryStream input(
        const_cast<char*>(stream.GetBuffer()), stream.GetLength());
    uint8_t packetType = 0;
    uint32_t sequence = 0;
    uint32_t agvID = 0;
    uint32_t taskID = 0;
    uint32_t cargoID = 0;
    uint32_t nodeID = 0;
    uint8_t state = 0;
    input.Read(packetType);
    input.Read(sequence);
    input.Read(agvID);
    input.Read(taskID);
    input.Read(cargoID);
    input.Read(nodeID);
    input.Read(state);
    Require(packetType == static_cast<uint8_t>(UPT_CARGO_STATE),
            "packet discriminator must be UPT_CARGO_STATE");
    Require(sequence == loaded->sequence && agvID == 3 &&
                taskID == loaded->taskID && cargoID == loaded->cargoID &&
                nodeID == 120 &&
                state == static_cast<uint8_t>(UnityCargoState::LOADED),
            "serialized cargo fields must preserve their values");
    Require(input.GetRemainDataSize() == 0,
            "cargo packet must not contain trailing fields");

    const auto unloaded = store.MarkUnloaded(3, 4);
    Require(unloaded.has_value(), "drop must create unloaded state");
    Require(unloaded->sequence > loaded->sequence,
            "drop must advance state sequence");
    Require(unloaded->taskID == loaded->taskID &&
                unloaded->cargoID == loaded->cargoID,
            "drop must identify the same task and cargo");
    Require(unloaded->state == UnityCargoState::UNLOADED,
            "drop state must be UNLOADED");
    Require(!store.MarkUnloaded(3, 4).has_value(),
            "duplicate drop must be idempotent");

    const auto nextLoaded = store.MarkLoaded(3, 200);
    Require(nextLoaded.has_value(), "next pickup must create a new task");
    Require(nextLoaded->taskID != loaded->taskID &&
                nextLoaded->cargoID != loaded->cargoID,
            "next pickup must allocate new identifiers");

    const auto otherLoaded = store.MarkLoaded(1, 201);
    Require(otherLoaded.has_value(), "another AGV must track independently");
    const auto snapshot = store.GetSnapshot();
    Require(snapshot.size() == 2 && snapshot[0].agvID == 1 &&
                snapshot[1].agvID == 3,
            "reconnect snapshot must be deterministic by AGV ID");

    UnityCargoStatePayload invalid;
    OutputMemoryStream invalidStream;
    Require(!WriteUnityCargoStatePacket(invalidStream, invalid),
            "invalid zero identifiers must not serialize");

    std::cout << "Unity cargo state tests passed\n";
    return 0;
}
