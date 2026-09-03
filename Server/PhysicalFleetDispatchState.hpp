#pragma once

#include <cstdint>

struct PhysicalFleetDepartureRequest
{
    uint32_t startNodeID = 0;
    uint32_t targetNodeID = 0;
};

class PhysicalFleetDispatchState
{
public:
    bool EnsureConfirmedNode(uint32_t nodeID)
    {
        if (nodeID == 0)
            return false;
        if (m_ConfirmedNodeID == 0)
            m_ConfirmedNodeID = nodeID;
        return m_ConfirmedNodeID == nodeID;
    }

    bool CommitArrival(uint32_t nodeID)
    {
        if (nodeID == 0)
            return false;
        m_ConfirmedNodeID = nodeID;
        m_PhysicalArrivalCommitted = true;
        return true;
    }

    bool HoldDeparture(uint32_t startNodeID, uint32_t targetNodeID)
    {
        if (!EnsureConfirmedNode(startNodeID) || targetNodeID == 0 ||
            startNodeID == targetNodeID || m_DepartureHold)
        {
            return false;
        }
        m_DepartureHold = true;
        m_Request = { startNodeID, targetNodeID };
        return true;
    }

    bool GrantDeparture(uint32_t startNodeID, uint32_t targetNodeID)
    {
        if (!MatchesHeldDeparture(startNodeID, targetNodeID))
            return false;
        m_DepartureHold = false;
        return true;
    }

    bool MatchesHeldDeparture(
        uint32_t startNodeID,
        uint32_t targetNodeID) const
    {
        return m_DepartureHold &&
            m_Request.startNodeID == startNodeID &&
            m_Request.targetNodeID == targetNodeID &&
            m_ConfirmedNodeID == startNodeID;
    }

    void CancelDeparture()
    {
        m_DepartureHold = false;
        m_Request = {};
    }

    uint32_t GetConfirmedNodeID() const { return m_ConfirmedNodeID; }
    bool IsPhysicalArrivalCommitted() const
    {
        return m_PhysicalArrivalCommitted;
    }
    bool IsDepartureHeld() const { return m_DepartureHold; }
    PhysicalFleetDepartureRequest GetDepartureRequest() const
    {
        return m_Request;
    }

private:
    uint32_t m_ConfirmedNodeID = 0;
    bool m_PhysicalArrivalCommitted = false;
    bool m_DepartureHold = false;
    PhysicalFleetDepartureRequest m_Request;
};
