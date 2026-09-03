#include "PhysicalFleetDispatchState.hpp"

#include <cstdlib>
#include <iostream>

namespace
{
    [[noreturn]] void Fail(const char* expression, int line)
    {
        std::cerr << "PhysicalFleetDispatchStateTest failed at line "
                  << line << ": " << expression << "\n";
        std::exit(1);
    }

#define REQUIRE(expression) \
    do { if (!(expression)) Fail(#expression, __LINE__); } while (false)

    void TestEveryRouteEdgeStartsHeld()
    {
        PhysicalFleetDispatchState state;

        REQUIRE(state.HoldDeparture(1, 6));
        REQUIRE(state.GetConfirmedNodeID() == 1);
        REQUIRE(!state.IsPhysicalArrivalCommitted());
        REQUIRE(state.MatchesHeldDeparture(1, 6));
        REQUIRE(!state.HoldDeparture(1, 6));
        REQUIRE(!state.GrantDeparture(1, 7));
        REQUIRE(state.GrantDeparture(1, 6));
        REQUIRE(!state.IsDepartureHeld());

        REQUIRE(state.CommitArrival(6));
        REQUIRE(state.HoldDeparture(6, 11));
        REQUIRE(state.MatchesHeldDeparture(6, 11));
        REQUIRE(state.GrantDeparture(6, 11));

        REQUIRE(state.CommitArrival(12));
        REQUIRE(state.HoldDeparture(12, 7));
        REQUIRE(state.MatchesHeldDeparture(12, 7));
    }

    void TestFailureKeepsCommittedArrivalAndDepartureHeld()
    {
        PhysicalFleetDispatchState state;
        REQUIRE(state.EnsureConfirmedNode(12));
        REQUIRE(state.CommitArrival(7));
        REQUIRE(state.HoldDeparture(7, 6));

        REQUIRE(state.GetConfirmedNodeID() == 7);
        REQUIRE(state.IsPhysicalArrivalCommitted());
        REQUIRE(state.IsDepartureHeld());
        REQUIRE(state.MatchesHeldDeparture(7, 6));

        state.CancelDeparture();
        REQUIRE(state.GetConfirmedNodeID() == 7);
        REQUIRE(!state.IsDepartureHeld());
    }
}

int main()
{
    TestEveryRouteEdgeStartsHeld();
    TestFailureKeepsCommittedArrivalAndDepartureHeld();
    std::cout << "Physical fleet dispatch state tests passed\n";
    return 0;
}
