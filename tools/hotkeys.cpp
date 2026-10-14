// hotkeys <mapA> <mapB> - exercises what the repurposed hotkeys do on a map:
// Launch Ball / Take Possession = previous / next checkpoint (wrapping),
// Reset Shot = the last checkpoint reached, and all of it still working
// across a mid-session map swap and after unload.
//
// Build (from repo root):
//   c++ -std=c++20 -O2 -I. -Isrc tools/hotkeys.cpp src/maploader/*.cpp \
//       src/sim/*.cpp build/RocketSim/libRocketSim.a -o hotkeys
// Run (from the repo root, so assets/collision_meshes resolves):
//   ./hotkeys downloaded_maps/r3.udk downloaded_maps/LethIceRings.udk
//   (exit 0 = all hotkey checks passed)
#include <cmath>
#include <cstdio>
#include <string>

#include "maploader/map_loader.h"
#include "sim/sim.h"

using RocketSim::CarState;

static int failures = 0;
#define CHECK(cond, ...)                                    \
    do {                                                    \
        if (!(cond)) {                                      \
            failures++;                                     \
            std::printf("  FAIL: " __VA_ARGS__);            \
            std::printf("\n");                              \
        }                                                   \
    } while (0)

static void settle(Sim& sim, float seconds) {
    for (int i = 0; i < (int)(seconds * 120.0f + 0.5f); i++) sim.advance(1.0 / 120.0);
}

static bool near(const V3& a, const V3& b, float tol = 5.f) {
    const V3 d = a - b;
    return d.dot(d) <= tol * tol;
}

static bool loadInto(const char* path, maploader::MapData* out, std::string* err) {
    return maploader::loadMap(path, out, err);
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s <mapA> <mapB>\n", argv[0]);
        return 2;
    }
    std::string err;
    Sim sim;
    if (!sim.init("assets/collision_meshes")) return 2;

    maploader::MapData a;
    if (!loadInto(argv[1], &a, &err)) {
        std::printf("load A failed: %s\n", err.c_str());
        return 2;
    }
    const size_t nA = a.sessions.size();
    std::vector<maploader::MapSession> sa = a.sessions;
    if (!sim.applyMap(std::move(a), &err)) {
        std::printf("apply A failed: %s\n", err.c_str());
        return 2;
    }
    std::printf("== %s (%zu sessions)\n", argv[1], nA);

    // --- Launch Ball / Take Possession step along the route, wrapping ------
    CHECK(sim.sessionIndex() == 0, "starts at checkpoint 0 (got %d)", sim.sessionIndex());
    sim.cycleSession(1);
    CHECK(sim.sessionIndex() == 1, "next -> 1 (got %d)", sim.sessionIndex());
    CHECK(near(V3(sim.car()->GetState().pos.x, sim.car()->GetState().pos.y,
                  sim.car()->GetState().pos.z),
               sa[1].carPos),
          "car teleported to checkpoint 1's spawn");
    sim.cycleSession(-1);
    CHECK(sim.sessionIndex() == 0, "prev -> 0 (got %d)", sim.sessionIndex());
    if (nA > 1) {
        sim.cycleSession(-1);
        CHECK(sim.sessionIndex() == (int)nA - 1, "prev wraps to the last (got %d)",
              sim.sessionIndex());
        sim.cycleSession(1);
        CHECK(sim.sessionIndex() == 0, "next wraps back to 0 (got %d)", sim.sessionIndex());
    }

    // --- reached by driving there, then Reset Shot returns to it ----------
    int reached = -1;
    sim.onCheckpointReached = [&](int idx, int) { reached = idx; };
    const int target = nA > 4 ? 4 : (int)nA - 1;
    const maploader::MapSession& ts = sa[(size_t)target];
    sim.car()->SetState([&] {
        CarState s = sim.car()->GetState();
        s.pos = RocketSim::Vec(ts.carPos.x, ts.carPos.y, ts.carPos.z);
        s.vel = RocketSim::Vec(0, 0, 0);
        s.angVel = RocketSim::Vec(0, 0, 0);
        return s;
    }());
    settle(sim, 0.5f);
    sim.onCheckpointReached = nullptr;
    CHECK(reached == target, "driving onto checkpoint %d reaches it (got %d)", target, reached);
    CHECK(sim.sessionIndex() == target, "session is now %d (got %d)", target,
          sim.sessionIndex());

    // Move the car off the checkpoint *without* cycling (lift it - it stays
    // closest to the same session, so nothing else steals it). Reset Shot
    // must bring it back to the checkpoint last reached, preset ignored.
    {
        CarState s = sim.car()->GetState();
        s.pos.z += 400.f;
        s.vel = RocketSim::Vec(0, 0, 0);
        sim.car()->SetState(s);
        settle(sim, 0.3f);
    }
    CHECK(sim.sessionIndex() == target, "moving off it does not change the session (%d, got %d)",
          target, sim.sessionIndex());
    sim.resetShot(0);
    CHECK(sim.sessionIndex() == target, "Reset Shot returns to the last reached (%d, got %d)",
          target, sim.sessionIndex());
    CHECK(near(V3(sim.car()->GetState().pos.x, sim.car()->GetState().pos.y,
                  sim.car()->GetState().pos.z),
               ts.carPos),
          "Reset Shot parks the car back on that checkpoint");

    // Cycling *selects* a checkpoint (and teleports the car there), so that
    // is what Reset Shot uses afterwards - the cycled-to point counts as
    // reached, since the car was put down on it.
    if (nA > 2) {
        sim.cycleSession(1);
        const int cycled = sim.sessionIndex();
        CarState s = sim.car()->GetState();
        s.pos.z += 400.f;
        s.vel = RocketSim::Vec(0, 0, 0);
        sim.car()->SetState(s);
        settle(sim, 0.3f);
        sim.resetShot(0);
        CHECK(sim.sessionIndex() == cycled, "Reset Shot goes to the cycled checkpoint (%d, got %d)",
              cycled, sim.sessionIndex());
        CHECK(near(V3(sim.car()->GetState().pos.x, sim.car()->GetState().pos.y,
                      sim.car()->GetState().pos.z),
                   sa[(size_t)cycled].carPos),
              "car is back on the cycled checkpoint");
    }

    // --- same behaviour after a mid-session swap --------------------------
    maploader::MapData b;
    if (!loadInto(argv[2], &b, &err)) {
        std::printf("load B failed: %s\n", err.c_str());
        return 2;
    }
    const size_t nB = b.sessions.size();
    std::vector<maploader::MapSession> sb = b.sessions;
    if (!sim.applyMap(std::move(b), &err)) {
        std::printf("apply B failed: %s\n", err.c_str());
        return 2;
    }
    std::printf("== swapped to %s (%zu sessions)\n", argv[2], nB);
    CHECK(sim.sessionCount() == (int)nB, "swap replaces the session list (%d != %zu)",
          sim.sessionCount(), nB);
    CHECK(sim.sessionIndex() >= 0 && sim.sessionIndex() < (int)nB,
          "session index is inside the new list (%d of %zu)", sim.sessionIndex(), nB);
    sim.cycleSession(1);
    CHECK(sim.sessionIndex() >= 0 && sim.sessionIndex() < (int)nB, "cycling works on B");
    reached = -1;
    sim.onCheckpointReached = [&](int idx, int) { reached = idx; };
    const int tB = (int)nB - 1;
    const maploader::MapSession& tsB = sb[(size_t)tB];
    sim.car()->SetState([&] {
        CarState s = sim.car()->GetState();
        s.pos = RocketSim::Vec(tsB.carPos.x, tsB.carPos.y, tsB.carPos.z);
        s.vel = RocketSim::Vec(0, 0, 0);
        s.angVel = RocketSim::Vec(0, 0, 0);
        return s;
    }());
    settle(sim, 0.5f);
    sim.onCheckpointReached = nullptr;
    CHECK(reached == tB, "proximity still fires on the new map (got %d, want %d)", reached, tB);

    // --- unload leaves a working default arena ----------------------------
    sim.unloadMap();
    CHECK(sim.sessionCount() == 0, "unload clears sessions (%d)", sim.sessionCount());
    sim.cycleSession(1);   // must be a no-op, not a crash
    sim.resetShot(0);
    settle(sim, 0.5f);
    CHECK(sim.sessionIndex() == 0, "index is 0 after unload (%d)", sim.sessionIndex());

    sim.shutdown();
    std::printf("%s\n", failures ? "FAILURES" : "all hotkey checks passed");
    return failures ? 1 : 0;
}
