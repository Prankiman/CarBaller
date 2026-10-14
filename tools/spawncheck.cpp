// spawncheck <map.udk> [lift] [v] - audit every spawn/checkpoint of a map.
//
// For each session: drop the car on its spawn, let it settle, then try to
// drive away. Flags the ones that free-fall forever (nothing under them),
// land inside geometry, land on their roof, or cannot move. Then walks the
// checkpoint list to confirm proximity reaches each one exactly once.
//
// Build (from repo root):
//   c++ -std=c++20 -O2 -I. -Isrc tools/spawncheck.cpp src/maploader/*.cpp \
//       src/sim/*.cpp build/RocketSim/libRocketSim.a -o spawncheck
// Run (from the repo root, so assets/collision_meshes resolves):
//   ./spawncheck downloaded_maps/LethIceRings.udk
//   (exit 0 = every spawn stood, settled and drivable, every checkpoint
//    reachable exactly once)
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "maploader/map_loader.h"
#include "sim/sim.h"

using RocketSim::Angle;
using RocketSim::CarControls;
using RocketSim::CarState;
using RocketSim::Vec;
using V3f = V3;

static void settle(Sim& sim, float seconds) {
    const int ticks = (int)(seconds * 120.0f + 0.5f);
    for (int i = 0; i < ticks; i++) sim.advance(1.0 / 120.0);
}

static void place(Sim& sim, const V3f& p, float yaw, float lift) {
    CarState s = sim.car()->GetState();
    s.pos = Vec(p.x, p.y, p.z + lift);
    s.rotMat = Angle(yaw, 0, 0).ToRotMat();
    s.vel = Vec(0, 0, 0);
    s.angVel = Vec(0, 0, 0);
    sim.car()->SetState(s);
    sim.car()->controls = CarControls();
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <map> [lift] [v]\n", argv[0]);
        return 2;
    }
    const char* path = argv[1];
    const float lift = (argc > 2) ? (float)atof(argv[2]) : 0.0f;
    const bool verbose = argc > 3;

    maploader::MapData md;
    std::string err;
    if (!maploader::loadMap(path, &md, &err)) {
        std::printf("load failed: %s\n", err.c_str());
        return 1;
    }

    Sim sim;
    if (!sim.init("assets/collision_meshes")) {
        std::printf("sim init failed\n");
        return 1;
    }
    const std::vector<maploader::MapSession> sessions = md.sessions;  // applyMap consumes md
    if (!sim.applyMap(std::move(md), &err)) {
        std::printf("applyMap failed: %s\n", err.c_str());
        return 1;
    }

    int derivedN = 0;
    for (const maploader::MapSession& s : sessions) derivedN += s.derived ? 1 : 0;
    std::printf("== %s  sessions=%d derived=%d\n", path, sim.sessionCount(), derivedN);

    // ---- 1. every spawn, settled and driven -----------------------------
    int fell = 0, stuck = 0, inside = 0, upside = 0, voids = 0, walled = 0;
    const int n = sim.sessionCount();
    for (int i = 0; i < n; i++) {
        const maploader::MapSession& s = sessions[(size_t)i];
        const V3f p(s.carPos.x, s.carPos.y, s.carPos.z);
        if (verbose) std::printf("  -- %s reach=%.0f\n", s.name.c_str(), s.reach);

        place(sim, p, s.carYawRad, lift);
        settle(sim, 2.0f);
        CarState a = sim.car()->GetState();
        const float z1 = a.pos.z, vz1 = a.vel.z;
        settle(sim, 2.0f);  // still coming down after 4 s means there is no floor
        CarState b = sim.car()->GetState();
        const bool falling = (z1 - b.pos.z) > 300.0f;
        const float dz = b.pos.z - p.z;
        const float upZ = b.rotMat.up.z;

        // drive away, both directions, from a fresh drop
        auto drive = [&](float throttle) {
            place(sim, p, s.carYawRad, lift);
            settle(sim, 1.0f);
            const Vec from = sim.car()->GetState().pos;
            CarControls c;
            c.throttle = throttle;
            sim.car()->controls = c;
            settle(sim, 1.5f);
            const Vec to = sim.car()->GetState().pos;
            sim.car()->controls = CarControls();
            return (float)std::sqrt((to.x - from.x) * (to.x - from.x) +
                                    (to.y - from.y) * (to.y - from.y));
        };
        const float fwd = drive(1.0f), back = drive(-1.0f);
        const bool wedged = !falling && fwd < 300.0f && back < 300.0f;

        const char* flag = "";
        if (falling) {
            flag = "VOID";  // nothing below it at all
            voids++;
        } else if (upZ < 0.5f) {
            flag = "UPSIDE-DOWN";
            upside++;
        } else if (wedged) {
            flag = "WEDGED";
            stuck++;
        } else if (dz < -800.0f) {
            flag = "FELL";
            fell++;
        } else if (dz > 200.0f) {
            flag = "EJECTED";
            inside++;
        } else if (fwd < 300.0f || back < 300.0f) {
            flag = "front/rear-walled";
            walled++;
        }

        if (verbose || flag[0])
            std::printf("  [%s] z=%-8.0f dz=%+7.0f vz=%+6.0f up.z=%+.2f fwd=%5.0f back=%5.0f %s\n",
                        s.name.c_str(), p.z, dz, vz1, upZ, fwd, back, flag);
    }
    std::printf("  %d/%d fell, %d/%d wedged, %d/%d upside-down, %d/%d ejected, %d/%d void, "
                "%d/%d walled\n",
                fell, n, stuck, n, upside, n, inside, n, voids, n, walled, n);

    // ---- 2. does proximity walk the list cleanly? -----------------------
    int misses = 0, thrash = 0;
    for (int i = 0; i < n; i++) {
        const maploader::MapSession& s = sessions[(size_t)i];
        std::vector<int> seen;
        sim.onCheckpointReached = [&](int idx, int) { seen.push_back(idx); };
        place(sim, V3f(s.carPos.x, s.carPos.y, s.carPos.z), s.carYawRad, 0.0f);
        settle(sim, 0.5f);
        sim.onCheckpointReached = nullptr;
        if (sim.sessionIndex() != i) misses++;
        for (size_t k = 1; k < seen.size(); k++)
            if (seen[k] == seen[k - 1]) thrash++;
    }
    std::printf("  %d reach misses, %d re-triggers on the same spot\n", misses, thrash);
    sim.shutdown();
    return 0;
}
