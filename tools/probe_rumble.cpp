// Headless diagnostic: how much do the car/ball chatter during a dribble
// carry and during repeated hits, how often do ball-hit events fire, and how
// much of that reaches the rendered camera (screen-space jitter)?
//
// Build (from repo root):
//   c++ -std=c++20 -O2 -I. -Isrc tools/probe_rumble.cpp src/sim/sim.cpp \
//       src/camera/rl_camera.cpp build/RocketSim/libRocketSim.a -o probe_rumble
// Run:
//   ./probe_rumble [meshDir]

#include "src/camera/rl_camera.h"
#include "src/sim/sim.h"

#include "RocketSim/src/RLConst.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using RocketSim::BallState;
using RocketSim::CarControls;
using RocketSim::CarState;
using RocketSim::RotMat;
using RocketSim::Vec;

namespace {

constexpr double kTick = 1.0 / 120.0;
constexpr float kFrameDt = 1.0f / 60.0f;
// 1600px window at 110 deg horizontal FOV: 110 deg = 1.9199 rad -> 833 px/rad
constexpr float kPxPerRad = 1600.0f / (110.0f * (float)M_PI / 180.0f);

float angBetween(const V3& a, const V3& b) {
    float la = a.len(), lb = b.len();
    if (la < 1e-6f || lb < 1e-6f) return 0;
    float d = clampf(a.dot(b) / (la * lb), -1.0f, 1.0f);
    return std::acos(d);
}

struct TickSample {
    float carZ, carVz, ballZ, extra;
    V3 rel;            // ball - car (world)
    bool onRoof = false;
};
struct FrameSample {
    V3 eye, target, carPos, ballPos;
    bool onRoof = false;
};

struct PhaseResult {
    std::string name;
    int frames = 0;
    int hitEvents = 0;
    int steadyHits = 0;                // events after the first 2 s
    int maxHitsPer100ms = 0;
    float maxExtra = 0;                // |extraHitVel| seen (drives FX strength)
    float steadyMaxExtra = 0;
    // was the ball actually carried on the roof during steady state?
    int steadyTicks = 0, onRoofTicks = 0;
    V3 relMean;                        // steady ball - car (world)
    // tick level
    float maxTickDCarZ = 0, maxTickDBallZ = 0, maxTickDCarVz = 0;
    float carZmin = 1e9f, carZmax = -1e9f;
    // frame level (rendered at 60fps, steady state = after first second)
    float carPxMax = 0, carPxRms = 0;   // car motion within camera space
    float ballPxMax = 0, ballPxRms = 0; // ball motion within camera space
    float viewPxMax = 0, viewPxRms = 0; // view rotation (whole-screen shake)
    // ...restricted to frames where the ball rests ON THE ROOF (the dribble
    // case; other frames the ball is legitimately bouncing elsewhere)
    int roofFrames = 0;
    float viewRoofPxMax = 0, viewRoofPxRms = 0;
    float eyeDMax = 0;
};

void runPhase(Sim& sim, RLCamera& cam, const char* name, const char* mode,
              int frames, PhaseResult& out) {
    out = PhaseResult();
    out.name = name;
    out.frames = frames;

    // --- reset: car at midfield, ball parked or resting
    sim.takePosition(2);
    BallState bs;
    bs.rotMat = RotMat::GetIdentity();
    bs.vel = Vec(0, 0, 0);
    bs.angVel = Vec(0, 0, 0);
    if (std::strcmp(mode, "baseline") == 0)
        bs.pos = Vec(0, -30000, 100);   // far away: no interaction
    else
        bs.pos = Vec(0, 0, RocketSim::RLConst::BALL_REST_Z);
    sim.arena()->ball->SetState(bs);

    CameraSettings cs;                  // shipped defaults...
    cs.stiffness = 0.4f;                // ...with the user's tuned stiffness
    cs.distance = 270.0f;
    cs.height = 100.0f;
    cs.angle = -3.0f;
    cs.swivelSpeed = 4.0f;
    cs.transitionSpeed = 2.0f;

    cam.reset(sim.snapshot(0.0f));

    std::vector<TickSample> ticks;
    std::vector<FrameSample> frs;
    std::vector<int> hitsPerBin(frames / 6 + 2, 0);

    int hitTotal = 0;
    sim.onBallHit = [&](const BallHitEvent&) { hitTotal++; };

    const bool drive = std::strcmp(mode, "baseline") == 0 ||
                       std::strcmp(mode, "drivecarry") == 0;
    const bool isDribble = std::strcmp(mode, "carry") == 0 ||
                           std::strcmp(mode, "drivecarry") == 0;
    const bool isHits = std::strcmp(mode, "hits") == 0;

    for (int f = 0; f < frames; f++) {
        CarControls ctl;
        if (drive) ctl.throttle = 1.0f;
        sim.car()->controls = ctl;

        if (isDribble && ((std::strcmp(mode, "carry") == 0 && f == 10) ||
                          (std::strcmp(mode, "drivecarry") == 0 && f == 90))) {
            // gentle placement: ball centered on the roof, 2 uu gap,
            // inheriting velocity (0 stationary / cruising speed when driving)
            CarState csx = sim.car()->GetState();
            BallState bs2;
            bs2.rotMat = RotMat::GetIdentity();
            bs2.vel = csx.vel;
            bs2.angVel = Vec(0, 0, 0);
            const V3 up(csx.rotMat.up.x, csx.rotMat.up.y, csx.rotMat.up.z);
            V3 p(csx.pos.x, csx.pos.y, csx.pos.z);
            p = p + up * (20.755f + 38.6591f * 0.5f + 91.25f + 2.0f);
            bs2.pos = Vec(p.x, p.y, p.z);
            sim.arena()->ball->SetState(bs2);
        }

        if (isHits && f >= 60 && (f - 60) % 60 == 0) {
            // drop the ball straight onto the (stationary) car's roof:
            // 300 uu up, -200 uu/s -> lands ~0.48 s, before the next drop
            CarState csx = sim.car()->GetState();
            BallState drop;
            drop.rotMat = RotMat::GetIdentity();
            drop.vel = Vec(0, 0, -200);
            drop.angVel = Vec(0, 0, 0);
            drop.pos = Vec(csx.pos.x, csx.pos.y, csx.pos.z + 300);
            sim.arena()->ball->SetState(drop);
        }

        const int hitsBefore = hitTotal;
        bool frameOnRoof = true;
        for (int k = 0; k < 2; k++) {        // 2 ticks per frame, as in app.cpp
            sim.advance(kTick);
            const CarState c = sim.car()->GetState();
            const BallState b = sim.arena()->ball->GetState();
            TickSample ts;
            ts.carZ = c.pos.z;
            ts.carVz = c.vel.z;
            ts.ballZ = b.pos.z;
            ts.extra = c.ballHitInfo.isValid ? c.ballHitInfo.extraHitVel.Length()
                                             : 0.0f;
            ts.rel = V3(b.pos.x - c.pos.x, b.pos.y - c.pos.y, b.pos.z - c.pos.z);
            const V3 fwd(c.rotMat.forward.x, c.rotMat.forward.y, c.rotMat.forward.z);
            const V3 rgt(c.rotMat.right.x, c.rotMat.right.y, c.rotMat.right.z);
            ts.onRoof = ts.rel.z > 110.0f && std::fabs(ts.rel.dot(fwd)) < 130.0f &&
                        std::fabs(ts.rel.dot(rgt)) < 100.0f;
            frameOnRoof = frameOnRoof && ts.onRoof;
            ticks.push_back(ts);
            out.maxExtra = std::max(out.maxExtra, ts.extra);
        }
        if (hitTotal > hitsBefore) {
            int b = f / 6;                       // 100 ms bins
            if (b < (int)hitsPerBin.size()) hitsPerBin[b] += hitTotal - hitsBefore;
            if (f >= 120) out.steadyHits += hitTotal - hitsBefore;
        }

        // rendered frame, exactly like app.cpp
        const SimSnapshot snap = sim.snapshot(sim.accumAlpha());
        cam.update(kFrameDt, cs, snap, 0.0f, 0.0f);
        FrameSample fs;
        fs.eye = cam.eye;
        fs.target = cam.target;
        fs.carPos = snap.carPos;
        fs.ballPos = snap.ballPos;
        fs.onRoof = frameOnRoof;
        frs.push_back(fs);
    }

    out.hitEvents = hitTotal;
    for (int h : hitsPerBin) out.maxHitsPer100ms = std::max(out.maxHitsPer100ms, h);

    const int t0 = 240;   // skip first 2 s of settling
    for (int i = t0; i < (int)ticks.size(); i++) {
        const TickSample &a = ticks[i - 1], &b = ticks[i];
        out.maxTickDCarZ = std::max(out.maxTickDCarZ, std::fabs(b.carZ - a.carZ));
        out.maxTickDBallZ = std::max(out.maxTickDBallZ, std::fabs(b.ballZ - a.ballZ));
        out.maxTickDCarVz = std::max(out.maxTickDCarVz, std::fabs(b.carVz - a.carVz));
        out.steadyMaxExtra = std::max(out.steadyMaxExtra, b.extra);
        out.steadyTicks++;
        if (b.onRoof) out.onRoofTicks++;
        out.relMean = out.relMean + b.rel;
    }
    for (int i = t0; i < (int)ticks.size(); i++) {
        out.carZmin = std::min(out.carZmin, ticks[i].carZ);
        out.carZmax = std::max(out.carZmax, ticks[i].carZ);
    }
    if (out.steadyTicks > 0) out.relMean = out.relMean * (1.0f / out.steadyTicks);

    const int f0 = 60;    // skip first 1 s of settling
    double carAcc = 0, ballAcc = 0, viewAcc = 0, roofAcc = 0;
    int n = 0;
    for (int i = f0; i < (int)frs.size(); i++) {
        const FrameSample &a = frs[i - 1], &b = frs[i];
        // motion in camera space (relative to scene): each sample vs its own eye
        const float carPx = angBetween(b.carPos - b.eye, a.carPos - a.eye) * kPxPerRad;
        const float ballPx = angBetween(b.ballPos - b.eye, a.ballPos - a.eye) * kPxPerRad;
        const float viewPx = angBetween(b.target - b.eye, a.target - a.eye) * kPxPerRad;
        out.carPxMax = std::max(out.carPxMax, carPx);
        out.ballPxMax = std::max(out.ballPxMax, ballPx);
        out.viewPxMax = std::max(out.viewPxMax, viewPx);
        out.eyeDMax = std::max(out.eyeDMax, (b.eye - a.eye).len());
        carAcc += double(carPx) * carPx;
        ballAcc += double(ballPx) * ballPx;
        viewAcc += double(viewPx) * viewPx;
        n++;
        // dribble case: ball resting on the roof (both frames on-roof) ->
        // view wobble here is the "car rumble" the player perceives
        if (a.onRoof && b.onRoof) {
            out.roofFrames++;
            roofAcc += double(viewPx) * viewPx;
            out.viewRoofPxMax = std::max(out.viewRoofPxMax, viewPx);
        }
    }
    if (n > 0) {
        out.carPxRms = (float)std::sqrt(carAcc / n);
        out.ballPxRms = (float)std::sqrt(ballAcc / n);
        out.viewPxRms = (float)std::sqrt(viewAcc / n);
    }
    if (out.roofFrames > 0)
        out.viewRoofPxRms = (float)std::sqrt(roofAcc / out.roofFrames);
}

}  // namespace

int main(int argc, char** argv) {
    const std::string meshDir = argc > 1 ? argv[1] : "assets/collision_meshes";

    Sim sim;
    if (!sim.init(meshDir)) {
        std::fprintf(stderr, "[probe] sim init failed (%s)\n", meshDir.c_str());
        return 1;
    }
    RLCamera cam;

    struct Case { const char* name; const char* mode; int frames; };
    const Case cases[] = {
        {"baseline (drive, no ball)", "baseline", 180},
        {"dribble carry, stationary", "carry", 360},
        {"dribble carry, driving", "drivecarry", 300},
        {"repeated drops onto car", "hits", 360},
    };

    std::printf("=== carballer rumble probe ===\n");
    for (const Case& c : cases) {
        PhaseResult r;
        runPhase(sim, cam, c.name, c.mode, c.frames, r);
        const float sec = c.frames / 60.0f;
        std::printf("\n--- %s (%.1fs) ---\n", r.name.c_str(), sec);
        std::printf("  hit events      : %d total (%.1f/s), %d steady (after 2s), "
                    "max %d in any 100ms\n",
                    r.hitEvents, r.hitEvents / sec, r.steadyHits, r.maxHitsPer100ms);
        std::printf("  extraHitVel     : max %.0f uu/s (steady %.0f)\n",
                    r.maxExtra, r.steadyMaxExtra);
        std::printf("  ball position   : on roof %d%% of steady ticks, mean rel "
                    "(%.0f, %.0f, %.0f) uu\n",
                    r.steadyTicks ? (100 * r.onRoofTicks / r.steadyTicks) : 0,
                    r.relMean.x, r.relMean.y, r.relMean.z);
        std::printf("  tick chatter    : max |dCarZ|=%6.3f uu  |dBallZ|=%6.3f uu  "
                    "|dCarVz|=%6.1f uu/s\n",
                    r.maxTickDCarZ, r.maxTickDBallZ, r.maxTickDCarVz);
        std::printf("  car Z steady    : %.3f .. %.3f uu (pp %.3f uu)\n",
                    r.carZmin, r.carZmax, r.carZmax - r.carZmin);
        std::printf("  screen motion   : car max %5.2f px rms %4.2f | ball max %5.2f "
                    "px rms %4.2f | view rot max %5.2f px rms %4.2f\n",
                    r.carPxMax, r.carPxRms, r.ballPxMax, r.ballPxRms,
                    r.viewPxMax, r.viewPxRms);
        std::printf("  on-roof view    : %d frames, rot max %5.2f px rms %4.2f px\n",
                    r.roofFrames, r.viewRoofPxMax, r.viewRoofPxRms);
        std::printf("  camera eye step : max %.2f uu/frame\n", r.eyeDMax);
    }
    sim.shutdown();
    return 0;
}
