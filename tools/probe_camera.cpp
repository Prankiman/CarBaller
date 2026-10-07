// Headless diagnostic: does the ball cam ever SNAP - move the eye or turn the
// view by a whole pose in a single frame - instead of easing into the new
// pose? And does it still keep the ball centered while easing?
//
// It also checks the car cam's swivel PIVOT: up/down has to rotate the rig
// around the car (eye moves, car holds its place on screen) the way left/right
// does, not tilt the view from a fixed eye, and the swinging eye must never
// drop through the floor.
//
// The degenerate cases are the interesting ones: whenever the ball passes
// over (or under) the car, its horizontal offset from the car runs through
// zero - where the bearing atan2 returns is undefined - so the ideal rig can
// ask for a pose on the far side of the car in one frame. The eye follower
// folds frame-to-frame rig speed into its follow rate, which turns any such
// jump into an instant snap (factor lands at ~1).
//
// Build (from repo root):
//   c++ -std=c++20 -O2 -I. -Isrc -Ithird_party tools/probe_camera.cpp src/camera/rl_camera.cpp -o probe_camera
// Run:
//   ./probe_camera           (exit 0 = no snap, no centering/pivot regression)
//
// Numbers before the ball-cam aim slew (kBallAimSlew) was added, same probe:
//   A overhead crossing : eyeStep 516 uu  rotStep 75 deg
//   C dribble teleport  : eyeStep 225 uu  rotStep 50 deg
//   E overhead drift    : eyeStep 534 uu  rotStep 95 deg

#include "src/camera/rl_camera.h"

#include <cmath>
#include <cstdio>

namespace {

constexpr float kFrameDt = 1.0f / 60.0f;

struct Metrics {
    float maxEyeStep = 0;   // uu the eye moved in a single frame
    float maxRotStep = 0;   // deg the view direction turned in a single frame
    float maxCenterErr = 0; // deg between view dir and true dir to the ball
    float rmsCenterErr = 0;
    int frames = 0;
};

// Per-scenario limits. eyeStep/rotStep are the anti-snap bounds: a pose
// change bigger than this in one frame reads as a cut, not a swing. The
// degenerate cases (A/C/E) are where the ideal rig itself used to jump; the
// legitimate sweeps (B/D/F/G/H) are bounded by physics instead - G's limits
// are what the unlimited rig already did while tracking a 4000 uu/s pass,
// since that motion is real, not a jump. center is how far the ball is
// allowed to drift off the view axis while the rig eases: ball cam exists to
// hold the ball in the middle, so the ease may cost a little centering in
// the degenerate cases but nothing in normal play.
struct Case {
    const char* name;
    SimSnapshot (*at)(float t);
    float seconds;
    float maxEyeStep;
    float maxRotStep;
    float maxCenter;
};

SimSnapshot makeSnap(V3 carPos, V3 carF, V3 ballPos) {
    SimSnapshot s;
    s.carPos = carPos;
    s.carF = carF;
    s.carR = V3(0, 1, 0);
    s.carU = V3(0, 0, 1);
    s.ballPos = ballPos;
    s.onGround = true;
    return s;
}

SimSnapshot baseCar() { return makeSnap(V3(0, 0, 17), V3(1, 0, 0), V3()); }

// A: ball flies straight over the car, ~2 uu off the car's vertical axis.
SimSnapshot overHead(float t) {
    SimSnapshot s = baseCar();
    s.ballPos = V3(2.0f, 700.0f - 2200.0f * t, 520.0f);
    return s;
}
// B: ordinary pass across the field, 600-900 uu away.
SimSnapshot normalPass(float t) {
    SimSnapshot s = baseCar();
    s.ballPos = V3(-500.0f + 900.0f * t, 800.0f - 300.0f * t,
                   300.0f + 200.0f * std::sin(t));
    return s;
}
// C: dribble preset - the ball teleports onto the roof at t = 0.25.
SimSnapshot dribbleTeleport(float t) {
    SimSnapshot s = baseCar();
    const V3 far(600, -700, 91.25f + 17);
    const V3 roof(20, 0, 17 + 20.755f + 38.6591f * 0.5f + 91.25f + 6.0f);
    s.ballPos = t < 0.25f ? far : roof;
    return s;
}
// D: ball parked dead ahead on the roof - must stay rigid, nothing to ease.
SimSnapshot dribbleHold(float) {
    SimSnapshot s = baseCar();
    s.ballPos = V3(20, 0, 17 + 20.755f + 38.6591f * 0.5f + 91.25f + 6.0f);
    return s;
}
// E: ball directly overhead drifting around - the bearing is pure noise.
SimSnapshot overheadDrift(float t) {
    SimSnapshot s = baseCar();
    s.ballPos = V3(std::sin(t * 9.0f) * 6.0f, std::cos(t * 7.0f) * 6.0f, 400.0f);
    return s;
}
// F: tight orbit at 400 uu - fast but fully legitimate tracking.
SimSnapshot orbit(float t) {
    SimSnapshot s = baseCar();
    s.ballPos = V3(400.0f * std::cos(t * 3.0f), 400.0f * std::sin(t * 3.0f), 250.0f);
    return s;
}
// G: 4000 uu/s ball whipping past at grazing range (150 uu off the axis) -
// the fastest bearing sweep real play can ask for.
SimSnapshot grazeFast(float t) {
    SimSnapshot s = baseCar();
    s.ballPos = V3(150.0f, 1400.0f - 4000.0f * t, 200.0f);
    return s;
}
// H: ball 600 uu out crossing the field at 2500 uu/s - common chase case.
SimSnapshot fastPass(float t) {
    SimSnapshot s = baseCar();
    s.ballPos = V3(600.0f, 1500.0f - 2500.0f * t, 300.0f);
    return s;
}

const Case kCases[] = {
    {"A overhead crossing",   overHead,        1.2f, 150, 25, 25},
    {"B normal pass",         normalPass,      1.5f,  40,  8,  8},
    {"C dribble teleport",    dribbleTeleport, 1.2f, 150, 25,  8},
    {"D dribble hold",        dribbleHold,     2.0f,  40,  8,  8},
    {"E overhead drift",      overheadDrift,   2.0f, 150, 25,  8},
    {"F tight orbit",         orbit,           3.0f,  40,  8,  8},
    {"G supersonic graze",    grazeFast,       0.8f, 120, 26,  8},
    {"H fast pass 600uu",     fastPass,        1.5f,  40,  8,  8},
};

Metrics run(const Case& c) {
    CameraSettings cs;  // shipped defaults
    RLCamera cam;
    cam.reset(c.at(0), cs);
    cam.setMode(CamMode::Ball);

    Metrics m;
    V3 prevEye = cam.eye;
    V3 prevDir = (cam.target - cam.eye).norm();
    double err2 = 0;
    for (int i = 0; i < int(c.seconds / kFrameDt); i++) {
        const SimSnapshot s = c.at(i * kFrameDt);
        cam.update(kFrameDt, cs, s, 0, 0);

        const V3 dir = (cam.target - cam.eye).norm();
        const float centerErr =
            std::acos(clampf(dir.dot((s.ballPos - cam.eye).norm()), -1.0f, 1.0f));
        const float eyeStep = (cam.eye - prevEye).len();
        const float rotStep =
            std::acos(clampf(dir.dot(prevDir), -1.0f, 1.0f)) * 57.2957795f;

        m.maxEyeStep = std::fmax(m.maxEyeStep, eyeStep);
        m.maxRotStep = std::fmax(m.maxRotStep, rotStep);
        m.maxCenterErr = std::fmax(m.maxCenterErr, centerErr * 57.2957795f);
        err2 += double(centerErr) * double(centerErr);
        m.frames++;
        prevEye = cam.eye;
        prevDir = dir;
    }
    if (m.frames) m.rmsCenterErr = float(std::sqrt(err2 / m.frames));
    return m;
}

// ---- car-cam swivel pivot. Up/down used to rotate the camera around its own
// starting point (fixed eye, tilting view: the car slid across the frame)
// while left/right orbited the car. Both axes now swing the rig on a sphere
// around the car, so over a window inside the swivel's orbit range the eye
// must travel a real distance AND the car must stay put on screen. The eye
// also never drops through the floor (the renderer draws it opaque), for the
// whole run, not just the window.
struct Swivel {
    const char* name;
    float swY;        // +1 = look up (swings the eye down), -1 = look down
    float carZ;       // car center height (uu)
    bool onGround;
    float minTravel;  // uu the eye must cover inside the centering window
};

const Swivel kSwivels[] = {
    {"up, ground",     1.0f,  17.0f, true,  60},
    {"down, ground",  -1.0f,  17.0f, true,  60},
    {"up, air",        1.0f, 500.0f, false, 60},
    {"down, air",     -1.0f, 500.0f, false, 60},
};

bool checkSwivelPivot() {
    constexpr float kWindow = 0.10f;  // s: pitchOff is still fully inside the
                                      // orbit range at default settings
    constexpr float kMaxOff = 8.0f;   // deg the car may drift off the view axis
    constexpr float kMinEyeZ = 15.5f; // uu: never through the arena floor

    bool ok = true;
    std::printf("\n=== car-cam swivel pivot (eye travels, car stays put) ===\n");
    for (const Swivel& v : kSwivels) {
        CameraSettings cs;  // shipped defaults
        RLCamera cam;
        SimSnapshot s = makeSnap(V3(0, 0, v.carZ), V3(1, 0, 0),
                                 V3(0, 0, v.carZ));
        s.onGround = v.onGround;
        cam.reset(s, cs);
        cam.setMode(CamMode::Car);
        cam.update(kFrameDt, cs, s, 0, 0);
        const float startZ = cam.eye.z;

        float travel = 0, maxOff = 0, minZ = cam.eye.z;
        for (int i = 0; i < int(1.0f / kFrameDt); i++) {
            cam.update(kFrameDt, cs, s, 0, v.swY);
            if (i * kFrameDt < kWindow) travel = std::fabs(cam.eye.z - startZ);

            const V3 toCar = s.carPos - cam.eye;
            const V3 dir = (cam.target - cam.eye).norm();
            const float horiz = std::sqrt(toCar.x * toCar.x + toCar.y * toCar.y);
            const float elevCar = std::atan2(toCar.z, horiz);
            const float elevView = std::atan2(dir.z,
                                              std::sqrt(dir.x * dir.x + dir.y * dir.y));
            if (i * kFrameDt < kWindow)
                maxOff = std::fmax(maxOff, std::fabs(elevCar - elevView) * 57.2957795f);
            minZ = std::fmin(minZ, cam.eye.z);
        }

        const bool okTravel = travel >= v.minTravel;
        const bool okCenter = maxOff <= kMaxOff;
        const bool okFloor = minZ >= kMinEyeZ;
        if (!(okTravel && okCenter && okFloor)) ok = false;
        std::printf("%-14s %s travel %5.1f/%3.0f uu  car off-axis %4.1f/%2.0f deg"
                    "  eye min z %6.1f uu\n",
                    v.name, (okTravel && okCenter && okFloor) ? "ok  " : "FAIL",
                    travel, v.minTravel, maxOff, kMaxOff, minZ);
    }
    return ok;
}

}  // namespace

int main() {
    int failed = 0;
    std::printf("=== carballer ball-cam snap probe (60fps frames) ===\n");
    for (const Case& c : kCases) {
        const Metrics m = run(c);
        const bool okEye = m.maxEyeStep <= c.maxEyeStep;
        const bool okRot = m.maxRotStep <= c.maxRotStep;
        const bool okCtr = m.maxCenterErr <= c.maxCenter;
        const bool ok = okEye && okRot && okCtr;
        if (!ok) failed++;
        std::printf("%-22s %s eyeStep %6.1f/%3.0f uu  rotStep %5.1f/%2.0f deg"
                    "  center %5.1f/%2.0f deg (rms %.2f)\n",
                    c.name, ok ? "ok  " : "FAIL", m.maxEyeStep, c.maxEyeStep,
                    m.maxRotStep, c.maxRotStep, m.maxCenterErr, c.maxCenter,
                    m.rmsCenterErr);
    }
    if (!checkSwivelPivot()) failed++;
    std::printf("\n%s\n", failed ? "SNAP/CENTERING/PIVOT REGRESSION" : "no snaps, centering ok, swivel orbits the car");
    return failed ? 1 : 0;
}
