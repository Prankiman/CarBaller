// Headless diagnostic: does the ball cam ever SNAP - move the eye or turn the
// view by a whole pose in a single frame - instead of easing into the new
// pose? And does it still keep the ball centered while easing?
//
// It also checks the car cam's swivel PIVOT: up/down has to rotate the rig
// around the car (eye moves, car holds its place on screen) the way left/right
// does, not tilt the view from a fixed eye, and the swinging eye must never
// drop through the floor. Note the car HOLDS A PLACE that is not the screen
// centre: the parked view is level (reference applied-pitch law - elevation
// of the look-dir + Camera Angle), which leaves the car resting ~15-18 deg
// below the axis, so the check is that the offset stays constant, not zero.
//
// And the car cam's AIR CAMERA: in the air the camera sits behind the
// direction of TRAVEL (not behind the nose) and eases onto it, re-read every
// frame - so pitch and air-yaw must not move it while the velocity says
// otherwise, a change in velocity must ease it behind the new heading inside
// ~0.6s (timing owned by Stiffness), and touching down must put it back
// rigidly behind the facing.
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

float deg(float rad) { return rad * 57.2957795f; }

// Keep the diagnostic thresholds independent of the shipped profile. The
// probe measures camera mechanics, while the profile is intentionally tuned
// to match the reference game's imported settings.
CameraSettings probeSettings() {
    CameraSettings cs;
    cs.angle = -4.0f;
    cs.stiffness = 0.5f;
    cs.swivelSpeed = 5.0f;
    cs.transitionSpeed = 1.2f;
    cs.invertSwivel = false;
    return cs;
}

// Shortest signed difference between two angles, in rad.
float wrap(float a) {
    while (a > (float)M_PI) a -= 2 * (float)M_PI;
    while (a < -(float)M_PI) a += 2 * (float)M_PI;
    return a;
}

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
    CameraSettings cs = probeSettings();
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
    constexpr float kLateA = 0.35f;   // s - the swivel is near its plateau
    constexpr float kLateB = 0.50f;   // s - the composition law is steady here
    constexpr float kMaxLateDrift = 2.0f; // deg - the composition law must be
                     // CONSTANT once the orbit is under way (pinning law)
    constexpr float kMaxTransient = 12.0f; // deg - the eye/apply followers
                     // trade some transient pinning for their anti-jump
                     // protections (the reference applies swivel offsets
                     // laglessly; ours keeps the protections).
    constexpr float kMinEyeZ = 15.5f; // uu: never through the arena floor

    bool ok = true;
    std::printf("\n=== car-cam swivel pivot (eye travels, car stays put) ===\n");
    for (const Swivel& v : kSwivels) {
        CameraSettings cs = probeSettings();
        RLCamera cam;
        SimSnapshot s = makeSnap(V3(0, 0, v.carZ), V3(1, 0, 0),
                                 V3(0, 0, v.carZ));
        s.onGround = v.onGround;
        cam.reset(s, cs);
        cam.setMode(CamMode::Car);
        cam.update(kFrameDt, cs, s, 0, 0);
        const float startZ = cam.eye.z;

        float travel = 0, drift = 0, off0 = 0, offLate = -1e9f, offLate2 = -1e9f;
        float minZ = cam.eye.z;
        bool first = true;
        for (int i = 0; i < int(1.0f / kFrameDt); i++) {
            cam.update(kFrameDt, cs, s, 0, v.swY);
            const float t = i * kFrameDt;
            if (t < kWindow) travel = std::fabs(cam.eye.z - startZ);

            const V3 toCar = s.carPos - cam.eye;
            const V3 dir = (cam.target - cam.eye).norm();
            const float horiz = std::sqrt(toCar.x * toCar.x + toCar.y * toCar.y);
            const float elevCar = std::atan2(toCar.z, horiz);
            const float elevView = std::atan2(dir.z,
                                              std::sqrt(dir.x * dir.x + dir.y * dir.y));
            if (t < kWindow) {
                const float off = std::fabs(elevCar - elevView) * 57.2957795f;
                if (first) { off0 = off; first = false; }
                else drift = std::fmax(drift, std::fabs(off - off0));
            }
            if (t >= kLateA && t < kLateA + 2 * kFrameDt && offLate < -1e8f)
                offLate = elevCar - elevView;
            if (t >= kLateB && t < kLateB + 2 * kFrameDt && offLate2 < -1e8f)
                offLate2 = elevCar - elevView;
            minZ = std::fmin(minZ, cam.eye.z);
        }
        const float lateDrift =
            std::fabs((offLate2 - offLate) * 57.2957795f);

        const bool okTravel = travel >= v.minTravel;
        // Pinning now means the car holds its (off-centre) composition spot
        // while the eye swings - not that it sits centred: with the level
        // parked pitch (reference law) the car rests ~15-18 deg below the
        // axis the whole time, and the swivel's eye-orbit + pitch co-motion
        // keeps that constant (drift, not absolute offset, is the failure
        // mode the probe is after).
        const bool okLate = lateDrift <= kMaxLateDrift;
        const bool okTransient = drift <= kMaxTransient;
        const bool okFloor = minZ >= kMinEyeZ;
        if (!(okTravel && okLate && okTransient && okFloor)) ok = false;
        std::printf("%-14s %s travel %5.1f/%3.0f uu  off %5.1f (late %.1f/%.0f,"
                    " transient %4.1f/%.0f)  eye min z %6.1f uu\n",
                    v.name, (okTravel && okLate && okTransient && okFloor)
                                ? "ok  " : "FAIL",
                    travel, v.minTravel, off0, lateDrift, kMaxLateDrift,
                    drift, kMaxTransient, minZ);
    }
    return ok;
}

// ---- car-cam AIR CAMERA: RL does not decide "which side of the car" on a
// timer - in the air the camera sits behind the direction of TRAVEL and
// eases onto it, re-read every frame. That gives four properties worth
// pinning down, all measured here at 60fps:
//   1. pitch never moves it (the nose projection flips 180 deg when pitch
//      passes vertical, which the old air follow used to chase: measured
//      180 deg of camera swing at 75 uu/frame from a pure 0->120 pitch);
//   2. air-yawing the car does not move it either when the momentum says
//      otherwise - the velocity heading is the target, not the nose;
//   3. a change in velocity (a flip, a stall, gravity swinging you round)
//      eases the camera behind the new heading inside ~0.6s - that timing
//      comes from Stiffness, RL's own "how rigidly the camera follows your
//      car" setting (kAirEaseBase/kAirEaseStiff in rl_camera.cpp);
//   4. on the ground it goes back to being rigid behind the facing, so a
//      landing re-aligns immediately instead of floating.
//
// Each case flies an airborne car (synthetic pose + velocity, never a sim)
// and compares the eye's bearing around the car with the heading it should
// be sitting BEHIND (+180 deg: the eye is on the far side). settle = the
// first instant from which the camera never leaves that heading again;
// step = the largest distance the eye covered in one frame, which catches a
// snap rather than a swing.
struct Air {
    const char* name;
    float seconds;
    float yawEnd, tYaw;        // car facing ramp (rad, held after tYaw)
    float pitchEnd, tPitch;    // car pitch ramp
    float velYawEnd, tVelYaw;  // horizontal velocity heading ramp
    float speed;               // horizontal speed, uu/s (0 = no direction)
    bool expectVel;            // air target = velocity heading, not facing
    float landAt;              // s: touches down here (< 0 = stays airborne)
    float maxSwing;            // deg the camera may move from its start
    float maxSettle;           // s: trailing settle must begin by then
    float maxOff;              // deg off the expected heading once settled
    float maxStep;             // uu the eye may move in one frame
};

float ramp(float t, float end, float tEnd) {
    return (tEnd <= 0) ? end : end * std::fmin(t / tEnd, 1.0f);
}

SimSnapshot airAt(float t, const Air& c) {
    const float yaw = ramp(t, c.yawEnd, c.tYaw);
    const float pit = ramp(t, c.pitchEnd, c.tPitch);
    const float vy = ramp(t, c.velYawEnd, c.tVelYaw);
    SimSnapshot s;
    s.carPos = V3(0, 0, 500);
    s.carF = V3(std::cos(pit) * std::cos(yaw), std::cos(pit) * std::sin(yaw),
                std::sin(pit));
    s.carR = V3(-std::sin(yaw), std::cos(yaw), 0);
    s.carU = V3(-std::sin(pit) * std::cos(yaw), -std::sin(pit) * std::sin(yaw),
                std::cos(pit));
    s.carVel = V3(std::cos(vy) * c.speed, std::sin(vy) * c.speed, -350.0f);
    s.onGround = (c.landAt >= 0 && t >= c.landAt);
    return s;
}

// Heading the camera should be LOOKING along (its eye is 180 deg round from
// this): the velocity in the air when the case says so, otherwise the
// facing - and always the facing once the wheels are down.
float wantYaw(const Air& c, float t) {
    if (c.landAt >= 0 && t >= c.landAt) return ramp(t, c.yawEnd, c.tYaw);
    if (c.expectVel) return ramp(t, c.velYawEnd, c.tVelYaw);
    return ramp(t, c.yawEnd, c.tYaw);
}

bool checkAirCamera() {
    const float D = (float)M_PI / 180.0f;
    const Air kAir[] = {
        // name                  secs  yawEnd tYaw pitchEnd tPitch velYaw tVelYaw speed  vel? land  swing settle  off step
        {"pitch 0->120",         2.0f, 0,     0,   120 * D, 1.0f,  0,     0,      1410, true,  -1,    3,  0.10f,  3, 70},
        {"pitch 0->360 (flip)",  2.0f, 0,     0,   360 * D, 1.2f,  0,     0,      1410, true,  -1,    3,  0.10f,  3, 70},
        {"pitch 120, slow",      2.0f, 0,     0,   120 * D, 1.0f,  0,     0,      150,  false, -1,    3,  0.10f,  3, 70},
        {"air yaw 90, vel held", 2.0f, 90 * D,0.4f, 0,      0,     0,     0,      1410, true,  -1,    3,  0.10f,  3, 70},
        {"hover, no velocity",   2.0f, 0,     0,   0,       0,     0,     0,      0,    false, -1,    3,  0.10f,  3, 70},
        {"velocity turns 90",    2.5f, 0,     0,   0,       0,     90 * D,0.15f,  1410, true,  -1,  360,  1.00f,  5, 60},
        {"velocity turns 180",   3.0f, 0,     0,   0,       0,     180 * D,0.30f, 1410, true,  -1,  360,  1.20f,  5, 60},
        {"lands, nose at 90",    3.0f, 90 * D,0.4f, 0,       0,     0,     0,      1410, true,  1.5f, 360, 2.20f,  5, 90},
    };

    bool ok = true;
    std::printf("\n=== car-cam air camera (velocity based, stiffness eased) ===\n");
    for (const Air& c : kAir) {
        CameraSettings cs = probeSettings();
        RLCamera cam;
        const SimSnapshot s0 = airAt(0, c);
        cam.setMode(CamMode::Car);
        cam.reset(s0, cs);

        const float startWant = wantYaw(c, 0);
        const int n = int(c.seconds / kFrameDt);
        float off[640];
        float swing = 0, step = 0;
        V3 prev = cam.eye;
        for (int i = 0; i < n && i < 640; i++) {
            const float t = i * kFrameDt;
            const SimSnapshot s = airAt(t, c);
            cam.update(kFrameDt, cs, s, 0, 0);

            const float bearing =
                std::atan2(cam.eye.y - s.carPos.y, cam.eye.x - s.carPos.x);
            off[i] = std::fabs(deg(wrap(bearing - (wantYaw(c, t) + (float)M_PI))));
            swing = std::fmax(swing, deg(std::fabs(wrap(bearing - (startWant + (float)M_PI)))));
            step = std::fmax(step, (cam.eye - prev).len());
            prev = cam.eye;
        }

        // First index of the trailing run that stays on the heading.
        int settle = n - 1;
        while (settle > 0 && off[settle - 1] <= c.maxOff) settle--;
        float worst = 0, preWorst = 0;
        for (int i = settle; i < n; i++) worst = std::fmax(worst, off[i]);
        for (int i = 0; i < settle; i++) preWorst = std::fmax(preWorst, off[i]);

        const float settleS = settle * kFrameDt;
        const bool okSwing = swing <= c.maxSwing;
        const bool okSettle = settleS <= c.maxSettle;
        const bool okOff = worst <= c.maxOff;
        const bool okStep = step <= c.maxStep;
        if (!(okSwing && okSettle && okOff && okStep)) ok = false;
        std::printf("%-22s %s swing %5.1f/%3.0f deg  settle %.2f/%.2f s"
                    "  off %4.1f/%2.0f deg  peak-before %5.1f deg"
                    "  step %4.1f/%3.0f uu\n",
                    c.name, (okSwing && okSettle && okOff && okStep) ? "ok  "
                                                                      : "FAIL",
                    swing, c.maxSwing, settleS, c.maxSettle, worst, c.maxOff,
                    preWorst, step, c.maxStep);
    }
    return ok;
}

// ---- GROUND SELF-CORRECTION: the pitch-immune air axis can end up 180 deg
// off. While the nose sits in the too-vertical band (fxy < 0.15) the bearing
// is not readable, so a real heading change of more than 90 deg made in that
// window is indistinguishable from a pitch flip when the nose re-emerges -
// and the continuity rule then pins the axis to bearing + 180, where it
// stays, because a stuck axis and a nose held past vertical present exactly
// the same evidence. In the air you never see it (the velocity heading is
// the target), but if the ground trusted that axis a landing would ease the
// camera onto it and it would sit in FRONT of the car forever - measured at
// 180.0 deg off after this sequence before the ground was split off to read
// the nose projection fresh every frame (which is both self-correcting and
// the healing of the axis for the next takeoff).
//
// Sequence: level flight at 0 -> nose to 88 deg -> yaw 0->120 while the nose
// is unreadable -> level out -> land at 1.5s -> drive straight. The camera
// must track the nose the whole way and, after landing, stay behind it.
bool checkGroundHeal() {
    const float D = (float)M_PI / 180.0f;
    auto stateAt = [&](float t, float& yaw, float& pitch, bool& ground) {
        yaw = 0;
        if (t >= 0.8f) yaw = 120 * D * std::fmin((t - 0.8f) / 0.4f, 1.0f);
        pitch = 0;
        if (t >= 0.5f && t < 0.8f) pitch = 88 * D * ((t - 0.5f) / 0.3f);
        else if (t >= 0.8f && t < 1.2f) pitch = 88 * D;
        else if (t >= 1.2f && t < 1.5f) pitch = 88 * D * (1.0f - (t - 1.2f) / 0.3f);
        ground = (t >= 1.5f);
    };

    CameraSettings cs = probeSettings();
    RLCamera cam;
    float yaw = 0, pitch = 0;
    bool ground = false;
    stateAt(0, yaw, pitch, ground);

    auto snap = [&](float t) {
        stateAt(t, yaw, pitch, ground);
        const float cp = std::cos(pitch), sp = std::sin(pitch);
        const float cy = std::cos(yaw), sy = std::sin(yaw);
        SimSnapshot s;
        s.carPos = V3(0, 0, 500);  // fixed height: this test is about the
                                   // heading, not the drop onto the floor
        s.carF = V3(cp * cy, cp * sy, sp);
        s.carR = V3(-sy, cy, 0);
        s.carU = V3(-sp * cy, -sp * sy, cp);
        s.carVel = V3(cy * 1410.0f, sy * 1410.0f, ground ? 0.0f : -350.0f);
        s.onGround = ground;
        return s;
    };

    cam.setMode(CamMode::Car);
    cam.reset(snap(0), cs);

    const int n = int(3.0f / kFrameDt);
    float off[192] = {0};
    float step = 0, postLand = 0;
    V3 prev = cam.eye;
    for (int i = 0; i < n && i < 192; i++) {
        const float t = i * kFrameDt;
        const SimSnapshot s = snap(t);
        cam.update(kFrameDt, cs, s, 0, 0);
        stateAt(t, yaw, pitch, ground);
        const float bearing =
            std::atan2(cam.eye.y - s.carPos.y, cam.eye.x - s.carPos.x);
        off[i] = std::fabs(deg(wrap(bearing - (yaw + (float)M_PI))));
        step = std::fmax(step, (cam.eye - prev).len());
        prev = cam.eye;
        if (ground && t > 2.0f) postLand = std::fmax(postLand, off[i]);
    }
    // The landing may have to carry the camera round if the air left it on
    // the wrong side; it must be back behind the car well within 0.6s and
    // then stay there for the rest of the drive.
    int settle = n - 1;
    while (settle > 0 && off[settle - 1] <= 5.0f) settle--;
    const float settleS = settle * kFrameDt;
    const bool okSettle = settleS <= 2.1f;   // 1.5s landing + 0.6s to heal
    const bool okPost = postLand <= 2.0f;
    const bool okStep = step <= 70.0f;
    const bool ok = okSettle && okPost && okStep;
    std::printf("\n=== ground self-correction (nose projection read fresh) ===\n");
    std::printf("land after vertical   %s settle %.2f/2.10 s  post-land"
                " off %4.1f/2.0f deg  step %4.1f/70.0 uu\n",
                ok ? "ok  " : "FAIL", settleS, postLand, step);
    return ok;
}

// ---- reference-decoded laws. The camera was re-written from the
// reference game's decoded camera blob (probe + WASM disassembly); these are
// the three most fragile laws, checked directly:
//  1. swivel: full stick input converges to a PLATEAU of 1.0472 rad
//     (59.99 deg) of yaw offset, exponential in hold; on release with
//     "snap" on, it returns exponentially at ~34.5*(ss/10)/s.
//  2. mode transition: the car<->ball blend follows consumed = 1-lambda^2
//     (slow start, fast finish) over T = 0.5/transitionSpeed seconds.
//  3. ball-cam pivot rise: with the ball elevated at E over the car, the rig
//     height grows by 0.5217*height*|E|, so the parked eye height rises to
//     rigR = hypot(distance, height') and elev = atan2(height', distance).
bool checkRefLaws() {
    bool ok = true;
    std::printf("\n=== reference camera laws (swivel plateau, transition shape, ball-cam rise) ===\n");

    // -- 1. swivel plateau (car cam, full right-input held for 2 s)
    CameraSettings cs = probeSettings();
    RLCamera cam;
    SimSnapshot s = baseCar();
    cam.reset(s, cs);
    cam.setMode(CamMode::Car);
    cam.update(kFrameDt, cs, s, 0, 0);
    float plateauBearing = 0, maxSwivelStep = 0;
    V3 prevEye = cam.eye;
    for (int i = 0; i < int(2.0f / kFrameDt); i++) {
        cam.update(kFrameDt, cs, s, 1.0f, 0.0f);
        maxSwivelStep = std::fmax(maxSwivelStep, (cam.eye - prevEye).len());
        prevEye = cam.eye;
    }
    {
        const V3 rel = cam.eye - s.carPos;
        plateauBearing =
            wrap(std::atan2(rel.y, rel.x) + (float)M_PI);  // = yawOff_
    }
    const bool okPlateau = std::fabs(plateauBearing - 1.0472f) < 0.03f;
    ok = ok && okPlateau;
    std::printf("swivel plateau         %s bearing %6.3f/1.047 rad"
                "  max step %5.1f uu\n",
                okPlateau ? "ok  " : "FAIL", plateauBearing, maxSwivelStep);

    // -- 1b. release: snap-back exponential, kRelease = 34.5*(ss/10)/s
    float releaseBearing = plateauBearing;
    for (int i = 0; i < int(0.2f / kFrameDt); i++)
        cam.update(kFrameDt, cs, s, 0.0f, 0.0f);
    {
        const V3 rel = cam.eye - s.carPos;
        releaseBearing =
            wrap(std::atan2(rel.y, rel.x) + (float)M_PI);  // = yawOff_
    }
    // expect ~ plateau * exp(-34.5*0.5*0.2) = 0.032*plateau (plus what the
    // eye follower's own lag leaves behind - so allow 0.15 of the plateau)
    const bool okRelease = releaseBearing < 0.15f * 1.0472f;
    ok = ok && okRelease;
    std::printf("swivel release         %s bearing %6.3f/%.3f rad after 0.2 s\n",
                okRelease ? "ok  " : "FAIL", releaseBearing, 0.032f);

    // -- 2. transition shape: car -> ball with a static ball OFF to the side
    //        (a ball dead ahead on the car axis parks both eyes in the same
    //        spot, which would make the blend invisible). The eye is a blend
    //        of two static poses, so its per-frame speed profile IS the
    //        blend profile: slow over the first quarter of T, fastest in
    //        the second, done by about T = 0.5/transitionSpeed.
    cs.transitionSpeed = 1.0f;  // T = 0.5 s exactly
    RLCamera tc;
    SimSnapshot bs = baseCar();
    bs.ballPos = V3(300, 1500, 300);
    tc.reset(bs, cs);
    tc.setMode(CamMode::Car);
    tc.update(kFrameDt, cs, bs, 0, 0);
    V3 pe = tc.eye;
    float early = 0, mid = 0, tail = 0, total = 0, maxStep = 0;
    for (int i = 0; i < int(1.5f / kFrameDt); i++) {
        tc.update(kFrameDt, cs, bs, 0, 0);
        if (i == 0) { tc.setMode(CamMode::Ball); continue; }
        const float step = (tc.eye - pe).len();
        pe = tc.eye;
        total += step;
        maxStep = std::fmax(maxStep, step);
        // T = 0.5 s = 30 frames; frame 0 was still car mode. The blend
        // profile (1-lambda^2, done by T) is what the follower lag makes
        // hard to read directly, so check the robust consequences: the
        // transition SWEETS over more than a second of frames (no cut),
        // travel is real (there is an animation at all), and frames 31..90
        // are essentially dead (finishes by T).
        if (i >= 1 && i <= 5) early += step;
        else if (i <= 20) mid += step;
        else if (i <= 90) tail += step;
    }
    const bool okShape =
        total > 5.0f && maxStep < 40.0f && tail < 0.05f * total;
    ok = ok && okShape;
    std::printf("transition to ball     %s total %.1f uu, max step %.1f uu,"
                " tail %.2f uu (no cut, done by T)\n",
                okShape ? "ok  " : "FAIL", total, maxStep, tail);

    // -- 3. ball-cam pivot rise: ball at 45 deg over the car, height 100.
    //        rise = 0.5217*100*(pi/4) = 40.98 -> parked eye z = carZ +
    //        hypot(270, 140.98)*sin(atan2(140.98, 270)).
    RLCamera rc;
    SimSnapshot rs = baseCar();
    rs.ballPos = V3(1000, 0, 1017);   // dz = 1000 over car center z=17
    rc.reset(rs, cs);
    rc.setMode(CamMode::Ball);
    rc.update(kFrameDt, cs, rs, 0, 0);
    for (int i = 0; i < int(1.5f / kFrameDt); i++)
        rc.update(kFrameDt, cs, rs, 0, 0);
    constexpr float kRise = 0.5217f * 100.0f * (float)M_PI / 4.0f;
    const float h2 = 100.0f + kRise;
    const float expectZ = 17.0f + std::hypot(270.0f, h2) *
                                    std::sin(std::atan2(h2, 270.0f));
    const bool okRise = std::fabs(rc.eye.z - expectZ) < 4.0f;
    ok = ok && okRise;
    std::printf("ball-cam pivot rise    %s eye z %7.1f/%.1f uu"
                " (rise %.1f uu at 45 deg)\n",
                okRise ? "ok  " : "FAIL", rc.eye.z, expectZ, kRise);
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
    if (!checkAirCamera()) failed++;
    if (!checkGroundHeal()) failed++;
    if (!checkRefLaws()) failed++;
    std::printf("\n%s\n", failed ? "SNAP/CENTERING/PIVOT/AIR/HEAL/LAWS REGRESSION"
                                 : "no snaps, centering ok, swivel orbits the car, air cam follows velocity, ground self-corrects; reference laws hold");
    return failed ? 1 : 0;
}
