// Camera model probe: exercises the turball.io-style camera
// (src/camera/rl_camera.cpp) with synthetic snapshots and pins down the
// behaviour the port is supposed to have:
//
//   * no NaN / no runaway pose over a long chaotic trace
//   * ball cam keeps the ball on the view axis (bearing-wise) and pitches
//     toward an elevated ball
//   * car cam sits behind the car's facing on the ground, and swings behind
//     the direction of TRAVEL in the air
//   * mode switches blend from the old pose instead of cutting
//   * FOV widens with speed and pops at supersonic
//   * swivel offsets the view, then recenters faster than it moved out
//   * ground roll banks the camera up-vector ~10% of the car's roll, and the
//     bank decays in the air
//
// Build (from repo root):
//   c++ -std=c++20 -O2 -I. -Isrc -Ithird_party tools/probe_camera.cpp \
//       src/camera/rl_camera.cpp -o probe_camera
// Run:
//   ./probe_camera          (exit 0 = all checks pass)
#include "src/camera/rl_camera.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDt = 1.0f / 60.0f;

int failed = 0;

void check(bool ok, const char* what, const char* detail) {
    if (!ok) failed++;
    std::printf("%-56s %s  %s\n", what, ok ? "ok  " : "FAIL", detail);
}

float deg(float r) { return r * 180.0f / kPi; }
float wrap(float a) {
    while (a > kPi) a -= 2.0f * kPi;
    while (a < -kPi) a += 2.0f * kPi;
    return a;
}
V3 dirOf(const RLCamera& c) { return (c.target - c.eye).norm(); }
float horizYaw(const V3& d) { return std::atan2(d.y, d.x); }
bool finite(const V3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// Build a car frame from yaw (about world up), pitch (nose up) and roll
// (about the forward axis).
struct Frame {
    V3 f, r, u;
};

Frame frameOf(float yaw, float pitch, float roll) {
    // Start with the car level, facing +X (right = +Y, up = +Z), then pitch
    // about the right axis and roll about the resulting forward axis.
    V3 f(1, 0, 0), r(0, 1, 0), u(0, 0, 1);
    const float cp = std::cos(pitch), sp = std::sin(pitch);
    f = V3(f.x * cp, f.y * cp, sp);            // nose up/down
    u = V3(-sp, 0, cp);
    const float cr = std::cos(roll), sr = std::sin(roll);
    r = V3(r.x * cr + u.x * sr, r.y * cr + u.y * sr, r.z * cr + u.z * sr);
    u = f.cross(r);                            // right-handed: f x r = up
    // yaw the whole frame about world Z
    const float cy = std::cos(yaw), sy = std::sin(yaw);
    auto rotZ = [&](const V3& v) { return V3(v.x * cy - v.y * sy, v.x * sy + v.y * cy, v.z); };
    return {rotZ(f), rotZ(r), rotZ(u)};
}

SimSnapshot makeSnap(const V3& pos, const Frame& fr, const V3& ball,
                     const V3& vel = V3(), bool ground = true,
                     bool supersonic = false) {
    SimSnapshot s;
    s.carPos = pos;
    s.carF = fr.f;
    s.carR = fr.r;
    s.carU = fr.u;
    s.carVel = vel;
    s.ballPos = ball;
    s.ballVel = V3();
    s.onGround = ground;
    s.supersonic = supersonic;
    return s;
}

void step(RLCamera& cam, const CameraSettings& cs, const SimSnapshot& s,
          int frames, float swX = 0, float swY = 0) {
    for (int i = 0; i < frames; i++) cam.update(kDt, cs, s, swX, swY);
}

CameraSettings probeSettings() {
    CameraSettings cs;
    cs.fov = 110;
    cs.distance = 270;
    cs.height = 100;
    cs.angle = -3;
    cs.stiffness = 0.35f;
    cs.swivelSpeed = 4;
    cs.transitionSpeed = 2;
    cs.invertSwivel = true;
    return cs;
}

}  // namespace

int main() {
    const CameraSettings cs = probeSettings();
    const Frame level = frameOf(0, 0, 0);
    const Frame level90 = frameOf(kPi * 0.5f, 0, 0);

    // ---- 1. ball cam: ball dead ahead -> ball on the view axis horizontally
    {
        RLCamera cam;
        SimSnapshot s = makeSnap(V3(0, 0, 17), level, V3(3500, 0, 300));
        cam.reset(s, cs);
        step(cam, cs, s, 180);
        const V3 d = dirOf(cam);
        const V3 toBall = (s.ballPos - cam.eye).norm();
        const float err = deg(std::acos(clampf(
            V3(d.x, d.y, 0).norm().dot(V3(toBall.x, toBall.y, 0).norm()), -1.0f, 1.0f)));
        char buf[128];
        std::snprintf(buf, sizeof buf, "horizontal view-to-ball %.3f deg, eye z %.1f", err, cam.eye.z);
        check(err < 1.0f && finite(cam.eye) && finite(cam.target),
              "ball cam keeps a grounded ball on the view axis", buf);
    }

    // ---- 2. ball cam pitches up for an elevated ball
    {
        RLCamera low, high;
        SimSnapshot sLow = makeSnap(V3(0, 0, 17), level, V3(2200, 0, 100));
        SimSnapshot sHigh = makeSnap(V3(0, 0, 17), level, V3(2200, 0, 1600));
        low.reset(sLow, cs);
        high.reset(sHigh, cs);
        step(low, cs, sLow, 240);
        step(high, cs, sHigh, 240);
        const float pLow = std::asin(clampf(dirOf(low).z, -1, 1));
        const float pHigh = std::asin(clampf(dirOf(high).z, -1, 1));
        const V3 dHigh = dirOf(high);
        const V3 toBall = (sHigh.ballPos - high.eye).norm();
        const float err = deg(std::acos(clampf(
            V3(dHigh.x, dHigh.y, 0).norm().dot(V3(toBall.x, toBall.y, 0).norm()), -1.0f, 1.0f)));
        char buf[160];
        std::snprintf(buf, sizeof buf,
                      "pitch low %.2f / high %.2f deg, horiz err %.2f, eye z %.1f",
                      deg(pLow), deg(pHigh), err, high.eye.z);
        check(deg(pHigh) > deg(pLow) + 8.0f && err < 1.0f && finite(high.eye),
              "ball cam tilts up toward an elevated ball", buf);
    }

    // ---- 3. car cam on the ground: behind the car, looking along the nose
    {
        RLCamera cam;
        SimSnapshot s = makeSnap(V3(0, 0, 17), level, V3(4000, 0, 100));
        cam.reset(s, cs);
        cam.setMode(CamMode::Car);
        step(cam, cs, s, 240);
        const V3 d = dirOf(cam);
        const V3 rel = cam.eye - s.carPos;
        char buf[160];
        std::snprintf(buf, sizeof buf,
                      "behind %.1f uu, view-nose horiz %.1f deg, eye z %.1f",
                      rel.dot(s.carF), deg(std::acos(clampf(
                          V3(d.x, d.y, 0).norm().dot(s.carF), -1.0f, 1.0f))),
                      cam.eye.z);
        check(rel.dot(s.carF) < -150.0f && d.x > 0.95f && cam.eye.z > s.carPos.z + 50.0f,
              "car cam on the ground sits behind the nose", buf);
    }

    // ---- 4. car cam in the air swings behind the direction of travel
    {
        RLCamera cam;
        SimSnapshot s = makeSnap(V3(0, 0, 800), level, V3(4000, 0, 100),
                                 V3(0, 1600, 0), false);
        cam.reset(s, cs);
        cam.setMode(CamMode::Car);
        step(cam, cs, s, 180);
        const V3 d = dirOf(cam);
        const float yaw = horizYaw(d);
        char buf[128];
        std::snprintf(buf, sizeof buf, "view yaw %.1f deg (travel +Y), dir.y %.3f",
                      deg(yaw), d.y);
        check(deg(yaw) > 45.0f && d.y > 0.7f,
              "car cam swings behind the velocity in the air", buf);
    }

    // ---- 5. mode switch blends out of the old pose instead of cutting
    {
        RLCamera cam;
        // Ball off to the side so ball cam and car cam settle far apart.
        SimSnapshot s = makeSnap(V3(0, 0, 17), level, V3(0, 3000, 300));
        cam.reset(s, cs);
        step(cam, cs, s, 180);                       // settled ball cam
        const V3 before = cam.eye;
        cam.setMode(CamMode::Car);
        cam.update(kDt, cs, s, 0, 0);
        const float cut = (cam.eye - before).len();
        const bool wasBlending = cam.transitioning();
        step(cam, cs, s, 60);
        const V3 after = cam.eye;
        const float settled = (after - before).len();
        char buf[160];
        std::snprintf(buf, sizeof buf,
                      "first-frame step %.2f uu, settled %.1f uu, blending %d",
                      cut, settled, wasBlending ? 1 : 0);
        check(cut < 12.0f && settled > 80.0f && wasBlending && !cam.transitioning(),
              "mode switch eases from the old pose to the new", buf);
    }

    // ---- 6. FOV follows speed and supersonic
    {
        RLCamera slow, fast, sonic;
        SimSnapshot sSlow = makeSnap(V3(0, 0, 17), level, V3(3000, 0, 300));
        SimSnapshot sFast = makeSnap(V3(0, 0, 17), level, V3(3000, 0, 300), V3(1200, 0, 0));
        SimSnapshot sSonic = makeSnap(V3(0, 0, 17), level, V3(3000, 0, 300),
                                      V3(2300, 0, 0), true, true);
        slow.reset(sSlow, cs); fast.reset(sFast, cs); sonic.reset(sSonic, cs);
        step(slow, cs, sSlow, 180);
        step(fast, cs, sFast, 180);
        step(sonic, cs, sSonic, 180);
        char buf[160];
        std::snprintf(buf, sizeof buf, "fov slow %.2f / fast %.2f / sonic %.2f",
                      slow.viewFov(), fast.viewFov(), sonic.viewFov());
        check(slow.viewFov() > cs.fov - 0.1f && fast.viewFov() > slow.viewFov() + 1.0f &&
                  sonic.viewFov() > fast.viewFov() + 3.0f,
              "FOV widens with speed and pops at supersonic", buf);
    }

    // ---- 7. swivel: offsets the view, then recenters on release
    {
        RLCamera cam;
        SimSnapshot s = makeSnap(V3(0, 0, 17), level, V3(4000, 0, 100));
        cam.reset(s, cs);
        cam.setMode(CamMode::Car);
        step(cam, cs, s, 120);
        const V3 parked = dirOf(cam);
        const float parkedYaw = horizYaw(parked);
        step(cam, cs, s, 45, 1.0f, 0.0f);            // swivel right
        const float swYaw = horizYaw(dirOf(cam));
        step(cam, cs, s, 15, 0.0f, 0.0f);            // release 0.25 s
        const float releasedYaw = horizYaw(dirOf(cam));
        step(cam, cs, s, 90, 0.0f, 0.0f);            // settle
        const float backYaw = horizYaw(dirOf(cam));
        char buf[160];
        std::snprintf(buf, sizeof buf,
                      "yaw parked %.1f -> swivel %.1f -> release %.1f -> %.1f",
                      deg(parkedYaw), deg(swYaw), deg(releasedYaw), deg(backYaw));
        // +X swivel must turn the view toward screen right (increasing yaw),
        // then snap back to the parked yaw after release.
        check(deg(wrap(swYaw - parkedYaw)) > 20.0f &&
                  std::fabs(deg(wrap(backYaw - parkedYaw))) < 3.0f,
              "swivel moves the view right and recenters after release", buf);

        // Invert Swivel (the turball default) makes a positive Y input tilt
        // the view down; turning it off flips that.
        RLCamera down;
        down.reset(s, cs);
        down.setMode(CamMode::Car);
        step(down, cs, s, 120);
        const float basePitch = std::asin(clampf(dirOf(down).z, -1, 1));
        step(down, cs, s, 45, 0.0f, 1.0f);           // positive Y
        const float downPitch = std::asin(clampf(dirOf(down).z, -1, 1));

        CameraSettings plain = cs;
        plain.invertSwivel = false;
        RLCamera up;
        up.reset(s, plain);
        up.setMode(CamMode::Car);
        step(up, plain, s, 120);
        step(up, plain, s, 45, 0.0f, 1.0f);          // positive Y
        const float upPitch = std::asin(clampf(dirOf(up).z, -1, 1));

        char buf2[160];
        std::snprintf(buf2, sizeof buf2,
                      "invert pitch %.2f -> %.2f, plain pitch %.2f, base %.2f",
                      deg(basePitch), deg(downPitch), deg(upPitch), deg(basePitch));
        check(deg(downPitch) < deg(basePitch) - 5.0f &&
                  deg(upPitch) > deg(basePitch) + 5.0f,
              "invertSwivel decides the vertical swivel direction", buf2);

        // Snap Camera to Default off: releasing holds the swivel offset.
        CameraSettings hold = cs;
        hold.snap = false;
        RLCamera held;
        held.reset(s, hold);
        held.setMode(CamMode::Car);
        step(held, hold, s, 120);
        step(held, hold, s, 45, 1.0f, 0.0f);
        const float heldYaw = horizYaw(dirOf(held));
        step(held, hold, s, 90, 0.0f, 0.0f);
        const float afterYaw = horizYaw(dirOf(held));
        char buf3[128];
        std::snprintf(buf3, sizeof buf3, "held %.1f -> %.1f deg",
                      deg(heldYaw), deg(afterYaw));
        check(std::fabs(deg(wrap(afterYaw - heldYaw))) < 3.0f,
              "snap off holds the swivel offset after release", buf3);
    }

    // ---- 8. ground roll banks the camera; airborne bank decays
    {
        const Frame rolled = frameOf(0, 0, 30.0f * kPi / 180.0f);
        RLCamera cam;
        SimSnapshot s = makeSnap(V3(0, 0, 17), rolled, V3(4000, 0, 100));
        cam.reset(s, cs);
        cam.setMode(CamMode::Car);
        step(cam, cs, s, 180);
        const float bank = deg(std::acos(clampf(cam.up.z, -1.0f, 1.0f)));
        SimSnapshot air = s;
        air.carPos = V3(0, 0, 800);
        air.onGround = false;
        air.carVel = V3(0, 0, 0);
        step(cam, cs, air, 180);
        const float airBank = deg(std::acos(clampf(cam.up.z, -1.0f, 1.0f)));
        char buf[128];
        std::snprintf(buf, sizeof buf, "ground bank %.2f deg -> air %.2f deg", bank, airBank);
        check(bank > 1.5f && bank < 6.0f && airBank < 0.5f,
              "ground roll banks ~10% of the car's roll, then decays", buf);
    }

    // ---- 9. long chaotic trace: finite poses, sane distance to the car
    {
        RLCamera cam;
        SimSnapshot s = makeSnap(V3(0, 0, 17), level, V3(2000, 0, 300));
        cam.reset(s, cs);
        float maxDist = 0, minDist = 1e9f;
        bool ok = true;
        for (int i = 0; i < 4000; i++) {
            const float t = i * kDt;
            const float yaw = 3.0f * std::sin(t * 0.7f);
            const float pitch = 0.9f * std::sin(t * 1.3f);
            const float roll = 2.0f * std::sin(t * 2.1f);
            const Frame fr = frameOf(yaw, pitch, roll);
            s.carF = fr.f; s.carR = fr.r; s.carU = fr.u;
            s.carPos = V3(1500.0f * std::sin(t * 0.5f),
                          900.0f * std::cos(t * 0.31f),
                          17.0f + 700.0f * std::fabs(std::sin(t * 0.8f)));
            s.carVel = V3(800 * std::cos(t * 0.5f), -400 * std::sin(t * 0.31f),
                          700 * std::cos(t * 0.8f));
            s.ballPos = V3(1200.0f * std::sin(t * 0.23f + 0.7f),
                           2400.0f * std::cos(t * 0.37f),
                           100.0f + 900.0f * std::fabs(std::sin(t * 0.55f)));
            s.onGround = (i / 200) % 3 == 0;
            s.supersonic = (i / 300) % 4 == 0;
            if (i == 2000) cam.setMode(CamMode::Ball);
            if (i == 3000) cam.setMode(CamMode::Car);
            cam.update(kDt, cs, s, std::sin(t * 0.9f), std::cos(t * 1.7f));
            if (!finite(cam.eye) || !finite(cam.target) || !finite(cam.up)) ok = false;
            const float dist = (cam.eye - s.carPos).len();
            if (!std::isfinite(dist)) ok = false;
            maxDist = std::max(maxDist, dist);
            minDist = std::min(minDist, dist);
            if (dist > 4000.0f || dist < 20.0f) ok = false;
        }
        char buf[160];
        std::snprintf(buf, sizeof buf, "eye-car distance %.0f..%.0f uu over 4000 frames",
                      minDist, maxDist);
        check(ok, "chaotic 4000-frame trace stays finite and sane", buf);
    }

    std::printf("\n%s\n", failed ? "CAMERA MODEL REGRESSION" : "all camera model checks ok");
    return failed ? 1 : 0;
}
