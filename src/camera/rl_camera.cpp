#include "rl_camera.h"

#include <cmath>

static V3 dirFromYawPitch(float yaw, float pitch) {
    float cp = std::cos(pitch);
    return {cp * std::cos(yaw), cp * std::sin(yaw), std::sin(pitch)};
}

static float smoothK(float stiffness, float dt, float base, float scale) {
    float k = base + stiffness * scale;
    return 1.0f - std::exp(-k * dt);
}

void RLCamera::reset(const SimSnapshot& s, const CameraSettings& cs) {
    // Intentionally does NOT touch desired_: the player's ball-cam choice
    // survives respawns/position presets (RL never flips the mode for you),
    // and before the first input this starts in ball cam like RL does.
    yawOff_ = pitchOff_ = 0;
    swivelIdle_ = 0;
    carYaw_ = std::atan2(s.carF.y, s.carF.x);
    blend_ = (desired_ == CamMode::Ball) ? 1.0f : 0.0f;  // snap, no animation
    hasSmoothed_ = false;
    shakeAmp_ = 0;

    V3 e, t;
    if (blend_ > 0.5f)
        computeBallCam(cs, s, e, t);
    else
        computeCarCam(cs, s, 0.0f, e, t);
    smoothedEye_ = e;
    hasSmoothed_ = true;
    smoothTarget_ = t;
    prevWantT_ = t;
    derivS_ = 0;
    hasSmoothTarget_ = true;
    eye = e;
    target = t;
}

void RLCamera::computeBallCam(const CameraSettings& cs, const SimSnapshot& s,
                              V3& outEye, V3& outTarget) {
    V3 toBall = s.ballPos - s.carPos;
    float baseYaw;
    if (toBall.len2d() > 1.0f)
        baseYaw = std::atan2(toBall.y, toBall.x);
    else
        baseYaw = std::atan2(s.carF.y, s.carF.x);

    float lookYaw = baseYaw + yawOff_;
    V3 fwd(std::cos(lookYaw), std::sin(lookYaw), 0);

    outEye = s.carPos - fwd * cs.distance + V3(0, 0, cs.height);

    V3 toT = s.ballPos - outEye;
    float basePitch = std::atan2(toT.z, std::max(toT.len2d(), 1.0f));
    float pitch = clampf(basePitch + cs.angle * (float)M_PI / 180.0f + pitchOff_,
                         -1.4f, 1.4f);

    V3 dir = dirFromYawPitch(lookYaw, pitch);
    outTarget = outEye + dir * std::max(toT.len(), 1000.0f);
}

void RLCamera::computeCarCam(const CameraSettings& cs, const SimSnapshot& s,
                             float dt, V3& outEye, V3& outTarget) {
    // The whole air rule, deliberately simple: grounded = the camera sits
    // behind the rear, following the nose heading. Airborne = the heading is
    // held at whatever it was when the car last touched floor/ceiling, so the
    // eye keeps the exact position relative to the car it had at takeoff.
    // Pitch, roll and yaw in flight only translate the camera with the car -
    // they never swing it around - and it re-anchors the instant the car
    // grounds again.
    if (s.onGround) {
        // Nose projected on the ground plane: well-defined on floor, ceiling
        // and walls, degenerate only while driving straight up a wall, where
        // the heading is simply held instead of snapping on noise.
        const float fxy = std::sqrt(s.carF.x * s.carF.x + s.carF.y * s.carF.y);
        if (fxy > 0.15f) {
            const float desiredYaw = std::atan2(s.carF.y, s.carF.x);
            const float delta = wrapAngle(desiredYaw - carYaw_);
            carYaw_ += delta * std::min(12.0f * dt, 1.0f);
        }
    }

    float lookYaw = carYaw_ + yawOff_;
    V3 fwd(std::cos(lookYaw), std::sin(lookYaw), 0);

    outEye = s.carPos - fwd * cs.distance + V3(0, 0, cs.height);

    V3 aim = s.carPos + V3(0, 0, cs.height * 0.25f);
    V3 toT = aim - outEye;
    float basePitch = std::atan2(toT.z, std::max(toT.len2d(), 1.0f));
    float pitch = clampf(basePitch + cs.angle * (float)M_PI / 180.0f + pitchOff_,
                         -1.4f, 1.4f);

    V3 dir = dirFromYawPitch(lookYaw, pitch);
    outTarget = outEye + dir * std::max(toT.len(), 1000.0f);
}

void RLCamera::update(float dt, const CameraSettings& cs, const SimSnapshot& s,
                      float swivelX, float swivelY) {
    // ---- swivel (persistent offsets)
    float rate = cs.swivelSpeed * 0.7f;  // rad/s at full stick (5 -> 3.5 rad/s)
    float invert = cs.invertSwivel ? -1.0f : 1.0f;
    yawOff_ += swivelX * rate * dt;
    pitchOff_ += swivelY * rate * dt * invert;
    if (yawOff_ > (float)M_PI) yawOff_ -= 2 * (float)M_PI;
    if (yawOff_ < -(float)M_PI) yawOff_ += 2 * (float)M_PI;
    pitchOff_ = clampf(pitchOff_, -0.8f, 0.7f);

    // ---- snap camera to default: once the swivel goes idle, ease the manual
    // offsets back to zero at full swivel rate (Rocket League's
    // "Snap Camera to Default"). Tiny inputs count as idle so stick noise
    // can't hold the offset off-center.
    if (std::fabs(swivelX) > 0.004f || std::fabs(swivelY) > 0.004f)
        swivelIdle_ = 0;
    else
        swivelIdle_ += dt;
    if (cs.snap && swivelIdle_ > 0.05f) {
        // RL snaps back quickly right after release: near-immediate start,
        // return rate = 2x swivel speed with a fast floor (6 rad/s).
        const float snapStep = std::fmax(rate * 2.0f, 6.0f) * dt;
        yawOff_ -= clampf(yawOff_, -snapStep, snapStep);
        pitchOff_ -= clampf(pitchOff_, -snapStep, snapStep);
    }

    // ---- mode transition
    float targetBlend = (desired_ == CamMode::Ball) ? 1.0f : 0.0f;
    float tRate = 0.5f + cs.transitionSpeed * 2.5f;
    float step = tRate * dt;
    if (blend_ < targetBlend) blend_ = std::min(targetBlend, blend_ + step);
    else if (blend_ > targetBlend) blend_ = std::max(targetBlend, blend_ - step);

    // ---- compute both rigs
    V3 ballEye, ballTarget, carEye, carTarget;
    computeBallCam(cs, s, ballEye, ballTarget);
    computeCarCam(cs, s, dt, carEye, carTarget);

    V3 wantEye = V3::lerp(carEye, ballEye, blend_);
    V3 wantTarget = V3::lerp(carTarget, ballTarget, blend_);

    // ---- stiffness: position follow smoothing. Rocket League's stiffness
    // shows up as world-space lag: jumps displace the camera and it
    // recenters, and during an aerial the eye lingers near the spot that was
    // behind the car at takeoff instead of climbing rigidly with it. Only
    // the vertical axis gets that treatment - horizontal follow stays tight
    // so fast forward flight doesn't drag the camera far behind the car.
    const float k = smoothK(cs.stiffness, dt, 1.5f, 14.0f);
    const float kz = s.onGround ? k : k * 0.5f;
    if (!hasSmoothed_) {
        smoothedEye_ = wantEye;
        hasSmoothed_ = true;
    } else {
        smoothedEye_.x += (wantEye.x - smoothedEye_.x) * k;
        smoothedEye_.y += (wantEye.y - smoothedEye_.y) * k;
        smoothedEye_.z += (wantEye.z - smoothedEye_.z) * kz;
    }

    // ---- one-euro filter on the look-at point.
    // The ball's resting contact with the car micro-bounces it several times a
    // second; aiming straight at it wobbled the whole view (the perceived
    // "car rumble" while dribbling). Low-passing at a fixed low cutoff would
    // also lag real motion, so the cutoff rises with the target's (smoothed)
    // speed: chatter keeps the cutoff low, swivels/flicks snap open fast.
    if (!hasSmoothTarget_) {
        smoothTarget_ = wantTarget;
        prevWantT_ = wantTarget;
        derivS_ = 0;
        hasSmoothTarget_ = true;
    } else {
        const V3 d = wantTarget - smoothTarget_;
        const float rawDeriv = (wantTarget - prevWantT_).len() / std::max(dt, 1e-4f);
        prevWantT_ = wantTarget;
        derivS_ += (rawDeriv - derivS_) * (1.0f - std::exp(-2.0f * (float)M_PI * 3.0f * dt));
        // Car cam aims from the car itself (never the ball), so track it
        // tightly - the view stays on the rear panel through maneuvers.
        // Ball cam keeps the low floor that kills the resting ball's
        // contact chatter.
        const float fc = 1.2f + 1.8f * (1.0f - blend_) + 0.01f * derivS_;  // Hz
        const float k = 1.0f - std::exp(-2.0f * (float)M_PI * fc * dt);
        smoothTarget_ += d * k;
    }

    // ---- shake
    shakeTime_ += dt;
    shakeAmp_ *= std::exp(-5.5f * dt);
    if (shakeAmp_ < 0.001f) shakeAmp_ = 0;
    V3 shake(
        std::sin(shakeTime_ * 47.0f) * shakeAmp_,
        std::sin(shakeTime_ * 39.0f + 1.7f) * shakeAmp_,
        std::sin(shakeTime_ * 53.0f + 3.1f) * shakeAmp_);

    eye = smoothedEye_ + shake * 9.0f;
    target = smoothTarget_ + shake * 3.0f;
}
