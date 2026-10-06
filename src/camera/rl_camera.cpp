#include "rl_camera.h"

#include <cmath>

static V3 dirFromYawPitch(float yaw, float pitch) {
    float cp = std::cos(pitch);
    return {cp * std::cos(yaw), cp * std::sin(yaw), std::sin(pitch)};
}

void RLCamera::reset(const SimSnapshot& s, const CameraSettings& cs) {
    // Intentionally does NOT touch desired_: the player's ball-cam choice
    // survives respawns/position presets (RL never flips the mode for you),
    // and before the first input this starts in ball cam like RL does.
    yawOff_ = pitchOff_ = 0;
    swivelIdle_ = 0;
    carYaw_ = std::atan2(s.carF.y, s.carF.x);
    lockYaw_ = carYaw_;
    airTime_ = 0;
    locked_ = false;
    blend_ = (desired_ == CamMode::Ball) ? 1.0f : 0.0f;  // snap, no animation
    hasSmoothed_ = false;
    shakeAmp_ = 0;

    V3 e, t;
    if (blend_ > 0.5f)
        computeBallCam(cs, s, e, t);
    else
        computeCarCam(cs, s, 0.0f, e, t);
    smoothedEye_ = e;
    prevWantEye_ = e;
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

// After leaving a surface the camera first HOLDS the takeoff heading for
// kAirDelay - no nose-following at all yet, so a jump or flip straight off
// the ground cannot swing it. This is the undecided phase: the camera
// keeps the world heading it left the surface with while the car may
// already be rotating, so its back/front position relative to the car is
// deliberately still open. It then follows the nose for kAirFollowTime
// (the player's initial pitch/steer still moves the camera point), and the
// rear/front axis at the end of that window is locked in for the rest of
// the flight: 1.4s hold + 0.6s live follow = 2.0s total, with all of the
// extra length in the hold so it reads as a longer initial wait rather than
// a slower settle - the pre-change config reached 1.5s the other way round
// (1.0s hold + 0.5s follow, and no closing gain, so it eased onto the axis
// for a while after the window had already closed).
static constexpr float kAirDelay = 1.4f;
static constexpr float kAirFollowTime = 0.6f;
// 1e-5 of slop: 1.4f + 0.6f rounds a hair off 2.0f in binary.
static_assert(kAirDelay + kAirFollowTime <= 2.0f + 1e-5f,
              "air heading must be decided within 2.0s of takeoff");

// Per-second rate the car-cam heading chases its target (nose while
// tracking, lockYaw_ once locked). Grounded tracking and the air follow
// both start here; inside the air follow window the rate additionally gets
// the time-left gain below, so the hand-off into the lock carries no
// velocity at all - the heading has already arrived on the axis it locks.
static constexpr float kYawFollowRate = 12.0f;

// Air-follow gain expressed per second of window left: as the window
// closes, rate = kLockGain / secondsLeft takes over from kYawFollowRate and
// the tracking error decays like ((left / window)^kLockGain) - it reaches
// zero, smoothly and with zero velocity, at the instant the axis is locked.
// So the lock is a STOP, not a swing: after the window the camera's back/
// front position relative to the car is final, and the post-lock easing has
// (at most) a one-frame nose flip-flop left to absorb.
static constexpr float kLockGain = 2.5f;

// Cap (uu) on how far the smoothed eye trails the ideal rig horizontally
// while the rig is moving - the lag asymptotes to this value at high speed.
static constexpr float kRigLagCap = 40.0f;

void RLCamera::computeCarCam(const CameraSettings& cs, const SimSnapshot& s,
                             float dt, V3& outEye, V3& outTarget) {
    // Grounded = the camera sits behind the rear, following the nose
    // heading. Airborne = the heading is held for kAirDelay, then the
    // follow stays live for kAirFollowTime (converging onto the nose axis as
    // the window closes), then the rear/front axis of that moment is latched
    // as the lock target: pitch, roll and yaw afterwards only translate the
    // camera with the car - they never swing it around - until the car
    // grounds again and tracking resumes. The whole decision takes
    // kAirDelay + kAirFollowTime = 2.0s from takeoff, so by 2.0s the
    // back/front position is fixed for the rest of the flight.
    const float fxy = std::sqrt(s.carF.x * s.carF.x + s.carF.y * s.carF.y);

    bool track;
    if (s.onGround) {
        airTime_ = 0;
        locked_ = false;
        track = true;
    } else if (locked_) {
        track = false;
    } else {
        airTime_ += dt;
        if (airTime_ < kAirDelay) {
            // Takeoff delay: frozen on the heading the car left the ground
            // with - the nose cannot move the camera yet.
            track = false;
        } else if (airTime_ >= kAirDelay + kAirFollowTime) {
            // Window over: latch the camera heading from the rear/front
            // axis of this moment (the tracked heading is kept when the nose
            // points straight up/down, where the horizontal projection is no
            // direction). Setting the target - rather than carYaw_ itself -
            // is what keeps the end of the follow window continuous; with
            // the time-left gain below carYaw_ is already sitting on that
            // axis, so nothing visibly moves here.
            lockYaw_ = (fxy > 0.15f) ? std::atan2(s.carF.y, s.carF.x) : carYaw_;
            locked_ = true;
            track = false;
        } else {
            track = true;
        }
    }

    // Nose projected on the ground plane: well-defined on floor, ceiling
    // and walls, degenerate only while the nose points straight up or down,
    // where the heading is simply held instead of snapping on noise. Once
    // locked, the same easing runs against the latched lockYaw_ so the
    // heading glides onto the locked axis (and stops) instead of jumping.
    const bool haveTarget = locked_ || (track && fxy > 0.15f);
    if (haveTarget) {
        const float targetYaw = locked_ ? lockYaw_ : std::atan2(s.carF.y, s.carF.x);
        const float delta = wrapAngle(targetYaw - carYaw_);
        float rate = kYawFollowRate;
        if (!locked_ && !s.onGround) {
            // Closing gain: the chase starts at the plain follow rate and
            // only picks up as the seconds left in the window run out, so
            // the heading lands on the nose axis - with no lag left - at the
            // exact frame the axis is latched.
            const float left = std::max(kAirDelay + kAirFollowTime - airTime_, 1e-3f);
            rate = std::max(rate, kLockGain / left);
        }
        carYaw_ += delta * std::min(rate * dt, 1.0f);
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
    //
    // Everything here is a per-SECOND rate; the per-frame factor is always
    // 1 - exp(-rate * dt), which stays in (0,1) for any speed. Adding to the
    // factor instead of the rate lets it exceed 1 and the eye diverges.
    const float followRate = 1.5f + cs.stiffness * 14.0f;
    const float rateZ = s.onGround ? followRate : followRate * 0.5f;

    // A plain exponential follower settles v/rate behind a moving rig - at
    // boost speed (2300 uu/s, default stiffness) that was a whole camera
    // distance, so the eye sat far from the car while boosting and only
    // crept back after the car slowed. The rig-speed term in the horizontal
    // rate makes that lag saturate at kRigLagCap uu instead: rigid at speed
    // like RL, while rest and low-speed recentering keep the original
    // stiffness response. Vertical follow is untouched - the jump/aerial
    // linger is the part RL's stiffness actually does.
    const float dx = wantEye.x - prevWantEye_.x;
    const float dy = wantEye.y - prevWantEye_.y;
    const float rigSpeed2d = std::sqrt(dx * dx + dy * dy) / std::max(dt, 1e-4f);
    const float rateX = followRate + rigSpeed2d / kRigLagCap;
    prevWantEye_ = wantEye;

    const float kx = 1.0f - std::exp(-rateX * dt);
    const float kz = 1.0f - std::exp(-rateZ * dt);
    if (!hasSmoothed_) {
        smoothedEye_ = wantEye;
        hasSmoothed_ = true;
    } else {
        smoothedEye_.x += (wantEye.x - smoothedEye_.x) * kx;
        smoothedEye_.y += (wantEye.y - smoothedEye_.y) * kx;
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
