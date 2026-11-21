#include "rl_camera.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;
// turball stores bearings/pitch/roll in 16-bit angle units (65536 per turn).
constexpr float kUnitsPerRad = 65536.0f / kTwoPi;

float deg2rad(float d) { return d * kPi / 180.0f; }

V3 dirFromYawPitch(float yaw, float pitch) {
    const float cp = std::cos(pitch);
    return {cp * std::cos(yaw), cp * std::sin(yaw), std::sin(pitch)};
}

// The pitch with which turball seeds a fresh car cam: the car's own nose
// elevation plus Camera Angle. Bearing comes from the car's facing.
float carPitch(const SimSnapshot& s) { return std::asin(clampf(s.carF.z, -1, 1)); }

// turball clamps the eye to y = 10 (its world floor). CarBaller's custom maps
// put their floor anywhere, so lift the clamp to the loaded map's floor.
//
// The tracked floor is the last surface the car stood on, which on a rings map
// can be a checkpoint platform far above the course (and survives a session
// reset). Clamping the eye to it after the car has dropped below would pin the
// camera to a platform it left behind, so below that surface the clamp is
// abandoned and the camera follows the car down.
float eyeFloor(const SimSnapshot& s) {
    constexpr float kUnderFloorSlack = 20.0f;
    if (s.hasMapFloor && s.carPos.z < s.floorZ - kUnderFloorSlack)
        return -std::numeric_limits<float>::infinity();
    return (s.hasMapFloor ? s.floorZ : 0.0f) + 10.0f;
}

// Rotate world up around the view direction by `roll`, matching turball's
// hand-built camera.up (Rodrigues around the view axis).
V3 rolledUp(const V3& dir, float roll) {
    if (std::fabs(roll) < 1e-8f) return {0, 0, 1};
    const float s = std::sin(roll), c = std::cos(roll), k = 1.0f - c;
    return {
        dir.y * s + dir.x * dir.z * k,
        -dir.x * s + dir.y * dir.z * k,
        c + dir.z * dir.z * k,
    };
}

}  // namespace

// ---------------------------------------------------------------------------
// Core: turball's Dt, translated to CarBaller's Z-up frame. It owns the pivot
// spring, follow height, bearing/pitch smoothing, ground stability and roll,
// and ball-cam/air-cam aiming. Swivel and mode blending live in RLCamera.
// ---------------------------------------------------------------------------
struct RLCamera::Core {
    struct Pose {
        V3 pivot;
        float pitch = 0, yaw = 0, roll = 0;
        float distance = 0;
        float fov = 110.0f;
    };

    explicit Core(bool ballCam) : ballCam_(ballCam) {}

    bool ballCam_;
    bool initialized_ = false;

    V3 heading_{1, 0, 0};
    V3 origin_{0, 0, 0};
    V3 previousDirection_{1, 0, 0};
    V3 previousCameraPosition_{0, 0, 0};
    V3 baseTarget_{0, 0, 0};
    V3 followHeight_{0, 0, 0};
    V3 targetHeight_{0, 0, 0};
    V3 pivot_{0, 0, 0};
    V3 previousPivot_{0, 0, 0};
    V3 baseDirection_{1, 0, 0};
    V3 previousCarPosition_{0, 0, 0};

    float groundStability_ = 1.0f;
    float rollUnits_ = 0.0f;      // 16-bit angle units, like turball
    float bearing_ = 0.0f;        // view yaw (radians)
    float pitch_ = 0.0f;          // view pitch (radians, +=up)
    float viewDistance_ = 0.0f;
    float viewFov_ = 110.0f;
    float posePitch_ = 0.0f;
    float poseYaw_ = 0.0f;
    float poseRoll_ = 0.0f;
    bool previousTracking_ = false;

    Pose pose() const {
        return {pivot_, posePitch_, poseYaw_, poseRoll_, viewDistance_, viewFov_};
    }

    void update(const CameraSettings& cs, const SimSnapshot& s, float dt) {
        const bool u = ballCam_;
        const bool first = !initialized_;
        // Set when this core was in ball cam last frame and is now car cam.
        const bool justLeftBallCam = !first && previousTracking_ && !u;

        const float m = clampf(cs.height, 20.0f, 450.0f);
        const float speed = s.carVel.len();
        const float horizSpeed = s.carVel.len2d();

        // ---- ground stability: 1 on the ground, 0 in the air, eased.
        if (first || justLeftBallCam) groundStability_ = s.onGround ? 1.0f : 0.0f;
        groundStability_ = clampf(
            groundStability_ + (s.onGround ? 2.0f : -4.0f) * dt, 0.0f, 1.0f);

        // ---- follow height: world-up in ball cam, car-up on the ground in
        // car cam, smoothly rotating between the two while airborne.
        if (!u && s.onGround && groundStability_ >= 1.0f)
            targetHeight_ = s.carU * m;
        else
            targetHeight_ = V3(0, 0, m);

        if (u) {
            followHeight_ = V3(0, 0, m);
        } else if (first || justLeftBallCam) {
            followHeight_ = targetHeight_;
        } else {
            const float w = 1.0f - std::exp(-2.03f * dt);
            followHeight_ += (targetHeight_ - followHeight_) * w;
        }

        baseTarget_ = s.carPos + followHeight_;

        // ---- origin: the base target is allowed to lag, but tightly in ball
        // cam (0.001 uu) and up to 100 uu in car cam.
        const float carStep = (previousCarPosition_ - s.carPos).len();
        if (first || carStep > 1000.0f) {
            origin_ = baseTarget_;
        } else {
            const float w = 1.0f - std::exp(-6.32f * dt);
            origin_ += (baseTarget_ - origin_) * w;
            const V3 lag = origin_ - baseTarget_;
            const float len = lag.len();
            const float cap = u ? 0.001f : 100.0f;
            if (len > cap) origin_ = baseTarget_ + lag * (cap / len);
        }

        // ---- pivot spring.
        if (first) {
            // The reference's reset branch collapses to the base target (the
            // along-view projection cancels out), so anchor it directly.
            pivot_ = baseTarget_;
        } else {
            const V3 w = previousCameraPosition_;
            const V3 b = previousDirection_;
            const V3 k = baseTarget_ - w;     // airTarget
            const V3 a = origin_ - w;         // originLag from the camera
            const float J = k.dot(b);
            const float I = a.dot(b);
            const V3 d = (a - b * I) + b * J;
            const V3 it = w + d;
            const float stiff = clampf(cs.stiffness, 0.0f, 1.0f);
            pivot_ = it + (baseTarget_ - it) * stiff;
        }

        // ---- heading: horizontal line from pivot to ball in ball cam, car
        // nose in car cam. Ball cam uses this only for the bearing.
        float ballHoriz = 0.0f;
        if (u) {
            heading_ = s.ballPos - pivot_;
            ballHoriz = heading_.len2d();
            heading_.z = 0.0f;   // bearing only: the check below sees xz
        } else {
            heading_ = s.carF;
        }
        if (heading_.dot(heading_) < 1.0f) heading_ = s.carF;  // ball overhead
        heading_.z = 0.0f;
        if (heading_.dot(heading_) < 1e-6f) heading_ = V3(1, 0, 0);
        heading_ = heading_.norm();

        const float angle = deg2rad(cs.angle);
        float E = clampf(cs.distance, 80.0f, 900.0f);
        float q = angle;
        float ballRise = 0.0f;

        // ---- ball cam pitches toward an elevated ball beyond a small knee,
        // and raises the pivot with it.
        if (u) {
            const float S = std::max(ballHoriz, 1e-3f);
            const float w = std::atan2(s.ballPos.z - pivot_.z, S);
            ballRise = std::fabs(w) * (kUnitsPerRad * 0.005f);
            const float b = std::atan2(s.ballPos.z - pivot_.z - ballRise, S);
            const float knee = 4000.0f / kUnitsPerRad;   // ~21.97 deg
            const float k = 0.8f * clampf((std::fabs(b) - knee) / knee, 0.0f, 1.0f);
            q = angle + (b - angle) * k;
        }

        const float st = std::atan2(heading_.y, heading_.x);

        if (first) {
            // Anchor the view, don't ease in from a stale pose.
            bearing_ = st;
            pitch_ = q;
            if (!u && !s.onGround) {
                pitch_ = angle;
                if (speed > 0.001f && horizSpeed > 1e-4f) {
                    const float velBearing = std::atan2(s.carVel.y, s.carVel.x);
                    const float blend = clampf(horizSpeed / 500.0f, 0.0f, 1.0f);
                    bearing_ = st + wrapAngle(velBearing - st) * blend;
                }
            }
        } else {
            const float stiff = clampf(cs.stiffness, 0.0f, 1.0f);
            const float k = u ? 1.0f - std::exp(-8.59f * dt)
                              : (stiff == 1.0f
                                     ? 1.0f
                                     : 1.0f - std::exp(-(13.39f + 32.4975f * stiff) * dt));

            if (u) {
                bearing_ += wrapAngle(st - bearing_) * k;
                pitch_ += (q - pitch_) * k;
            } else {
                const float newBearing = bearing_ + wrapAngle(st - bearing_) * k;
                const float newPitch = pitch_ + (q - pitch_) * k;

                // Aim the car cam at the pivot from where it was, blended in
                // by speed: the faster you fly, the more the camera tracks
                // along the travel direction instead of the nose.
                V3 at = pivot_ - previousCameraPosition_;
                // Drop the pivot's vertical motion from the aim. Taking the
                // frame's full translation feeds a climb/dive back into the
                // pitch every frame, tilting the view further and further
                // toward straight down/up; near vertical the aim's horizontal
                // component vanishes and the bearing whips around. Horizontal
                // travel still swings the camera behind the car.
                at.z -= pivot_.z - previousPivot_.z;
                const float atHoriz = at.len2d();
                const float Jp = std::atan2(at.z, atHoriz);
                const float Ib = atHoriz > 1e-6f ? std::atan2(at.y, at.x) : bearing_;
                const float B = (7.0f - 5.0f * clampf(speed / 2300.0f, 0.0f, 1.0f)) / 12.0f;
                float O = pitch_ + (Jp - pitch_) * B;
                const float D = bearing_ + wrapAngle(Ib - bearing_) * B;

                // Reverse travel flattens the pitch out (turball).
                if (speed > 0.001f) {
                    const V3 dir = dirFromYawPitch(D, O);
                    if (dir.dot(s.carVel) < 0.0f)
                        O *= 1.0f - clampf(horizSpeed / 2300.0f, 0.0f, 1.0f);
                }

                if (groundStability_ == 1.0f) {
                    bearing_ = newBearing;
                    pitch_ = newPitch;
                } else {
                    bearing_ = D + wrapAngle(newBearing - D) * groundStability_;
                    pitch_ = O + (newPitch - O) * groundStability_;
                }
            }
        }

        previousTracking_ = u;
        pivot_.z += ballRise;

        posePitch_ = pitch_;
        poseYaw_ = bearing_;
        baseDirection_ = dirFromYawPitch(poseYaw_, posePitch_);

        // ---- distance extends with the component of travel along the view.
        if (speed > 0.001f) {
            const float along = s.carVel.dot(baseDirection_);
            E += std::max(-50.0f, along * 0.05f * (1.0f - clampf(cs.stiffness, 0.0f, 1.0f)));
        }
        if (first) {
            viewDistance_ = E;
        } else {
            viewDistance_ += (E - viewDistance_) * (1.0f - std::exp(-4.14f * dt));
        }
        E = viewDistance_;

        previousDirection_ = baseDirection_;
        previousCameraPosition_ = pivot_ - baseDirection_ * E;
        previousCameraPosition_.z =
            std::max(previousCameraPosition_.z, eyeFloor(s));
        previousCarPosition_ = s.carPos;
        previousPivot_ = pivot_;
        initialized_ = true;

        // ---- camera roll: 10% of the car's roll while planted, decaying with
        // speed once airborne (turball's groundStability / rollUnits).
        const float Us = (7.0f - 5.0f * clampf(speed / 2300.0f, 0.0f, 1.0f)) / 12.0f;
        if (u) {
            rollUnits_ = 0.0f;
        } else if (groundStability_ > 0.0f) {
            const float os = std::asin(clampf(-s.carR.z, -1.0f, 1.0f)) * kUnitsPerRad;
            rollUnits_ = std::trunc(std::trunc(os * groundStability_) * 0.1f);
        } else {
            rollUnits_ += std::trunc(-rollUnits_ * Us);
        }
        poseRoll_ = rollUnits_ / kUnitsPerRad;

        // ---- FOV widens with speed; supersonic adds a fixed +10 pop.
        const float targetFov =
            clampf(cs.fov, 60.0f, 145.0f) +
            (s.supersonic ? 10.0f : 5.0f * (speed / 2300.0f));
        if (first) {
            viewFov_ = targetFov;
        } else {
            const float lim = (s.supersonic ? 40.0f : 25.0f) * dt;
            viewFov_ += clampf(targetFov - viewFov_, -lim, lim);
        }
    }
};

RLCamera::RLCamera() = default;
RLCamera::~RLCamera() = default;

void RLCamera::reset(const SimSnapshot& s, const CameraSettings& cs) {
    (void)s;
    (void)cs;
    // Keep the player's chosen mode; start the core fresh so the next update
    // re-anchors exactly like turball's viewEpoch reset.
    activeBallCam_ = desired_ == CamMode::Ball;
    core_ = std::make_unique<Core>(activeBallCam_);
    blendActive_ = false;
    blendRemaining_ = blendDuration_ = 0.0f;
    sharedYaw_ = sharedPitch_ = 0.0f;
    shakeAmp_ = 0.0f;
    shakeTime_ = 0.0f;
}

void RLCamera::update(float dt, const CameraSettings& cs, const SimSnapshot& s,
                      float swivelX, float swivelY) {
    dt = clampf(dt, 0.0f, 0.05f);
    const bool wantBall = desired_ == CamMode::Ball;

    if (!core_) {
        activeBallCam_ = wantBall;
        core_ = std::make_unique<Core>(activeBallCam_);
    }

    bool switched = false;
    if (wantBall != activeBallCam_) {
        // turball advances the old core once, captures its pose, then builds a
        // fresh core in the new mode and blends from the old pose to the new.
        core_->update(cs, s, dt);
        const Core::Pose oldPose = core_->pose();

        core_ = std::make_unique<Core>(wantBall);
        core_->update(cs, s, dt);
        const Core::Pose newPose = core_->pose();

        const float duration = 0.5f / std::max(0.01f, cs.transitionSpeed);
        blendDelta_.pivot = oldPose.pivot - newPose.pivot;
        blendDelta_.pitch = wrapAngle(oldPose.pitch - newPose.pitch);
        blendDelta_.yaw = wrapAngle(oldPose.yaw - newPose.yaw);
        blendDelta_.roll = wrapAngle(oldPose.roll - newPose.roll);
        blendDelta_.distance = oldPose.distance - newPose.distance;
        blendDelta_.fov = oldPose.fov - newPose.fov;
        blendRemaining_ = duration;
        blendDuration_ = duration;
        blendActive_ = true;
        activeBallCam_ = wantBall;
        switched = true;
    } else {
        core_->update(cs, s, dt);
    }

    Core::Pose p = core_->pose();
    if (blendActive_) {
        const float t = blendDuration_ > 1e-6f ? blendRemaining_ / blendDuration_ : 0.0f;
        const float w = t * t;
        p.pivot = p.pivot + blendDelta_.pivot * w;
        p.pitch = wrapAngle(p.pitch + blendDelta_.pitch * w);
        p.yaw = wrapAngle(p.yaw + blendDelta_.yaw * w);
        p.roll = wrapAngle(p.roll + blendDelta_.roll * w);
        p.distance += blendDelta_.distance * w;
        p.fov += blendDelta_.fov * w;
    }

    // ---- shared swivel, applied after the pose (and across mode swaps).
    // CarBaller's world is mirrored relative to turball's (RL is left-handed),
    // so turball's yaw offset subtracts here to keep "look right" = screen
    // right. Vertical follows the Invert Swivel option, which also reproduces
    // turball's inverted-by-default vertical swivel.
    const float swSpeed = clampf(cs.swivelSpeed, 1.0f, 10.0f);
    const bool yawHeld = std::fabs(swivelX) > 1e-3f;
    const bool pitchHeld = std::fabs(swivelY) > 1e-3f;
    float targetYaw = sharedYaw_;
    float targetPitch = sharedPitch_;
    if (yawHeld) targetYaw = -clampf(swivelX, -1.0f, 1.0f) * deg2rad(130.0f);
    else if (cs.snap) targetYaw = 0.0f;
    if (pitchHeld)
        targetPitch = clampf(swivelY, -1.0f, 1.0f) * deg2rad(60.0f) *
                      (cs.invertSwivel ? 1.0f : -1.0f);
    else if (cs.snap)
        targetPitch = 0.0f;
    const float yawStep =
        clampf(dt * swSpeed * (std::fabs(targetYaw) < std::fabs(sharedYaw_) ? 3.0f : 1.0f),
               0.0f, 1.0f);
    const float pitchStep =
        clampf(dt * swSpeed * (std::fabs(targetPitch) < std::fabs(sharedPitch_) ? 3.0f : 1.0f),
               0.0f, 1.0f);
    sharedYaw_ += (targetYaw - sharedYaw_) * yawStep;
    sharedPitch_ += (targetPitch - sharedPitch_) * pitchStep;

    const float finalPitch = wrapAngle(p.pitch - sharedPitch_);
    const float finalYaw = wrapAngle(p.yaw - sharedYaw_);
    const V3 dir = dirFromYawPitch(finalYaw, finalPitch);

    V3 e = p.pivot - dir * p.distance;
    e.z = std::max(e.z, eyeFloor(s));
    target = e + dir;
    up = rolledUp(dir, p.roll);
    viewFovDeg_ = p.fov;

    // ---- shake (kept outside the camera model, like turball's game layer).
    shakeTime_ += dt;
    shakeAmp_ *= std::exp(-5.5f * dt);
    if (shakeAmp_ < 0.001f) shakeAmp_ = 0.0f;
    const V3 shake(std::sin(shakeTime_ * 47.0f) * shakeAmp_,
                   std::sin(shakeTime_ * 39.0f + 1.7f) * shakeAmp_,
                   std::sin(shakeTime_ * 53.0f + 3.1f) * shakeAmp_);
    eye = e + shake * 9.0f;
    target += shake * 3.0f;

    // turball consumes the mode blend AFTER this frame's pose is composed, so
    // the switch frame and the one after both show the old pose.
    if (blendActive_ && !switched) {
        blendRemaining_ = std::max(0.0f, blendRemaining_ - dt);
        if (blendRemaining_ == 0.0f) blendActive_ = false;
    }
}
