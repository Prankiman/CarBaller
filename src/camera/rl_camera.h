#pragma once
// Rocket League-style camera: ball cam (locked to the ball) and car cam
// (behind the car's rear, lagging during flips), with distance/height/angle/
// stiffness/swivel/transition settings.

#include "../core/math.h"
#include "../settings/settings.h"
#include "../sim/sim.h"

enum class CamMode { Car = 0, Ball = 1 };

class RLCamera {
public:
    // Re-anchors the eye at the snapshot. Keeps the current camera mode -
    // repositioning the car must never flip ball cam on (or off) for you.
    void reset(const SimSnapshot& s, const CameraSettings& cs);

    void setMode(CamMode m) { desired_ = m; }
    CamMode mode() const { return blend_ > 0.5f ? CamMode::Ball : CamMode::Car; }
    CamMode desiredMode() const { return desired_; }
    bool transitioning() const {
        return (desired_ == CamMode::Ball && blend_ < 0.999f) ||
               (desired_ == CamMode::Car && blend_ > 0.001f);
    }

    // swivelX/Y in [-1,1] (x=right, y=look up)
    void update(float dt, const CameraSettings& cs, const SimSnapshot& s,
                float swivelX, float swivelY);

    void kickShake(float amount) { shakeAmp_ = clampf(shakeAmp_ + amount, 0, 1.2f); }
    float shakeAmount() const { return shakeAmp_; }

    V3 eye, target;

private:
    void computeBallCam(const CameraSettings& cs, const SimSnapshot& s, V3& outEye, V3& outTarget);
    void computeCarCam(const CameraSettings& cs, const SimSnapshot& s, float dt, V3& outEye, V3& outTarget);

    CamMode desired_ = CamMode::Ball;
    float blend_ = 1.0f;        // 0 = car cam, 1 = ball cam

    float yawOff_ = 0;          // swivel offsets (rad)
    float pitchOff_ = 0;
    float swivelIdle_ = 0;       // seconds since the last swivel input

    // Car-cam heading: follows the nose while grounded. In the air it is
    // held at the takeoff axis for kAirDelay, follows the nose for
    // kAirFollowTime, then the rear/front axis at that instant is locked in
    // and held until the car touches ground again.
    float carYaw_ = 0;
    float airTime_ = 0;          // seconds since leaving the surface
    bool locked_ = false;        // air heading determined for this flight
    V3 smoothedEye_{0, 0, 0};
    V3 prevWantEye_{0, 0, 0};    // previous ideal eye, for rig speed
    bool hasSmoothed_ = false;

    // One-euro filtered look-at point: the aim follows a resting/bouncing
    // ball's contact micro-chatter only weakly (low cutoff), while real fast
    // motion (swivels, flicks, camera swings) passes through with ~1-3 deg lag.
    V3 smoothTarget_{0, 0, 0};
    V3 prevWantT_{0, 0, 0};
    float derivS_ = 0;          // EMA of target speed (uu/s)
    bool hasSmoothTarget_ = false;

    float shakeAmp_ = 0;
    float shakeTime_ = 0;
};
