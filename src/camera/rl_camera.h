#pragma once
// turball.io-style camera: ball cam (rig aimed at the ball) and car cam
// (behind the car / direction of travel), ported from turball's shipped
// camera core (Dt) and its mode-blending wrapper (vs). Settings mirror
// turball: fov, distance, height, angle, stiffness, swivelSpeed,
// transitionSpeed, invertSwivel.
//
// The camera is a spring-driven pivot: the eye lives `distance` behind the
// pivot along the view direction, and the view direction follows a bearing
// (yaw) + pitch that are smoothed at rates derived from stiffness. Ball cam
// keeps the pivot nearly rigid and pitches toward an elevated ball; car cam
// lets the pivot lag up to 100 uu, follows the car's up-axis on the ground,
// and swings behind the direction of travel in the air.

#include <memory>

#include "../core/math.h"
#include "../settings/settings.h"
#include "../sim/sim.h"

enum class CamMode { Car = 0, Ball = 1 };

class RLCamera {
public:
    RLCamera();
    ~RLCamera();
    RLCamera(const RLCamera&) = delete;
    RLCamera& operator=(const RLCamera&) = delete;

    // Re-anchors the eye at the snapshot. Keeps the current camera mode -
    // repositioning the car must never flip ball cam on (or off) for you.
    void reset(const SimSnapshot& s, const CameraSettings& cs);

    void setMode(CamMode m) { desired_ = m; }
    CamMode mode() const { return activeBallCam_ ? CamMode::Ball : CamMode::Car; }
    CamMode desiredMode() const { return desired_; }
    bool transitioning() const { return blendActive_; }

    // swivelX/Y in [-1,1] (x=right, y=look up)
    void update(float dt, const CameraSettings& cs, const SimSnapshot& s,
                float swivelX, float swivelY);

    void kickShake(float amount) { shakeAmp_ = clampf(shakeAmp_ + amount, 0, 1.2f); }
    float shakeAmount() const { return shakeAmp_; }

    // Horizontal field of view (degrees) for the current frame. turball's
    // camera widens it with speed and pops it at supersonic, so the renderer
    // must use this instead of the raw setting.
    float viewFov() const { return viewFovDeg_; }

    V3 eye, target;
    V3 up{0, 0, 1};   // rolled camera up vector (ground roll / air roll)

private:
    struct Core;
    std::unique_ptr<Core> core_;

    CamMode desired_ = CamMode::Ball;
    bool activeBallCam_ = true;

    // Mode-switch pose blend (turball vs::blend). The twin of the pose is
    // captured when the mode changes and is mixed in with (remaining/duration)^2.
    struct BlendDelta {
        V3 pivot;
        float pitch = 0, yaw = 0, roll = 0, distance = 0, fov = 0;
    };
    bool blendActive_ = false;
    float blendRemaining_ = 0;
    float blendDuration_ = 1;
    BlendDelta blendDelta_;

    // Shared swivel offsets (radians), carried across mode switches like
    // turball: swivel is applied after the pose and is not part of the core.
    float sharedYaw_ = 0;
    float sharedPitch_ = 0;

    float viewFovDeg_ = 110.0f;
    float shakeAmp_ = 0;
    float shakeTime_ = 0;
};
