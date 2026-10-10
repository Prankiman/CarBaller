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
    void computeBallCam(const CameraSettings& cs, const SimSnapshot& s, float dt,
                        V3& outEye, V3& outTarget);
    void computeCarCam(const CameraSettings& cs, const SimSnapshot& s, float dt, V3& outEye, V3& outTarget);
    // Shared eye placement for both rigs: swings the rig around the car on a
    // sphere (yaw AND pitch swivel orbit; see the note in the .cpp) and hands
    // back the part of the pitch swivel the orbit could not absorb - the
    // caller adds that to the view pitch instead of pitchOff_.
    void placeEye(const CameraSettings& cs, const SimSnapshot& s,
                  float lookYaw, V3& outEye, float& outTilt) const;

    CamMode desired_ = CamMode::Ball;
    float blend_ = 1.0f;        // 0 = car cam, 1 = ball cam

    // Ball-cam aim: the rig's own yaw/pitch, before the swivel offsets are
    // added. These are slewed (see kBallAimSlew) instead of read straight
    // off the snapshot, because the measured car->ball bearing is only well
    // conditioned while the ball is clear of the car's vertical axis - see
    // the long note above computeBallCam.
    float ballYaw_ = 0;         // eased bearing to the ball (rad, unwrapped)
    float ballPitch_ = 0;       // eased elevation to the ball (rad)
    float measYaw_ = 0;         // same bearing, unwrapped (never slewed)
    float prevPhi_ = 0;         // last reliable raw bearing
    bool hasBallAim_ = false;   // aim state anchored to a snapshot
    bool hasPrevPhi_ = false;   // prevPhi_ holds a real measurement

    // Swivel offsets (rad): both swing the rig AROUND the car (see
    // placeEye). Only the part of the pitch offset the orbit cannot absorb
    // reaches the view as a plain tilt.
    float yawOff_ = 0;
    float pitchOff_ = 0;
    float swivelIdle_ = 0;       // seconds since the last swivel input

    // Car-cam heading: the axis the camera sits behind. On the ground it is
    // the car's facing, tracked rigidly (RL keeps the camera straight as
    // long as the car is on a surface). In the air the target switches to
    // the direction of TRAVEL - the horizontal velocity heading, eased onto
    // at the stiffness-scaled rate in the .cpp, with the facing taking over
    // again as the speed runs out - so a flip swings the camera behind the
    // new momentum over ~0.5s while pitching and air rolling move it not at
    // all, and there is no decision window to wait out.
    float carYaw_ = 0;
    // Facing axis read off the frame, kept continuous so a pitch past
    // vertical (which flips the nose bearing by 180 deg without the car
    // turning) cannot swing the camera - see kAxisHalfTurn in the .cpp.
    // Nose and rear are the same line 180 deg apart, so this axis is
    // equally "behind the rear" whichever end it is named for. That
    // continuity rule only governs the AIR: on the ground the axis is
    // re-read from the nose every frame, so it can never stick to the
    // wrong side and always heals at landing.
    float axisYaw_ = 0;
    bool hasAxisYaw_ = false;    // axisYaw_ holds a real bearing
    V3 smoothedEye_{0, 0, 0};
    V3 prevWantEye_{0, 0, 0};    // previous ideal eye, for rig speed
    float prevRigOffZ_ = 0;      // previous rig-relative eye height: splits the
                                 // swivel's own vertical motion off from the
                                 // car's (the latter keeps the stiffness lag)
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
