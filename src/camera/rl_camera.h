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
    void reset(const SimSnapshot& s);

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

    float carYaw_ = 0;          // smoothed horizontal follow (car cam)
    V3 smoothedEye_{0, 0, 0};
    bool hasSmoothed_ = false;

    float shakeAmp_ = 0;
    float shakeTime_ = 0;
};
