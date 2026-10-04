#pragma once
// RocketSim wrapper: arena setup (infinite boost freeplay), fixed 120Hz stepping
// with render-rate interpolation, and freeplay actions.

#include "../core/math.h"
#include "../settings/settings.h"

#include "RocketSim/src/Sim/Arena/Arena.h"
#include "RocketSim/src/Sim/Car/Car.h"
#include "RocketSim/src/Sim/Ball/Ball.h"

#include <cstdint>
#include <functional>
#include <string>

struct SimSnapshot {
    V3 carPos, carVel, carAngVel;
    V3 carF, carR, carU;          // interpolated orientation frame
    V3 ballPos, ballVel, ballAngVel;
    V3 ballF, ballR, ballU;       // interpolated ball orientation (spin)
    float boost = 100;
    bool onGround = true;
    bool boosting = false;
    bool supersonic = false;
    bool jumping = false;
    bool flipping = false;
    bool flipReset = false;       // holding a flip reset taken off the BALL
                                  // (wall/ceiling resets never set it, see stepOnce)
    float flipResetAge = 0;       // seconds since the reset was obtained
    bool ballHitValid = false;
    V3 ballHitPos;
    float ballHitStrength = 0;    // |extraHitVel| (uu/s)
};

struct BallHitEvent {
    V3 pos;
    float strength = 0;
    uint64_t tick = 0;
};

struct BallSurfaceHitEvent {
    float strength = 0;      // impact speed into the surface (uu/s)
};

class Sim {
public:
    // meshDir: folder that contains "soccar/*.cmf"
    bool init(const std::string& meshDir);
    void shutdown();

    // Advance simulation time by dt seconds (accumulator, fixed tick).
    void advance(double dt);

    // Interpolation factor for rendering: accumulator / tickDt in [0,1).
    float accumAlpha() const { return float(accum_ / tickDt); }

    // Interpolated snapshot for rendering (alpha in [0,1)).
    SimSnapshot snapshot(float alpha) const;

    bool paused = false;

    // Freeplay actions
    void launchBall(const FreeplaySettings& fs);
    void startDribble();
    // Put the ball on the floor (or in the air, if we're airborne) directly
    // ahead of the car - freeplay "give me the ball".
    void takePossession();
    // Car to a kickoff/drop preset, ball back to the middle at rest.
    void resetShot(int presetIndex);

    void applyControlSettings(const ControlSettings& cs);

    // Called once per ball hit while stepping
    std::function<void(const BallHitEvent&)> onBallHit;

    // Called when the ball strikes the arena shell (floor/walls/ceiling),
    // with car hits excluded - those have their own feedback path.
    std::function<void(const BallSurfaceHitEvent&)> onBallSurfaceHit;

    // Called the tick a jump starts (ground jump or double jump), and the
    // tick a flip (dodge) starts. Mutually exclusive in RocketSim.
    std::function<void()> onCarJump;
    std::function<void()> onCarFlip;

    // Called the tick the car crosses into supersonic (rising edge only, so
    // the sound plays once per burst rather than every tick at speed).
    std::function<void()> onCarSupersonic;

    float tickDt = 1.0f / 120.0f;
    uint64_t ticksSimulated = 0;

    RocketSim::Arena* arena() const { return arena_; }
    RocketSim::Car* car() const { return car_; }

    V3 spawnPosForPreset(int presetIndex) const;
    float spawnYawForPreset(int presetIndex) const;
    static int presetCount() { return 6; }

private:
    void stepOnce();
    // Fire onBallSurfaceHit when the ball's incoming velocity shows a real
    // impact against one of the arena shell planes.
    void detectSurfaceHit(bool carHitThisTick);

    RocketSim::Arena* arena_ = nullptr;
    RocketSim::Car* car_ = nullptr;

    RocketSim::CarState prevCar_{}, curCar_{};
    RocketSim::BallState prevBall_{}, curBall_{};

    double accum_ = 0;
    uint64_t lastHitTick_ = ~0ULL;
    uint64_t lastSurfaceTick_ = 0;   // thud cooldown (~67 ms)
    float ballR_ = 91.25f;           // soccar ball radius, set in init()

    // Flip reset indicator state (drawn by the renderer, see stepOnce).
    bool flipResetHeld_ = false;
    float flipResetAge_ = 0;         // seconds, capped
};
