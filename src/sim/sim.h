#pragma once
// RocketSim wrapper: arena setup (infinite boost freeplay), fixed 120Hz stepping
// with render-rate interpolation, and freeplay actions.

#include "../core/math.h"
#include "../maploader/map_physics.h"
#include "../settings/settings.h"

#include "RocketSim/src/Sim/Arena/Arena.h"
#include "RocketSim/src/Sim/Car/Car.h"
#include "RocketSim/src/Sim/Ball/Ball.h"

#include <cstdint>
#include <functional>
#include <random>
#include <string>

struct SimSnapshot {
    V3 carPos, carVel, carAngVel;
    V3 carF, carR, carU;          // interpolated orientation frame
    V3 ballPos, ballVel, ballAngVel;
    V3 ballF, ballR, ballU;       // interpolated ball orientation (spin)
    float boost = 100;
    bool onGround = true;
    // Custom maps only: the surface the car was last standing on. Loaded maps
    // put that surface anywhere in z, so the camera cannot assume the default
    // arena's floor height when deciding how far its eye may drop.
    bool hasMapFloor = false;
    float floorZ = 0;
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
    // Hit-marker tool: where the car last touched the ball and how far up the
    // ball that contact sat (-1 = south pole, 0 = equator, +1 = north pole).
    V3 ballHitContact;
    float ballHitY = 0;
    float ballHitAge = 999.0f;    // seconds since the contact
};

struct BallHitEvent {
    V3 pos;
    float strength = 0;
    uint64_t tick = 0;
};

struct BallSurfaceHitEvent {
    float strength = 0;      // impact speed into the surface (uu/s)
};

// Built-in "Aim training" mode (after CoCo's workshop map): a bright-green
// square target on a solid panel in front of the backboard. Every attempt
// drops the ball from just below the ceiling at pitch centre; the car starts
// a little ahead of midfield. A target hit extends the streak, a miss
// resets it and re-arms the same target so the shot can be retaken.
struct AimTrainerState {
    bool active = false;
    int streak = 0;
    int best = 0;
    int attempts = 0;          // shots that actually touched the ball
    int hits = 0;
    // 0 = ball dropping / waiting for the first car touch
    // 1 = shot in flight (score can still be decided)
    // 2 = short result pause before the next attempt is set up
    int phase = 0;
    int lastResult = 0;        // 1 = hit, -1 = miss, 0 = none yet
    float lastResultAge = 0;   // seconds since the result
    float resultDelay = 0;     // seconds left in the result pause
    V3 targetCenter;           // on the panel's front face (world space)
    float targetHalf = 0;      // target square half-size (uu)
    V3 boardCenter;            // panel front face centre (world space)
    float boardHalfW = 0;
    float boardHalfH = 0;
    V3 ballSpawn;
};

class Sim {
public:
    // meshDir: folder that contains "soccar/*.cmf"
    // carBody: 0 = Fennec on the Octane hitbox, 1 = Dominus on its own hitbox
    bool init(const std::string& meshDir, int carBody = 0);
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
    // With a custom map loaded the preset is ignored and the current
    // checkpoint's spawn/ball positions are used instead.
    void resetShot(int presetIndex);

    // --- custom maps (cooked UE3 .udk/.upk packages) ---------------------
    // Parses `path` and swaps the arena's static collision for the map's
    // geometry, then puts car and ball on the first checkpoint. Any failure
    // falls back to the default soccar arena and describes itself in *err
    // (the caller turns that into a top-right toast).
    // Convenience: parse `path` and apply it (blocks the calling thread).
    bool loadMap(const std::string& path, std::string* err);
    // Apply an already-parsed package. This is the half that has to run on
    // the simulation thread; parsing itself can happen on a worker thread.
    // On failure the arena is left on the default collision.
    bool applyMap(maploader::MapData&& data, std::string* err);
    void unloadMap();
    bool hasMap() const { return mapLoaded_; }
    // Bumped every time the loaded map changes (load, unload). The renderer
    // keys its upload off this: hasMap() alone can't tell map A from map B,
    // so a presence check would leave the first map on screen after a swap.
    uint64_t mapGen() const { return mapGen_; }
    const maploader::MapData& mapData() const { return map_; }
    int sessionCount() const { return (int)map_.sessions.size(); }
    int sessionIndex() const { return session_; }
    // Previous/next checkpoint (wraps around) - repositions car and ball.
    void cycleSession(int delta);

    void applyControlSettings(const ControlSettings& cs);

    // --- built-in aim trainer -------------------------------------------
    // Starts / stops the target-panel mode. Starting drops any custom map and
    // installs a solid panel in front of the backboard; stopping removes it
    // and returns to the normal freeplay arena.
    void startAimTrainer();
    void stopAimTrainer();
    bool aimActive() const { return aim_.active; }
    const AimTrainerState& aimState() const { return aim_; }
    // Re-arm the current attempt. newTarget=true rolls a fresh target square;
    // false keeps the same one (used for retaking a missed shot).
    void resetAimAttempt(bool newTarget);

    // Called whenever a new aim attempt has been set up (car/ball teleport),
    // so the app can snap the camera to the new spawn.
    std::function<void()> onAimAttemptStart;

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

    // Called when driving into reach of a new checkpoint's spawn, with its
    // index and the map's checkpoint count. Without this the proximity
    // mechanic is invisible: nothing on screen changes until Reset Shot.
    std::function<void(int, int)> onCheckpointReached;

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
    // Driving within reach of a checkpoint's spawn marks it as reached, so
    // Reset Shot can send car and ball back there.
    void updateSessionByProximity();

    // Aim trainer helpers (see AimTrainerState above).
    void beginAimAttempt(bool newTarget);
    void spawnAimTarget();
    void updateAimTrainer(bool carHitThisTick);
    void resolveAimShot(bool hit);
    void addAimPanelCollision();
    void removeAimPanelCollision();

    RocketSim::Arena* arena_ = nullptr;
    RocketSim::Car* car_ = nullptr;

    // Loaded custom map (empty / inactive when playing the default arena).
    maploader::MapData map_;
    maploader::MapCollision* mapCol_ = nullptr;
    bool mapLoaded_ = false;
    uint64_t mapGen_ = 0;            // identity of the loaded map (see mapGen())
    int session_ = 0;                 // checkpoint Reset Shot returns to
    float mapFloorZ_ = 0;            // last surface the car stood on (camera floor)

    RocketSim::CarState prevCar_{}, curCar_{};
    RocketSim::BallState prevBall_{}, curBall_{};

    double accum_ = 0;
    uint64_t lastHitTick_ = ~0ULL;
    float ballHitAge_ = 999.0f;      // seconds since the last car-ball contact
    uint64_t lastSurfaceTick_ = 0;   // thud cooldown (~67 ms)
    float ballR_ = 91.25f;           // soccar ball radius, set in init()

    // Flip reset indicator state (drawn by the renderer, see stepOnce).
    bool flipResetHeld_ = false;
    float flipResetAge_ = 0;         // seconds, capped

    // Built-in aim trainer state.
    AimTrainerState aim_;
    std::mt19937 aimRng_{std::random_device{}()};
    float aimShotTime_ = 0;          // seconds since the shot was armed
    float aimRestTime_ = 0;          // seconds the ball has been nearly still
    btRigidBody* aimPanelRB_ = nullptr;
    btCollisionShape* aimPanelShape_ = nullptr;
};
