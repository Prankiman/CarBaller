#include "sim.h"

#include "../maploader/map_loader.h"

#include "RocketSim/src/RocketSim.h"

#include <algorithm>
#include <cmath>
#include <filesystem>

using namespace RocketSim;

static V3 toV3(const Vec& v) { return {v.x, v.y, v.z}; }
static Vec toVec(const V3& v) { return Vec(v.x, v.y, v.z); }

// Arena shell planes for the ball-thud detector (soccar). The +-y walls
// carry the goal-mouth cutout; posts and crossbar stay solid.
static constexpr float kShellHalfX = (float)RocketSim::RLConst::ARENA_EXTENT_X;  // 4096
static constexpr float kShellHalfY = (float)RocketSim::RLConst::ARENA_EXTENT_Y;  // 5120 (field)
static constexpr float kShellCeilZ = 2044.0f;              // soccar ceiling height
static constexpr float kGoalHalfW = 893.0f;                // mouth is 1786 wide
static constexpr float kGoalHeight = 642.77f;              // mouth height
static constexpr float kShellMargin = 40.0f;               // uu slack for 120 Hz steps
static constexpr float kThudMinImpact = 250.0f;            // uu/s; below = bounce chatter
static constexpr uint64_t kThudCooldownTicks = 8;          // ~67 ms
// Wheel contacts at or below this height are the floor (a resting car sits at
// ~17 uu); anything higher is a ball/wall/ceiling touch, i.e. a flip reset.
static constexpr float kFloorClearanceZ = 60.0f;
// Car origin above the surface it is resting on (see kFloorClearanceZ).
static constexpr float kCarRestZ = 17.0f;
// Age stops accumulating here: the indicator only needs the first flash.
static constexpr float kFlipResetAgeMax = 10.0f;
// Octane wheel bottoms (mount -4.5, r 12.5 front / -2.0, r 15 rear) sit this
// far below the car origin - i.e. further down than the physics box's bottom
// face, which is ~1.4 uu *above* the origin.
static constexpr float kWheelReachBelowOrigin = 17.0f;
// Slack on the ball test below: suspension travel + the ball's curvature at
// the point where the wheels bite.
static constexpr float kBallContactSlack = 12.0f;

// RocketSim tells us *that* the wheels found contact (isOnGround) but never
// what they touched, so ask geometry: for a reset off the ball, the ball has
// to be sitting against the wheels' side of the physics box, within wheel
// reach. That is what keeps a wall or ceiling plant from lighting the
// indicator - the disc is only for ball resets.
static bool ballAtWheels(const CarState& c, const BallState& b, const CarConfig& cfg,
                         float ballR) {
    // Physics box in car-local space: +X forward, +Y right, +Z roof (-Z wheels).
    const V3 off(cfg.hitboxPosOffset.x, cfg.hitboxPosOffset.y, cfg.hitboxPosOffset.z);
    const V3 half(cfg.hitboxSize.x * 0.5f, cfg.hitboxSize.y * 0.5f, cfg.hitboxSize.z * 0.5f);

    // Ball centre in that frame, measured from the box centre.
    const V3 d = toV3(b.pos) - toV3(c.pos);
    const V3 f = toV3(c.rotMat.forward), r = toV3(c.rotMat.right), u = toV3(c.rotMat.up);
    const V3 q = V3(d.dot(f), d.dot(r), d.dot(u)) - off;

    // Vector from the closest point on the box to the ball centre.
    const V3 v = V3(q.x - clampf(q.x, -half.x, half.x),
                    q.y - clampf(q.y, -half.y, half.y),
                    q.z - clampf(q.z, -half.z, half.z));
    const float dist = v.len();

    // A wheel planted on the ball leaves the ball's centre ~110 uu from the
    // box (wheel reach + the ball radius); anything further was no reset.
    const float reach = kWheelReachBelowOrigin + (off.z - half.z);
    if (dist > ballR + reach + kBallContactSlack) return false;
    // ...and the ball has to be on the side the wheels point at, so a wall or
    // ceiling plant with the ball alongside - or on the roof - never counts.
    return v.z <= -0.3f * dist;
}

bool Sim::init(const std::string& meshDir) {
    if (!std::filesystem::exists(std::filesystem::path(meshDir) / "soccar")) {
        std::fprintf(stderr, "[sim] collision meshes not found at %s\n", meshDir.c_str());
        return false;
    }
    RocketSim::Init(meshDir, /*silent=*/false);

    ArenaConfig cfg;
    cfg.noBallRot = false;              // keep ball spin (default is true!)
    cfg.useCustomBoostPads = true;      // -> empty list = no boost pads (freeplay)
    cfg.customBoostPads = {};
    // RocketSim's broadphase is tuned for the soccar arena's AABB sizes and
    // extent; custom maps are orders of magnitude bigger, so use Bullet's DBVT.
    cfg.useCustomBroadphase = false;

    arena_ = Arena::Create(GameMode::SOCCAR, cfg, 120.0f);
    if (!arena_) {
        std::fprintf(stderr, "[sim] Arena::Create failed\n");
        return false;
    }

    // Infinite boost freeplay: write mutators directly (SetMutatorConfig has a
    // carMass bug in Arena.cpp — avoid it entirely).
    arena_->_mutatorConfig.boostUsedPerSecond = 0;
    arena_->_mutatorConfig.carSpawnBoostAmount = 100;
    arena_->_mutatorConfig.demoMode = DemoMode::DISABLED;

    ballR_ = arena_->ball->GetRadius();

    car_ = arena_->AddCar(Team::BLUE, CAR_CONFIG_OCTANE);
    if (!car_) {
        std::fprintf(stderr, "[sim] AddCar failed\n");
        return false;
    }
    car_->config.dodgeDeadzone = 0.7f;

    resetShot(0);

    prevCar_ = curCar_ = car_->GetState();
    prevBall_ = curBall_ = arena_->ball->GetState();
    return true;
}

void Sim::shutdown() {
    // Map collision owns Bullet objects the arena references: hand them back
    // (and restore the arena's own collision) before the arena goes away.
    releaseMapCollision(arena_, mapCol_);
    mapCol_ = nullptr;
    mapLoaded_ = false;
    mapGen_++;
    map_ = maploader::MapData();
    delete arena_;
    arena_ = nullptr;
    car_ = nullptr;
}

void Sim::stepOnce() {
    prevCar_ = curCar_;
    prevBall_ = curBall_;

    arena_->Step(1);
    ticksSimulated++;

    curCar_ = car_->GetState();
    curBall_ = arena_->ball->GetState();

    // Remember the ground under the car while it is standing on something: on
    // a loaded map that plane can be a kilometre below the default arena's,
    // and it is what the camera uses to keep its eye out of the floor.
    if (mapLoaded_ && curCar_.isOnGround) mapFloorZ_ = curCar_.pos.z - kCarRestZ;

    // boost never drains (also guarantees full meter even if mutator is bypassed)
    if (curCar_.boost < 100) {
        curCar_.boost = 100;
        RocketSim::CarState s = car_->GetState();
        s.boost = 100;
        car_->SetState(s);
    }

    // Custom maps: walking into a checkpoint's spawn point marks it reached.
    if (mapLoaded_) updateSessionByProximity();

    // Jump/flip sound triggers: rising edges of RocketSim's jump state.
    if (onCarJump) {
        const bool jumpEdge = (curCar_.isJumping && !prevCar_.isJumping) ||
                              (curCar_.hasDoubleJumped && !prevCar_.hasDoubleJumped);
        if (jumpEdge) onCarJump();
    }
    if (onCarFlip && curCar_.hasFlipped && !prevCar_.hasFlipped) onCarFlip();
    if (onCarSupersonic && curCar_.isSupersonic && !prevCar_.isSupersonic)
        onCarSupersonic();

    // Flip reset (Rocket League's "Flip Reset Indicator" state).
    // RocketSim clears hasJumped/hasDoubleJumped/hasFlipped the tick >=3 wheels
    // find contact, so a car that planted its wheels mid-air genuinely gets its
    // flip back - we only have to notice it. Require that the car had actually
    // jumped (a car that just drove off a ledge never lost its flip), that the
    // contact is well above the floor, and - per the indicator's contract -
    // that the wheels found the BALL: wall and ceiling resets grant the flip
    // all the same, but they never light the disc.
    const bool flipResetObtained =
        curCar_.isOnGround && !prevCar_.isOnGround && prevCar_.hasJumped &&
        curCar_.pos.z > kFloorClearanceZ &&
        ballAtWheels(curCar_, curBall_, car_->config, ballR_);
    if (flipResetObtained) {
        flipResetHeld_ = true;
        flipResetAge_ = 0;
    } else if (flipResetHeld_) {
        const bool backOnFloor = curCar_.isOnGround && curCar_.pos.z <= kFloorClearanceZ;
        const bool flipUsed = curCar_.hasFlipped || curCar_.hasDoubleJumped;
        if (backOnFloor || flipUsed) {
            flipResetHeld_ = false;
        } else {
            flipResetAge_ = std::min(flipResetAge_ + tickDt, kFlipResetAgeMax);
        }
    }

    const BallHitInfo& hi = curCar_.ballHitInfo;
    const bool freshCarHit = hi.isValid && hi.tickCountWhenHit != lastHitTick_;

    // Ball vs arena shell (floor/walls/ceiling). Runs first so the fresh
    // car-hit flag can suppress it: car impacts carry their own fx.
    detectSurfaceHit(freshCarHit);

    if (freshCarHit) {
        lastHitTick_ = hi.tickCountWhenHit;
        if (onBallHit) {
            BallHitEvent ev;
            ev.pos = toV3(hi.ballPos);
            ev.strength = hi.extraHitVel.Length();
            ev.tick = hi.tickCountWhenHit;
            onBallHit(ev);
        }
    }
}

void Sim::detectSurfaceHit(bool carHitThisTick) {
    if (carHitThisTick || !arena_ || !arena_->ball) return;
    if (ticksSimulated < lastSurfaceTick_ + kThudCooldownTicks) return;

    const Vec& p = curBall_.pos;
    const Vec& v = prevBall_.vel;   // velocity entering this tick
    const float R = ballR_;

    // An impact counts when the ball sits against a plane (within the
    // margin) AND was moving into it faster than bounce chatter.
    float impact = 0;
    if (p.z - R < kShellMargin)                 impact = std::max(impact, -v.z);  // floor
    if (kShellCeilZ - (p.z + R) < kShellMargin) impact = std::max(impact,  v.z);  // ceiling
    if (kShellHalfX - (p.x + R) < kShellMargin) impact = std::max(impact,  v.x);  // +x wall
    if ((p.x - R) + kShellHalfX < kShellMargin) impact = std::max(impact, -v.x);  // -x wall

    // +-y walls: absent across the goal mouth (x/z inside the opening), and
    // nonexistent once the ball is already beyond the goal line.
    const bool inGoal = std::fabs(p.y) > kShellHalfY;
    const bool inMouth = std::fabs(p.x) < kGoalHalfW && p.z < kGoalHeight;
    if (!inGoal && !inMouth) {
        if (kShellHalfY - (p.y + R) < kShellMargin) impact = std::max(impact,  v.y);
        if ((p.y - R) + kShellHalfY < kShellMargin) impact = std::max(impact, -v.y);
    }

    if (impact < kThudMinImpact) return;
    lastSurfaceTick_ = ticksSimulated;
    if (onBallSurfaceHit) {
        BallSurfaceHitEvent ev;
        ev.strength = impact;
        onBallSurfaceHit(ev);
    }
}

void Sim::advance(double dt) {
    if (paused || !arena_) return;
    accum_ += dt;
    int steps = 0;
    while (accum_ >= tickDt && steps < 8) {
        stepOnce();
        accum_ -= tickDt;
        steps++;
    }
    if (accum_ > tickDt) accum_ = 0;   // dropped too many frames; don't spiral
}

SimSnapshot Sim::snapshot(float alpha) const {
    SimSnapshot s;
    auto lerpState = [&](const PhysState& a, const PhysState& b,
                         V3& pos, V3& vel, V3& ang) {
        pos = V3::lerp(toV3(a.pos), toV3(b.pos), alpha);
        vel = V3::lerp(toV3(a.vel), toV3(b.vel), alpha);
        ang = V3::lerp(toV3(a.angVel), toV3(b.angVel), alpha);
    };
    lerpState(prevCar_, curCar_, s.carPos, s.carVel, s.carAngVel);
    lerpState(prevBall_, curBall_, s.ballPos, s.ballVel, s.ballAngVel);

    V3 f0 = toV3(prevCar_.rotMat.forward), r0 = toV3(prevCar_.rotMat.right), u0 = toV3(prevCar_.rotMat.up);
    V3 f1 = toV3(curCar_.rotMat.forward), r1 = toV3(curCar_.rotMat.right), u1 = toV3(curCar_.rotMat.up);
    lerpFrame(f0, r0, u0, f1, r1, u1, alpha, s.carF, s.carR, s.carU);

    V3 bf0 = toV3(prevBall_.rotMat.forward), br0 = toV3(prevBall_.rotMat.right), bu0 = toV3(prevBall_.rotMat.up);
    V3 bf1 = toV3(curBall_.rotMat.forward), br1 = toV3(curBall_.rotMat.right), bu1 = toV3(curBall_.rotMat.up);
    lerpFrame(bf0, br0, bu0, bf1, br1, bu1, alpha, s.ballF, s.ballR, s.ballU);

    s.boost = curCar_.boost;
    s.onGround = curCar_.isOnGround;
    s.hasMapFloor = mapLoaded_;
    s.floorZ = mapFloorZ_;
    s.boosting = curCar_.isBoosting;
    s.supersonic = curCar_.isSupersonic;
    s.jumping = curCar_.isJumping;
    s.flipping = curCar_.isFlipping;
    s.flipReset = flipResetHeld_;
    s.flipResetAge = flipResetAge_;

    const BallHitInfo& hi = curCar_.ballHitInfo;
    s.ballHitValid = hi.isValid && hi.tickCountWhenHit == lastHitTick_;
    if (s.ballHitValid) {
        s.ballHitPos = toV3(hi.ballPos);
        s.ballHitStrength = hi.extraHitVel.Length();
    }
    return s;
}

// ---------------------------------------------------------------- freeplay

V3 Sim::spawnPosForPreset(int presetIndex) const {
    switch (presetIndex % presetCount()) {
        case 0: return {-2048, -2560, RLConst::CAR_SPAWN_REST_Z};   // blue corner kickoff
        case 1: return { 2048,  2560, RLConst::CAR_SPAWN_REST_Z};   // orange corner kickoff
        case 2: return {    0, -2048, RLConst::CAR_SPAWN_REST_Z};   // midfield, blue half
        case 3: return {    0,  2048, RLConst::CAR_SPAWN_REST_Z};   // midfield, orange half
        case 4: return {    0,     0, 1200};                        // dropped from the air
        case 5: return {    0, -4600, RLConst::CAR_SPAWN_REST_Z};   // in front of blue net
        default: return {0, -2560, RLConst::CAR_SPAWN_REST_Z};
    }
}

float Sim::spawnYawForPreset(int presetIndex) const {
    switch (presetIndex % presetCount()) {
        case 0: return 0.25f * (float)M_PI;              // face center
        case 1: return -0.75f * (float)M_PI;             // face center
        case 2: return 0.5f * (float)M_PI;               // face orange goal
        case 3: return -0.5f * (float)M_PI;              // face blue goal
        case 4: return 0.5f * (float)M_PI;
        case 5: return 0.5f * (float)M_PI;
        default: return 0.5f * (float)M_PI;
    }
}

// Reset Shot: car to a kickoff/drop preset, ball back to the middle at rest -
// every press is the same clean shot (or kickoff) to play.
void Sim::resetShot(int presetIndex) {
    if (!car_) return;

    // With a custom map loaded the preset is meaningless: Reset Shot returns
    // to the last checkpoint the car reached, which owns both a car spawn and
    // (when the map provides one) a ball placement.
    V3 spawn = spawnPosForPreset(presetIndex);
    float yaw = spawnYawForPreset(presetIndex);
    bool ballAtCheckpoint = false;
    V3 ballSpawn;

    if (mapLoaded_ && !map_.sessions.empty()) {
        if (session_ < 0 || session_ >= (int)map_.sessions.size()) session_ = 0;
        const maploader::MapSession& ms = map_.sessions[(size_t)session_];
        if (ms.hasCar) {
            spawn = ms.carPos;
            yaw = ms.carYawRad;
        }
        if (ms.hasBall) {
            ballAtCheckpoint = true;
            ballSpawn = ms.ballPos;
        }
    }

    CarState s = car_->GetState();
    s.pos = toVec(spawn);
    s.rotMat = Angle(yaw, 0, 0).ToRotMat();
    s.vel = Vec(0, 0, 0);
    s.angVel = Vec(0, 0, 0);
    s.flipRelTorque = Vec(0, 0, 0);

    s.isOnGround = true;
    s.hasJumped = false;
    s.hasDoubleJumped = false;
    s.hasFlipped = false;
    s.isFlipping = false;
    s.isJumping = false;
    s.isAutoFlipping = false;
    s.autoFlipTimer = 0;
    s.jumpTime = 0;
    s.flipTime = 0;
    s.airTime = 0;
    s.airTimeSinceJump = 0;
    s.boost = 100;
    s.isBoosting = false;
    s.boostingTime = 0;
    s.isDemoed = false;
    s.isSupersonic = false;
    s.supersonicTime = 0;
    s.handbrakeVal = 0;
    s.worldContact.hasContact = true;
    s.worldContact.contactNormal = Vec(0, 0, 1);

    car_->SetState(s);
    car_->controls = CarControls();

    if (arena_ && arena_->ball) {
        // Default arena: the middle at rest. Custom map: the checkpoint's own
        // ball placement, or - when the map provides none - set down a short
        // way in front of the car. Directly above the car would spawn the ball
        // inside the roof and leave it resting on top of the car.
        Vec ballPos(0, 0, RLConst::BALL_REST_Z);
        if (mapLoaded_ && !ballAtCheckpoint) {
            const float cy = std::cos(yaw), sy = std::sin(yaw);
            ballPos = Vec(spawn.x + 320.0f * cy, spawn.y + 320.0f * sy, spawn.z + ballR_);
        }
        if (ballAtCheckpoint) ballPos = toVec(ballSpawn);

        BallState bs;
        bs.pos = ballPos;
        bs.rotMat = RotMat::GetIdentity();
        bs.vel = Vec(0, 0, 0);
        bs.angVel = Vec(0, 0, 0);
        arena_->ball->SetState(bs);
    }

    flipResetHeld_ = false;
    flipResetAge_ = 0;
    lastSurfaceTick_ = ticksSimulated;   // the fresh ball drop is not an impact

    prevCar_ = curCar_ = car_->GetState();
    if (arena_ && arena_->ball) prevBall_ = curBall_ = arena_->ball->GetState();
    accum_ = 0;
}

// --- custom maps ------------------------------------------------------------

// Radius (squared, uu) within which reaching a checkpoint's spawn marks it
// reached - matched to how far apart the PlayerSpawn/BallSpawn pairs sit.
static constexpr float kCheckpointReach = 600.0f;

bool Sim::loadMap(const std::string& path, std::string* err) {
    std::string local;
    if (!err) err = &local;
    err->clear();

    maploader::MapData data;
    std::string perr;
    if (!::maploader::loadMap(path, &data, &perr)) {
        // A failed load always ends on the default arena - never leave a
        // half-loaded map behind.
        unloadMap();
        *err = perr.empty() ? "could not read package" : perr;
        return false;
    }
    return applyMap(std::move(data), err);
}

bool Sim::applyMap(maploader::MapData&& data, std::string* err) {
    std::string local;
    if (!err) err = &local;
    err->clear();
    if (!arena_) {
        *err = "simulation not ready";
        return false;
    }

    // Put the arena back on its built-in collision first so the swap is
    // always against a clean baseline, then bring in the map's geometry.
    unloadMap();

    std::string merr;
    mapCol_ = applyMapCollision(arena_, data, &merr);
    if (!mapCol_) {
        *err = merr.empty() ? "map has no collidable geometry" : merr;
        return false;
    }

    map_ = std::move(data);
    mapLoaded_ = true;
    mapGen_++;                       // a different map: re-upload, re-collide
    session_ = 0;
    resetShot(0);
    return true;
}

void Sim::unloadMap() {
    releaseMapCollision(arena_, mapCol_);
    mapCol_ = nullptr;
    mapLoaded_ = false;
    mapGen_++;
    session_ = 0;
    map_ = maploader::MapData();
    if (car_) resetShot(0);
}

void Sim::cycleSession(int delta) {
    if (!mapLoaded_ || map_.sessions.empty() || delta == 0) return;
    const int n = (int)map_.sessions.size();
    session_ = (((session_ + delta) % n) + n) % n;
    resetShot(session_);
}

// A checkpoint is reached by proximity, but it only takes over when it is
// strictly the closest one to the car. Without that comparison two points
// inside each other's reach (PathNodes cluster several per platform) hand the
// session back and forth every frame, firing the callback the whole way.
void Sim::updateSessionByProximity() {
    if (map_.sessions.empty()) return;
    if (session_ < 0 || session_ >= (int)map_.sessions.size()) session_ = 0;
    const V3 p(curCar_.pos.x, curCar_.pos.y, curCar_.pos.z);
    const V3 here = map_.sessions[(size_t)session_].carPos - p;
    const float here2 = here.dot(here);

    int best = -1;
    float best2 = 0.f;
    for (size_t i = 0; i < map_.sessions.size(); i++) {
        if ((int)i == session_) continue;
        const maploader::MapSession& s = map_.sessions[i];
        const float reach = s.reach > 0.f ? s.reach : kCheckpointReach;
        const V3 d = s.carPos - p;
        const float d2 = d.dot(d);
        if (d2 > reach * reach) continue;
        if (best < 0 || d2 < best2) {
            best = (int)i;
            best2 = d2;
        }
    }

    if (best >= 0 && best2 < here2) {
        session_ = best;
        if (onCheckpointReached)
            onCheckpointReached(session_, (int)map_.sessions.size());
    }
}

void Sim::launchBall(const FreeplaySettings& fs) {
    if (!arena_ || !arena_->ball) return;
    // Launch in place: the ball keeps exactly where it already is - on the
    // floor, on your roof, mid-chase - and is only given the launch velocity,
    // so it never teleports back to the middle.
    BallState bs = arena_->ball->GetState();
    bs.angVel = Vec(0, 0, 0);

    // fire downfield relative to the car's half
    float dirY = (curCar_.pos.y < 0) ? 1.0f : -1.0f;
    float ang = clampf(fs.launchAngle, -90, 90) * (float)M_PI / 180.0f;
    float sp = clampf(fs.launchSpeed, 0, 6000);
    bs.vel = Vec(0, dirY * std::cos(ang) * sp, std::sin(ang) * sp);

    arena_->ball->SetState(bs);
    prevBall_ = curBall_ = arena_->ball->GetState();
    // Artificial launch: suppress thuds for the next few ticks so a
    // downward launch can't fake an impact against the floor it starts on.
    lastSurfaceTick_ = ticksSimulated;
}

void Sim::startDribble() {
    if (!arena_ || !arena_->ball || !car_) return;
    CarState cs = car_->GetState();
    BallState bs;

    V3 up = toV3(cs.rotMat.up);
    V3 fwd = toV3(cs.rotMat.forward);
    // roof of the Octane hitbox: hitbox offset z + half height
    float roofOffset = 20.755f + 38.6591f * 0.5f;
    float ballR = arena_->ball->GetRadius();

    V3 p = toV3(cs.pos) + up * (roofOffset + ballR + 6.0f) + fwd * 20.0f;
    bs.pos = toVec(p);
    bs.rotMat = RotMat::GetIdentity();
    bs.vel = cs.vel;
    bs.angVel = Vec(0, 0, 0);

    arena_->ball->SetState(bs);
    prevBall_ = curBall_ = arena_->ball->GetState();
}

// Take Possession: drop the ball directly in front of the car - on the floor
// when we're grounded, or at our own height when we're mid-air (an aerial to
// chase). The car keeps whatever it was doing.
void Sim::takePossession() {
    if (!arena_ || !arena_->ball || !car_) return;
    CarState cs = car_->GetState();

    // "In front" flattened to the horizontal plane. With the nose pointed
    // near-vertical (wall/ceiling play) aim across the pitch instead: from
    // the car towards wherever the ball currently is, +X as a last resort.
    V3 f = toV3(cs.rotMat.forward);
    V3 dir(f.x, f.y, 0);
    if (dir.len() < 0.15f) {
        V3 d = toV3(arena_->ball->GetState().pos) - toV3(cs.pos);
        dir = V3(d.x, d.y, 0);
        if (dir.len() < 0.15f) dir = V3(1, 0, 0);
    }
    dir = dir.norm();

    const float ballR = arena_->ball->GetRadius();
    // Past the nose (Octane half length ~59 uu) + the ball's own radius, plus
    // a cushion, so it lands clear of the car and stays an easy first touch.
    constexpr float kCushion = 170.0f;
    const float ahead = 59.0f + ballR + kCushion;
    V3 p = toV3(cs.pos) + dir * ahead;
    const float z = std::max(cs.pos.z, RLConst::BALL_REST_Z);

    BallState bs;
    bs.pos = Vec(p.x, p.y, z);
    bs.rotMat = RotMat::GetIdentity();
    bs.vel = Vec(0, 0, 0);
    bs.angVel = Vec(0, 0, 0);
    arena_->ball->SetState(bs);
    prevBall_ = curBall_ = arena_->ball->GetState();
}

void Sim::applyControlSettings(const ControlSettings& cs) {
    if (car_) car_->config.dodgeDeadzone = clampf(cs.dodgeDeadzone, 0, 0.95f);
}
