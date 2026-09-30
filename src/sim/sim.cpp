#include "sim.h"

#include "RocketSim/src/RocketSim.h"

#include <algorithm>
#include <filesystem>

using namespace RocketSim;

static V3 toV3(const Vec& v) { return {v.x, v.y, v.z}; }
static Vec toVec(const V3& v) { return Vec(v.x, v.y, v.z); }

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

    car_ = arena_->AddCar(Team::BLUE, CAR_CONFIG_OCTANE);
    if (!car_) {
        std::fprintf(stderr, "[sim] AddCar failed\n");
        return false;
    }
    car_->config.dodgeDeadzone = 0.7f;

    takePosition(0);

    prevCar_ = curCar_ = car_->GetState();
    prevBall_ = curBall_ = arena_->ball->GetState();
    return true;
}

void Sim::shutdown() {
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

    // boost never drains (also guarantees full meter even if mutator is bypassed)
    if (curCar_.boost < 100) {
        curCar_.boost = 100;
        RocketSim::CarState s = car_->GetState();
        s.boost = 100;
        car_->SetState(s);
    }

    const BallHitInfo& hi = curCar_.ballHitInfo;
    if (hi.isValid && hi.tickCountWhenHit != lastHitTick_) {
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
    s.boosting = curCar_.isBoosting;
    s.supersonic = curCar_.isSupersonic;
    s.jumping = curCar_.isJumping;
    s.flipping = curCar_.isFlipping;

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

void Sim::takePosition(int presetIndex) {
    if (!car_) return;
    CarState s = car_->GetState();
    s.pos = toVec(spawnPosForPreset(presetIndex));
    s.rotMat = Angle(spawnYawForPreset(presetIndex), 0, 0).ToRotMat();
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

    prevCar_ = curCar_ = car_->GetState();
    if (arena_ && arena_->ball) prevBall_ = curBall_ = arena_->ball->GetState();
    accum_ = 0;
}

void Sim::launchBall(const FreeplaySettings& fs) {
    if (!arena_ || !arena_->ball) return;
    BallState bs;
    bs.pos = Vec(0, 0, RLConst::BALL_REST_Z);
    bs.rotMat = RotMat::GetIdentity();
    bs.vel = Vec(0, 0, 0);
    bs.angVel = Vec(0, 0, 0);

    // fire downfield relative to the car's half
    float dirY = (curCar_.pos.y < 0) ? 1.0f : -1.0f;
    float ang = clampf(fs.launchAngle, -90, 90) * (float)M_PI / 180.0f;
    float sp = clampf(fs.launchSpeed, 0, 6000);
    bs.vel = Vec(0, dirY * std::cos(ang) * sp, std::sin(ang) * sp);

    arena_->ball->SetState(bs);
    prevBall_ = curBall_ = arena_->ball->GetState();
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

void Sim::applyControlSettings(const ControlSettings& cs) {
    if (car_) car_->config.dodgeDeadzone = clampf(cs.dodgeDeadzone, 0, 0.95f);
}
