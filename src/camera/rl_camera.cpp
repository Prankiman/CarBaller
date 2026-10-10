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
    axisYaw_ = carYaw_;       // facing axis starts on the nose bearing
    hasAxisYaw_ = true;
    // Ball-cam aim re-anchors on the snapshot below instead of easing over
    // from wherever it was: a preset move is a reposition, not a swing.
    hasBallAim_ = false;
    hasPrevPhi_ = false;
    blend_ = (desired_ == CamMode::Ball) ? 1.0f : 0.0f;  // snap, no animation
    hasSmoothed_ = false;
    shakeAmp_ = 0;

    V3 e, t;
    if (blend_ > 0.5f)
        computeBallCam(cs, s, 0.0f, e, t);
    else
        computeCarCam(cs, s, 0.0f, e, t);
    smoothedEye_ = e;
    prevWantEye_ = e;
    prevRigOffZ_ = e.z - s.carPos.z;
    hasSmoothed_ = true;
    smoothTarget_ = t;
    prevWantT_ = t;
    derivS_ = 0;
    hasSmoothTarget_ = true;
    eye = e;
    target = t;
}

// Floor (uu) on the horizontal car->ball offset for the bearing to be worth
// reading: below this the ball sits on the car's vertical axis and atan2
// returns direction noise, so the aim is held instead.
static constexpr float kBallAimEps = 5.0f;

// Max rate (rad/s) the ball-cam rig may change its own aim.
//
// The rig used to read the aim straight off the snapshot, and the measured
// car->ball bearing is only well conditioned while the ball is clear of the
// car's vertical axis. As the ball passes directly over (or under) the car -
// a drop on the roof, a ball crossing overhead - the horizontal offset runs
// through zero and atan2 flips in a single frame, with the same result on
// either side of the crossing: the eye teleported to the far side of the car
// and the view spun around in ONE frame. The eye follower folds
// frame-to-frame rig speed into its follow rate, so a jump like that lands
// its per-frame factor at ~1: the camera snapped to the new pose instead of
// easing into it.
//
// Measured by driving the update loop through the degenerate cases
// (tools/probe_camera.cpp): an overhead crossing moved the eye 516 uu and
// turned the view 75 deg in one frame, a ball landing on the roof 225 uu /
// 50 deg, a ball drifting around overhead (bearing pure noise) 534 uu /
// 95 deg - all one-frame jumps.
//
// 20 rad/s is just under the fastest sweep real play can ask for (a 4000
// uu/s ball passing at grazing range, ~150 uu off the car's axis, sweeps
// ~27 rad/s), so tracking stays within ~1 deg of the unlimited case
// everywhere except the degenerate ones - which now swing around the car
// over ~0.15 s instead of jumping. Note this is deliberately above the car
// cam's nose-follow rate: the nose is never degenerate, the ball bearing is.
static constexpr float kBallAimSlew = 20.0f;

// ---------------------------------------------------------------- eye placement
//
// Both rigs used to sit at `carPos - fwd * distance + (0,0,height)` and feed
// the pitch swivel into the VIEW ONLY. That made the two axes asymmetric:
// left/right swung the eye around the car (the car held its place on screen
// while the world swept past), while up/down rotated the camera around its own
// starting point - a fixed eye and a tilting view, so the car slid across the
// screen. Rocket League rotates around the car on both axes, so the pitch
// swivel now swings the rig too.
//
// The default pose is one point on a sphere of radius R = hypot(distance,
// height) at elevation e0 = atan2(height, distance): pitchOff_ shifts the
// elevation (looking up carries the eye down, looking down carries it up),
// and the view pitch is then derived from wherever the eye actually ended up,
// which keeps the car at a constant screen offset for the whole swing. What
// the orbit cannot absorb comes back as a tilt - `outTilt` - so the control
// never goes dead.
//
// The one hard stop is the floor: the renderer always draws it opaque
// (Renderer::render), so an eye below it would look at the car through solid
// ground. On the ground the limit is the surface the car rests on (the corner
// ramps rise well above z = 0); in the air only the floor itself matters, so
// the full swing stays available exactly where it is most used - looking up
// mid-air. Past either stop the rest of the swivel degrades into the old
// tilt, smoothly and without a jump.
static constexpr float kEyeFloorZ = 16.0f;      // uu: eye stays above the arena floor
static constexpr float kCarRideHeight = 17.0f;  // car center above the surface it sits on
static constexpr float kMaxElev = 1.5f;         // rad: keep the eye behind the car (cos > 0)

void RLCamera::placeEye(const CameraSettings& cs, const SimSnapshot& s,
                        float lookYaw, V3& outEye, float& outTilt) const {
    const float rigR = std::hypot(cs.distance, cs.height);
    const float e0 = std::atan2(cs.height, cs.distance);

    float elev = e0 - pitchOff_;

    const float minZ = s.onGround ? std::max(kEyeFloorZ, s.carPos.z - kCarRideHeight)
                                  : kEyeFloorZ;
    const float sinMin = clampf((minZ - s.carPos.z) / rigR, -1.0f, 1.0f);
    elev = std::max(elev, std::asin(sinMin));
    elev = clampf(elev, -kMaxElev, kMaxElev);

    const V3 fwd(std::cos(lookYaw), std::sin(lookYaw), 0);
    outEye = s.carPos - fwd * (rigR * std::cos(elev)) + V3(0, 0, rigR * std::sin(elev));
    outTilt = pitchOff_ - (e0 - elev);   // swivel the rig could not take up
}

void RLCamera::computeBallCam(const CameraSettings& cs, const SimSnapshot& s,
                              float dt, V3& outEye, V3& outTarget) {
    const V3 toBall = s.ballPos - s.carPos;
    const bool reliable = toBall.len2d() > kBallAimEps;
    const bool first = !hasBallAim_;
    const float slewStep = kBallAimSlew * dt;

    // ---- horizontal aim: keep the bearing on one continuous branch.
    // measYaw_ unwraps the raw bearing frame to frame (always the short way,
    // so a far-side teleport still approaches the short way round), while
    // ballYaw_ chases it at the slew rate. Tracking the unwrapped
    // measurement - rather than the error against ballYaw_ directly - is what
    // keeps the branch choice correct through the overhead crossing, where
    // the shortest path from the current aim would send the rig the wrong
    // way around the car.
    if (reliable) {
        const float phi = std::atan2(toBall.y, toBall.x);
        if (first) {
            ballYaw_ = measYaw_ = phi;  // reset/preset: anchor, don't ease in
            hasBallAim_ = true;
        } else {
            measYaw_ = hasPrevPhi_ ? measYaw_ + wrapAngle(phi - prevPhi_) : phi;
            ballYaw_ += clampf(measYaw_ - ballYaw_, -slewStep, slewStep);
        }
        prevPhi_ = phi;
        hasPrevPhi_ = true;
    } else if (first) {
        // Reset with the ball dead on the car's vertical axis: there is no
        // bearing to anchor to, so take the car's heading and wait for the
        // first real measurement.
        ballYaw_ = measYaw_ = std::atan2(s.carF.y, s.carF.x);
        hasBallAim_ = true;
    }
    // (otherwise: ball on the vertical axis - hold, the bearing is noise.)

    // Keep both angles inside (-pi,pi] by the same amount so their
    // difference - the part the slew actually clamps - stays exact.
    if (ballYaw_ > (float)M_PI) {
        ballYaw_ -= 2 * (float)M_PI;
        measYaw_ -= 2 * (float)M_PI;
    } else if (ballYaw_ < -(float)M_PI) {
        ballYaw_ += 2 * (float)M_PI;
        measYaw_ += 2 * (float)M_PI;
    }

    float lookYaw = ballYaw_ + yawOff_;
    float tilt;      // pitch swivel the orbit could not absorb (see placeEye)
    placeEye(cs, s, lookYaw, outEye, tilt);

    // ---- vertical aim: same treatment, minus the branch (the elevation of
    // a point above the horizon plane is single-valued). What this buys is
    // the ball teleport - dribble puts the ball on the roof, possession drops
    // it ahead - which used to kick the view up/down in one frame.
    V3 toT = s.ballPos - outEye;
    const float measPitch = std::atan2(toT.z, std::max(toT.len2d(), 1.0f));
    if (first) ballPitch_ = measPitch;
    else ballPitch_ += clampf(measPitch - ballPitch_, -slewStep, slewStep);

    float pitch = clampf(ballPitch_ + cs.angle * (float)M_PI / 180.0f + tilt,
                         -1.4f, 1.4f);

    V3 dir = dirFromYawPitch(lookYaw, pitch);
    outTarget = outEye + dir * std::max(toT.len(), 1000.0f);
}

// ---------------------------------------------------------------- air heading
//
// Rocket League's car cam does not "pick a side of the car at takeoff and
// lock it": it sits behind the direction the car is TRAVELLING. On the
// ground those two are the same thing (you drive where you point, and
// reversing does not swing the camera around - which is why the ground case
// below still follows the facing rigidly). In the air they come apart, and
// the camera follows the momentum: a flip eases it behind the new velocity,
// pitching or air-rolling moves it not at all, and it settles over time
// rather than deciding once.
//
// Evidence, since RL ships no document for this:
//  - Dignitas, "Understanding the Cameras": "The car camera is pointed in
//    your car's direction of travel, and will rigidly stay straight as long
//    as your car is on the ground."
//  - r/RocketLeague on the setting: "the camera angle will always change
//    depending on where the ball is (for ballcam) or on the car momentum".
//  - In-game Stiffness tooltip, "controls how rigidly your camera follows
//    your car", plus community measurements of it as car-cam inertia
//    ("a certain amount of inertia ... this effect only occurs in car cam"),
//    which is the knob that owns the ease below.
//
// So the air target is the horizontal velocity heading, blended with the
// (pitch-immune) nose heading as the speed runs out: at a stall or a
// straight-up hover there is no horizontal direction to sit behind, so the
// facing takes over instead of the bearing collapsing onto noise.

// uu/s: below this the horizontal velocity is not a direction (stall,
// vertical hover, the first frames of a jump) and the nose heading carries
// the target on its own.
static constexpr float kVelDirMin = 200.0f;
// uu/s: at and above this the camera is fully on the velocity heading. A
// cruising aerial sits at 800-2300 uu/s, so real flight always lands here;
// only the blended middle band (a hop, a landing bounce, a slow fly-in)
// mixes the two.
static constexpr float kVelDirMax = 900.0f;

// Per-second rate the car-cam heading chases its target ON THE GROUND:
// rigid, matching RL's "will rigidly stay straight as long as your car is
// on a surface". The air uses the stiffness ease below instead.
static constexpr float kYawFollowRate = 12.0f;
// The ball cam gets its own, higher cap (kBallAimSlew, defined above
// computeBallCam): the nose heading can spin but never jumps, while the
// car->ball bearing is undefined over the car's vertical axis, so ball cam
// has to be able to swing harder to recover from a degenerate frame.

// Air ease: a first-order (exponential) approach onto the target heading,
// per second, with Stiffness doing exactly what RL's Stiffness does - it is
// the only one of RL's camera settings that governs how tightly the camera
// tracks the car, and the measured behaviour of it is inertia in car cam
// (low stiffness floats behind you, high stiffness snaps onto you). At the
// default stiffness 0.5 this is 6/s: 63% of the way to a new heading in
// 0.17s, 95% in 0.5s - the camera trails the momentum into place, with no
// window and no lock, so it can be re-aimed by the very next change in
// velocity instead of waiting out a decision.
static constexpr float kAirEaseBase = 2.5f;   // 1/s at stiffness 0
static constexpr float kAirEaseStiff = 7.0f;  // 1/s per unit of stiffness

// The rear/front axis the camera sits on when it is following the car's
// FACING (ground, and the air's low-speed fallback), expressed as a world
// heading, kept PITCH-IMMUNE. It is the horizontal projection of the nose -
// and the rear's projection is the same line 180 deg round, so tracking nose
// or rear is literally the same axis and it does not matter which end is
// named.
//
// The projection is independent of pitch almost everywhere: pitch the car
// from level to 80 deg and the bearing does not move at all. The one place
// it breaks is EXACTLY vertical: once pitch passes 90 deg, cos(pitch)
// changes sign and atan2 reports the same line flipped by 180 deg, with no
// way to tell the two apart from the frame alone. Following that jump orbits
// the camera around the car on a pure pitch input - measured
// (tools/probe_camera.cpp, "air camera" section): a 0->120 deg pitch swung
// the camera 180 deg around the car at 75 uu/frame, while any pitch inside
// +-90 deg moved it 0. That is pitch driving the camera position, which
// Rocket League does not do.
//
// Both branches are the same line, so each frame simply keeps whichever end
// is continuous with the last one. A real yaw turn moves the bearing by
// ~5 deg per frame even at full air steer (120 Hz), while the pitch flip is
// a single-frame 180 deg jump - so nearest-branch follows steering and drops
// the flip. The result: pitching through vertical, or all the way around,
// leaves the camera exactly where it was.
static constexpr float kAxisHalfTurn = 0.5f * (float)M_PI;
// Nose within ~81 deg of straight up/down: its horizontal projection is too
// short to be a direction, so the axis is held at its last good value.
static constexpr float kAxisMinFxy = 0.15f;

// Cap (uu) on how far the smoothed eye trails the ideal rig while the RIG
// itself is moving - horizontally (swivel, car turns, boost) and vertically
// (the pitch swivel carrying the eye around the car); the lag asymptotes to
// this value at high speed. It is deliberately never applied to the car's own
// motion, which is what keeps RL's stiffness lag (jumps, aerials) intact.
static constexpr float kRigLagCap = 40.0f;

void RLCamera::computeCarCam(const CameraSettings& cs, const SimSnapshot& s,
                             float dt, V3& outEye, V3& outTarget) {
    // Grounded = rigidly behind the car's facing (RL keeps the camera
    // straight as long as the car is on a surface; reversing never swings
    // it). Airborne = eased onto the direction of travel, continuously - no
    // takeoff hold, no decision window, no lock: the momentum is re-read
    // every frame, so a flip re-aims the camera and a pitch or air roll
    // leaves it alone. The eye itself (behind the rear, and around the car
    // under the pitch swivel) is placed by placeEye below.
    const float fxy = std::sqrt(s.carF.x * s.carF.x + s.carF.y * s.carF.y);

    // ---- pitch-immune facing axis (see the long note above): the nose
    // bearing, kept continuous frame to frame so a pitch past vertical -
    // which flips the bearing by 180 deg without the car turning at all -
    // cannot drag the camera around the car. Held, not snapped, while the
    // nose is too close to straight up/down for its projection to be a
    // direction (that last-good axis is already what the camera shows, so
    // holding it is also continuous).
    //
    // On the GROUND the branch rule is skipped: the raw bearing is taken
    // as the axis, every frame. Wheels define the attitude there, so the
    // projection cannot be flipped by pitch, and reading it fresh keeps
    // the ground camera self-correcting - which also heals the axis for
    // the next takeoff. The branch rule is one-way: continuity can pin it
    // to the wrong side (an airborne near-vertical manoeuvre moves the
    // real heading by >90 deg while the nose is too vertical to read, and
    // it locks onto bearing + 180), and nothing in the air can tell "held
    // past vertical" from "corrupted" - both present as the same
    // persistent 180 deg disagreement. Before this split, a landing eased
    // the camera onto that corrupted axis and it sat in FRONT of the car
    // permanently - measured at 180.0 deg off after level flight -> nose
    // to 88 deg -> yaw 120 -> land (regression case "land after vertical"
    // in tools/probe_camera.cpp).
    if (fxy > kAxisMinFxy) {
        const float bearing = std::atan2(s.carF.y, s.carF.x);
        if (s.onGround || !hasAxisYaw_) {
            axisYaw_ = bearing;
            hasAxisYaw_ = true;
        } else {
            const float d = wrapAngle(bearing - axisYaw_);
            axisYaw_ = (std::fabs(d) <= kAxisHalfTurn)
                           ? bearing
                           : wrapAngle(bearing + (float)M_PI);
        }
    }

    // ---- pick what the camera sits behind this frame: the facing on the
    // ground, the direction of travel in the air (facing as the speed runs
    // out). Blended on the unit circle so a heading that has wrapped
    // through +-180 deg interpolates the short way round.
    float targetYaw = axisYaw_;
    if (!s.onGround) {
        const float speed2d = std::hypot(s.carVel.x, s.carVel.y);
        float w = clampf((speed2d - kVelDirMin) / (kVelDirMax - kVelDirMin),
                         0.0f, 1.0f);
        w = w * w * (3.0f - 2.0f * w);   // ease the handover, no kink
        if (w > 0.0f) {
            const float velYaw = std::atan2(s.carVel.y, s.carVel.x);
            const float x = std::cos(axisYaw_) +
                            w * (std::cos(velYaw) - std::cos(axisYaw_));
            const float y = std::sin(axisYaw_) +
                            w * (std::sin(velYaw) - std::sin(axisYaw_));
            if (x * x + y * y > 1e-6f) targetYaw = std::atan2(y, x);
        }
    }

    // ---- ease onto it: rigid on the ground, stiffness-scaled inertia in
    // the air (kAirEase* above). Exponential, so the per-frame step shrinks
    // with the error - the heading arrives without a visible stop.
    const float rate = s.onGround
                           ? kYawFollowRate
                           : kAirEaseBase + kAirEaseStiff * cs.stiffness;
    const float delta = wrapAngle(targetYaw - carYaw_);
    carYaw_ += delta * (1.0f - std::exp(-rate * dt));

    float lookYaw = carYaw_ + yawOff_;
    float tilt;      // pitch swivel the orbit could not absorb (see placeEye)
    placeEye(cs, s, lookYaw, outEye, tilt);

    V3 aim = s.carPos + V3(0, 0, cs.height * 0.25f);
    V3 toT = aim - outEye;
    float basePitch = std::atan2(toT.z, std::max(toT.len2d(), 1.0f));
    float pitch = clampf(basePitch + cs.angle * (float)M_PI / 180.0f + tilt,
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
    computeBallCam(cs, s, dt, ballEye, ballTarget);
    computeCarCam(cs, s, dt, carEye, carTarget);

    V3 wantEye = V3::lerp(carEye, ballEye, blend_);
    V3 wantTarget = V3::lerp(carTarget, ballTarget, blend_);

    // ---- stiffness: position follow smoothing. Rocket League's stiffness
    // shows up as world-space lag: jumps displace the camera and it
    // recenters, and during an aerial the eye lingers near the spot that was
    // behind the car at takeoff instead of climbing rigidly with it. That
    // lag belongs to the CAR's motion: horizontal follow below stays tight
    // so fast forward flight doesn't drag the camera far behind the car, and
    // vertical follow keeps its slow rate for the jumps/aerials.
    //
    // Everything here is a per-SECOND rate; the per-frame factor is always
    // 1 - exp(-rate * dt), which stays in (0,1) for any speed. Adding to the
    // factor instead of the rate lets it exceed 1 and the eye diverges.
    const float followRate = 1.5f + cs.stiffness * 14.0f;
    const float baseRateZ = s.onGround ? followRate : followRate * 0.5f;

    // A plain exponential follower settles v/rate behind a moving rig - at
    // boost speed (2300 uu/s, default stiffness) that was a whole camera
    // distance, so the eye sat far from the car while boosting and only
    // crept back after the car slowed. The rig-speed term in the horizontal
    // rate makes that lag saturate at kRigLagCap uu instead: rigid at speed
    // like RL, while rest and low-speed recentering keep the original
    // stiffness response.
    const float dx = wantEye.x - prevWantEye_.x;
    const float dy = wantEye.y - prevWantEye_.y;
    const float rigSpeed2d = std::sqrt(dx * dx + dy * dy) / std::max(dt, 1e-4f);
    const float rateX = followRate + rigSpeed2d / kRigLagCap;
    prevWantEye_ = wantEye;

    // The rig's OWN vertical motion is a different animal: it is the pitch
    // swivel carrying the eye around the car (placeEye), and trailing that
    // by the stiffness rate would slide the car across the screen - the very
    // thing the orbit exists to prevent (at full swivel the ideal eye moves
    // ~1000 uu/s, which is a 118 uu / 23 deg trail at baseRateZ). Subtracting
    // the car gives exactly that motion: a jumping or climbing car reads as
    // zero rig-relative speed and keeps its linger, the swivel reads as its
    // own speed and picks up the same saturating term the horizontal rate
    // has. With no vertical swivel this term is identically zero, so the
    // tuned stiffness response is untouched.
    const float rigOffZ = wantEye.z - s.carPos.z;
    const float rigVz = (rigOffZ - prevRigOffZ_) / std::max(dt, 1e-4f);
    prevRigOffZ_ = rigOffZ;
    const float rateZ = baseRateZ + std::fabs(rigVz) / kRigLagCap;

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
