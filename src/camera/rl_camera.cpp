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
    ballOrbitAim_ = 0;
    swivelIdle_ = 0;
    carYaw_ = std::atan2(s.carF.y, s.carF.x);
    axisYaw_ = carYaw_;       // facing axis starts on the nose bearing
    hasAxisYaw_ = true;
    // Ball-cam aim re-anchors on the snapshot below instead of easing over
    // from wherever it was: a preset move is a reposition, not a swing.
    hasBallAim_ = false;
    hasPrevPhi_ = false;
    hasSwing_ = false;
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
// Max rate (rad/s) the ball-cam rig may change its own VIEW PITCH.
//
// History: the rig used to read both aim axes straight off the snapshot, and
// the horizontal bearing is only well conditioned while the ball is clear of
// the car's vertical axis. As the ball passes directly over (or under) the
// car the horizontal offset runs through zero and atan2 flips in one frame,
// teleporting the eye to the far side of the car. A 20 rad/s cap on the yaw
// fixed that, but probed against the reference rig it is wrong: the
// reference applies the bearing LAGLESSLY (airborne flybys, over-the-ball
// passes and turning aerials all show 0 deg of lag), where the cap leaves
// the eye sideways for ~0.16 s on every pass over the ball - the "camera is
// not behind the car in the air" report. So the YAW now tracks the unwrapped
// measurement directly, exactly like the reference; the degenerate
// axis-crossing swing the cap used to soften is real Rocket League behavior
// (the reference itself moves the eye 516 uu / turns the view 75 deg in one
// frame through an overhead crossing - see the probe header).
//
// The PITCH keeps the cap: the elevation of a point above the horizon plane
// is single-valued (no atan2 flip), a fast-approaching ball genuinely needs
// the cap to keep the view from kicking in one frame, and the roof-dribble
// teleport (a ~150 deg pitch jump) must ease rather than snap.
static constexpr float kBallAimSlew = 20.0f;

// ---- ball-cam vertical aim + rig swing (decoded against the reference rig,
// settled poses; see the comment in computeBallCam):
//
// kAimKnee/kAimWidth: the view pitch stays parked while the ball rides up
    // to ~3 deg above the parked eye level, then tilts onto the ball over a
    // ~12 deg near-linear blend (reference decode, converted to eye-relative
    // elevations: parked while the ball sits 0.8-3.6 deg over the car (below
    // or just at eye level from the parked eye), ~4 deg off centre at 17.6
    // deg over the car (~9.9 deg from the parked eye), ~2 deg off at 30.3
    // (~23.6 from the eye) - i.e. fully tracking by ~15 deg eye-relative).
    // The width is a compromise the reference itself implies: a tighter
    // width centres the ball better through a fast graze but leans the view
    // further off the car during a roof launch (reference holds the car at
    // ~20 deg off-axis there; 12 deg of width keeps it ~23).
    // One-sided: for a ball BELOW eye level the reference stays parked (the
    // ball just rides below centre).
    static constexpr float kAimKnee = 3.0f * (float)M_PI / 180.0f;
    static constexpr float kAimWidth = 12.0f * (float)M_PI / 180.0f;
// kSwingElevMin: the swung eye's elevation floor for low pitches; below the
// aim-blend the eye may mirror the view pitch (see computeBallCam) so the
// eye-car-ball line stays straight at steep looks.
static constexpr float kSwingElevMin = -5.0f * (float)M_PI / 180.0f;
// kSwingFloorZ: the swung eye never goes below ~10 uu over the surface
// (reference pins the eye at z = 10 with the car centre at z = 17).
static constexpr float kSwingFloorZ = 10.0f;
// kRiseCap: the parked ball-cam height lift with an elevated ball is a few
// uu and saturates around 15 (reference settled eye heights 104-126 uu above
// the car centre across ball elevations 0-45 deg) - the large
// 0.5217*height*|E| lift an earlier decode assumed is NOT the reference's
// answer to a high ball; the aim deadband + rig swing above are.
static constexpr float kRiseCap = 15.0f;

// ---------------------------------------------------------------- eye placement
//
// Both rigs sit on a sphere of radius R = hypot(distance, height) at
// elevation e0 = atan2(height, distance) around the car: left/right swivel
// swings the eye around the car, and the pitch swivel swings it too (up/down
// rotates around the car on both axes the way Rocket League does, not a
// fixed eye with a tilting view). The view pitch is derived from wherever the
// eye actually ended up, which keeps the car at a constant screen offset for
// the whole swing. What the orbit cannot absorb comes back as a tilt -
// `outTilt` - so the control never goes dead.
//
// `heightRise` is the ball-cam's small parked-height lift with an elevated
// ball (measured against the reference rig: a few uu to ~15 at 45 deg - NOT
// the large lift an early decode assumed; the reference's real answer to a
// high ball is the vertical aim/swing in computeBallCam, not a taller park).
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
static constexpr float kMaxElev = 1.48353f; // rad (85 deg, reference eye-pitch limit): keep the eye behind the car (cos > 0)

void RLCamera::placeEye(const CameraSettings& cs, const SimSnapshot& s,
                        float lookYaw, V3& outEye, float& outTilt,
                        float heightRise) const {
    const float h = cs.height + heightRise;
    const float rigR = std::hypot(cs.distance, h);
    const float e0 = std::atan2(h, cs.distance);

    float elev = e0 - pitchOff_;

    // The stop is kEyeFloorZ *above the surface we are looking over*. On the
    // ground that surface is the one the car rests on; in the air it is the
    // arena floor (z = 0) or, on a loaded map, the last ground the car stood
    // on - loaded maps put their floors anywhere in z, and reading the stop as
    // an absolute arena height would throw the rig to its elevation limit
    // there and pitch the whole view at the sky.
    float surface;
    if (s.onGround) {
        surface = s.carPos.z - kCarRideHeight;
    } else if (s.hasMapFloor) {
        surface = std::min(s.floorZ, s.carPos.z);
    } else {
        surface = 0.0f;
    }
    const float minZ = surface + kEyeFloorZ;
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

    // ---- horizontal aim: keep the bearing on one continuous branch, and
    // track it LAGLESSLY (the reference applies the bearing with no cap -
    // probed: 0 deg of lag through airborne flybys, over-the-ball passes and
    // turning aerials, where a rate cap leaves the eye sideways for ~0.16 s
    // on every pass over the ball). measYaw_ unwraps the raw bearing frame
    // to frame (always the short way, so a far-side teleport still
    // approaches the short way round); the degenerate axis-crossing swing
    // that falls out of the atan2 flip is real Rocket League behavior (the
    // reference moves the eye 516 uu in one frame through an overhead
    // crossing - see the kBallAimSlew note).
    if (reliable) {
        const float phi = std::atan2(toBall.y, toBall.x);
        if (first) {
            ballYaw_ = measYaw_ = phi;  // reset/preset: anchor, don't ease in
            hasBallAim_ = true;
        } else {
            measYaw_ = hasPrevPhi_ ? measYaw_ + wrapAngle(phi - prevPhi_) : phi;
            ballYaw_ = measYaw_;
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

    // ---- parked pose: the eye behind the car at (distance, height + rise).
    // No distance multiplier (the reference rig sits exactly `distance`
    // behind), and the rise is the small parked lift the reference shows
    // with an elevated ball (see placeEye) - the reference's real answer to
    // a HIGH ball is the aim deadband + rig swing below, not a taller park.
    const float toBallHoriz = std::max(toBall.len2d(), 1.0f);
    const float ballElev = std::atan2(toBall.z, toBallHoriz);
    const float rise = std::fmin(0.5217f * cs.height * std::fabs(ballElev),
                                 kRiseCap);
    placeEye(cs, s, lookYaw, outEye, tilt, rise);

    // ---- ball elevation measured from the parked eye, slew-limited (the
    // slew is what keeps a pop or a fast pass from turning the view in one
    // frame; see kBallAimSlew).
    V3 toT = s.ballPos - outEye;
    const float measPitch = std::atan2(toT.z, std::max(toT.len2d(), 1.0f));
    if (first) ballPitch_ = measPitch;
    else ballPitch_ += clampf(measPitch - ballPitch_, -slewStep, slewStep);

    // ---- vertical aim: a PARKED view with a deadband, not a ball-tracking
    // one. Measured against the reference rig (refcam probes, settled):
    // for a ball up to ~8 deg above the parked eye level the view pitch sits
    // exactly at the parked angle (cs.angle + tilt) and the ball simply rides
    // above centre - which is what keeps the car resting low-but-in-frame
    // while a pass or a dribble ball is elevated. Past the knee the aim
    // tilts smoothly onto the ball (fully ball-centred by ~+33 deg, matching
    // the reference's offset shrinking to ~2 deg), and for a ball BELOW the
    // eye level the same deadband mirrors: the view tilts down onto it.
    // Tracking the ball exactly at all elevations (the old law) was what
    // pushed the car out of the bottom of the frame the moment the ball
    // launched off the roof.
    const float deg2rad = (float)M_PI / 180.0f;
    const float parkedPitch = cs.angle * deg2rad + tilt;
    const float ballAim = clampf(ballPitch_ + cs.angle * deg2rad + tilt,
                                 -kMaxElev, kMaxElev);
    // One-sided deadband (see kAimKnee/kAimWidth): parked below the knee,
    // then a near-linear blend onto the ball.
    float w = 0.0f;   // 0: parked, 1: tilted onto the ball
    if (ballPitch_ > kAimKnee)
        w = clampf((ballPitch_ - kAimKnee) / kAimWidth, 0.0f, 1.0f);
    float viewPitch = parkedPitch + w * (ballAim - parkedPitch);

    // Colinear rig swing: as the view tilts onto an elevated ball (past the
    // aim knee), the eye swings AROUND the car to mirror the view pitch, so
    // the eye-car-ball line stays straight and the car remains centred in
    // frame at ANY pitch - including a powered dive under a launched ball
    // (measured: car holds 1-3 deg off-axis through a 45 deg dive, vs
    // 50-65 deg with the old 0.45-ratio swing that stopped at -5 deg of
    // elevation). The floor stop below then slides the eye inward along the
    // bearing near the surface exactly as before.
    const V3 rel = outEye - s.carPos;
    const float distH = std::max(rel.len2d(), 1.0f);
    const float eParked = std::atan2(rel.z, distH);
    const float eTarget = clampf(eParked + w * (-viewPitch - eParked),
                                 std::min(kSwingElevMin, -viewPitch),
                                 kMaxElev);
    const float swing = eParked - eTarget;
    float hTarget = distH;
    float zTarget = s.carPos.z + hTarget * std::tan(eTarget);

    // Floor stop for the swung rig: the reference pins the eye ~10 uu above
    // the surface and, past that, slides IN along the bearing (radius
    // shrinks, e.g. behind 195..86 for balls 600..3000 uu overhead while the
    // elevation falls out to -2..-4.6 deg purely from the pin: the eye z is
    // held and the elevation is just atan2(pin - carZ, radius)). The radius
    // is bounded so the atan2 never blows up as the elevation crosses zero
    // and so the slide-in stops near the reference's ~0.3*parked radius.
    float surface;
    if (s.onGround) {
        surface = s.carPos.z - kCarRideHeight;
    } else if (s.hasMapFloor) {
        surface = std::min(s.floorZ, s.carPos.z);
    } else {
        surface = 0.0f;
    }
    const float swingMinZ = surface + kSwingFloorZ;
    if (zTarget < swingMinZ) {
        hTarget = clampf((swingMinZ - s.carPos.z) / std::min(std::tan(eTarget),
                                                             -1e-3f),
                         0.3f * distH, distH);
        zTarget = swingMinZ;
    }

    // Ease the actual swung pose toward the targets. The swing ANGLE eases
    // (a fast rate: it must track the slewed aim, only rounding its kinks),
    // while the RADIUS eases slowly - without that the pin hand-off
    // teleports: the frame the ball descends enough for the swing law to
    // leave the floor pin, the radius would jump 0.3*parked -> parked
    // (~190 uu) in one frame. During a pure manual swivel the angle target
    // is identically zero, so the easing never lags the orbit.
    if (!hasSwing_) {
        swingAng_ = swing;
        swingH_ = hTarget;
        hasSwing_ = true;
    } else {
        static constexpr float kSwingAngRate = 30.0f;  // rad/s (the mirrored
        // elevation must track the slewed aim: 12 left a visible whip-lag
        // during powered dives)
        static constexpr float kSwingHRate = 900.0f;   // uu/s
        swingAng_ += clampf(swing - swingAng_, -kSwingAngRate * dt,
                            kSwingAngRate * dt);
        swingH_ += clampf(hTarget - swingH_, -kSwingHRate * dt,
                          kSwingHRate * dt);
    }
    const float e = clampf(eParked - swingAng_,
                           std::min(kSwingElevMin, -viewPitch), kMaxElev);
    float swingH = swingH_;
    float swingZ = s.carPos.z + swingH * std::tan(e);
    // Never let the eased pose fall through the floor while easing.
    swingZ = std::fmax(swingZ, swingMinZ);

    const V3 fwd(std::cos(lookYaw), std::sin(lookYaw), 0);
    outEye = s.carPos - fwd * swingH + V3(0, 0, swingZ - s.carPos.z);

    // Ball cam's resting aim is the ball, but manual swivel must still orbit
    // the car. A ball-derived pitch leaves the car off-axis as soon as the
    // eye moves around it, which makes the camera appear to rotate in place.
    // Blend toward the same car-anchored aim used by car cam only while a
    // meaningful swivel offset is present, preserving the existing ball lock
    // with no stick input.
    toT = s.ballPos - outEye;
    V3 ballDir = dirFromYawPitch(lookYaw, viewPitch);
    if (ballOrbitAim_ > 0.0f) {
        const V3 carAim = s.carPos + V3(0, 0, cs.height * 0.25f);
        const V3 toCar = carAim - outEye;
        const float carBasePitch = std::atan2(toCar.z, std::max(toCar.len2d(), 1.0f));
        const float carPitch = clampf(
            carBasePitch + cs.angle * (float)M_PI / 180.0f + tilt,
            -kMaxElev, kMaxElev);
        const V3 carDir = dirFromYawPitch(lookYaw, carPitch);
        ballDir = V3::lerp(ballDir, carDir, ballOrbitAim_).norm();
    }

    V3 dir = ballDir;
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

    // Keep the car lower in frame so the view retains more of the playfield
    // above it, matching Rocket League's car-cam composition.
    V3 aim = s.carPos + V3(0, 0, cs.height * 0.75f);
    V3 toT = aim - outEye;
    float basePitch = std::atan2(toT.z, std::max(toT.len2d(), 1.0f));
    float pitch = clampf(basePitch + cs.angle * (float)M_PI / 180.0f + tilt,
                         -kMaxElev, kMaxElev);

    V3 dir = dirFromYawPitch(lookYaw, pitch);
    outTarget = outEye + dir * std::max(toT.len(), 1000.0f);
}

void RLCamera::update(float dt, const CameraSettings& cs, const SimSnapshot& s,
                      float swivelX, float swivelY) {
    // ---- swivel (persistent offsets) - reference-decoded law:
    // holding a stick input sways the camera out to a PLATEAU of
    // 1.0472|input| rad (= 59.99 deg, measured ±59.94 deg): the plateau is
    // ~linear in the input magnitude and independent of swivelSpeed.
    // It is approached exponentially at kHold ≈ 10.5·(ss/10)/s;
    // releasing (Snap Camera to Default) returns exponentially at
    // kRelease ≈ 34.5·(ss/10)/s (measured ss=10, dt=1/120).
    static constexpr float kSwivelPlateau = 1.0472f; // rad per full input
    const float invert = cs.invertSwivel ? -1.0f : 1.0f;
    const float sw = cs.swivelSpeed / 10.0f;
    const bool manual = std::fabs(swivelX) > 0.004f ||
                        std::fabs(swivelY) > 0.004f;
    if (manual) swivelIdle_ = 0;
    else swivelIdle_ += dt;
    const float kHold = 10.5f * sw;
    const float kRelease = 34.5f * sw;
    const float kNow = manual ? kHold : kRelease;
    const float yawTarget = manual ? kSwivelPlateau * swivelX : 0.0f;
    const float pitchTarget =
        manual ? kSwivelPlateau * swivelY * invert : 0.0f;
    const float kExp = 1.0f - std::exp(-kNow * dt);
    if (manual || (cs.snap && swivelIdle_ > 0.05f)) {
        yawOff_ += (yawTarget - yawOff_) * kExp;
        pitchOff_ += (pitchTarget - pitchOff_) * kExp;
    }
    if (yawOff_ > (float)M_PI) yawOff_ -= 2 * (float)M_PI;
    if (yawOff_ < -(float)M_PI) yawOff_ += 2 * (float)M_PI;
    pitchOff_ = clampf(pitchOff_, -0.8f, 0.7f);

    // Keep ball cam in its car-anchored orbit state briefly while the
    // snap-back is underway. Decaying this state independently of the raw
    // offsets prevents the aim from switching back to the ball in one frame
    // when the offsets reach zero.
    const bool manualSwivel = manual;
    const float orbitRate = manualSwivel ? 14.0f : 5.0f;
    const float orbitTarget = manualSwivel ? 1.0f : 0.0f;
    ballOrbitAim_ += (orbitTarget - ballOrbitAim_) *
                     (1.0f - std::exp(-orbitRate * dt));

    // ---- mode transition (reference-decoded law: consumed = 1 - lambda^2
    // with lambda = 1 - t/T, T = 0.5/clamp(transitionSpeed, 0.1, 10) s —
    // slow start, fast finish; the seconds-scale setting owns the duration).
    // A half-finished transition that gets reversed restarts from the
    // current blend (RL re-engages the capture, it does not rewind).
    float targetBlend = (desired_ == CamMode::Ball) ? 1.0f : 0.0f;
    if (std::fabs(targetBlend - blend_) <= 1e-6f) {
        transT_ = -1.0f;
    } else {
        const bool reversed = transT_ >= 0.0f &&
                              ((targetBlend - startBlend_) * (targetBlend - blend_)) < 0.0f;
        if (transT_ < 0.0f || reversed) {
            transT_ = 0.0f;
            startBlend_ = blend_;
        }
        transT_ += dt;
        const float T = 0.5f / clampf(cs.transitionSpeed, 0.1f, 10.0f);
        const float lam = clampf(1.0f - transT_ / T, 0.0f, 1.0f);
        blend_ = startBlend_ + (targetBlend - startBlend_) * (1.0f - lam * lam);
        if (lam <= 0.0f) {
            blend_ = targetBlend;
            transT_ = -1.0f;
        }
    }

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
    const float carVz = s.carVel.z;
    const float rigOffZ = wantEye.z - s.carPos.z;
    const float rigVz = (rigOffZ - prevRigOffZ_) / std::max(dt, 1e-4f);
    prevRigOffZ_ = rigOffZ;
    prevWantEye_ = wantEye;
    const float rateX = followRate + rigSpeed2d / kRigLagCap;
    // Vertical follow needs BOTH rates: the CAR's vertical speed (in a pure
    // fall the rig offset is constant, so the rig-offset rate reads ~0 while
    // the eye target plummets at 2300 uu/s - the follower settled
    // v/baseRate behind and the car slid out the bottom of the frame,
    // measured off-axis 175 deg vs the reference's ~24) AND the rig-offset
    // rate (a car-cam swivel pivot swings the eye around the car, so the
    // offset changes fast while the car itself stands still - without this
    // term the pivot lagged, measured travel 44 vs 82 uu). Summing them
    // mirrors the horizontal term and matches the reference probed through
    // a full freefall (car holds ~22-24 deg off-axis at every fall speed;
    // only the eye's own slow low-speed softness from baseRateZ survives).
    const float rateZ = baseRateZ +
        (std::fabs(carVz) + std::fabs(rigVz)) / kRigLagCap;
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
