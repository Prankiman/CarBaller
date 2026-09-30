#include "settings.h"

#include <nlohmann/json.hpp>
#include <fstream>

using nlohmann::json;

const char* actionName(Action a) {
    switch (a) {
        case Action::Throttle: return "Throttle";
        case Action::Brake: return "Brake / Reverse";
        case Action::SteerLeft: return "Steer Left";
        case Action::SteerRight: return "Steer Right";
        case Action::PitchDown: return "Pitch Down (stick forward)";
        case Action::PitchUp: return "Pitch Up (stick back)";
        case Action::YawLeft: return "Yaw Left (air)";
        case Action::YawRight: return "Yaw Right (air)";
        case Action::AirRollLeft: return "Air Roll Left";
        case Action::AirRollRight: return "Air Roll Right";
        case Action::AirRollFree: return "Air Roll (hold) + Powerslide";
        case Action::Powerslide: return "Powerslide";
        case Action::Boost: return "Boost";
        case Action::Jump: return "Jump";
        case Action::BallCam: return "Ball Cam";
        case Action::SwivelLeft: return "Camera Swivel Left";
        case Action::SwivelRight: return "Camera Swivel Right";
        case Action::SwivelUp: return "Camera Swivel Up";
        case Action::SwivelDown: return "Camera Swivel Down";
        case Action::LaunchBall: return "Launch Ball";
        case Action::Dribble: return "Start Dribble";
        case Action::TakePosition: return "Take Position";
        case Action::Pause: return "Pause / Menu";
        case Action::ToggleStats: return "Toggle Stats";
        default: return "?";
    }
}

// RL-flavored default bindings (Liquipedia defaults for KBM + Xbox pad).
Bindings Bindings::defaults() {
    Bindings b;
    using T = BindType;
    auto K = [](int sc) { return Binding{T::Key, sc, 1}; };
    auto M = [](int btn) { return Binding{T::Mouse, btn, 1}; };
    auto P = [](int btn) { return Binding{T::PadBtn, btn, 1}; };

    // SDL scancodes (SDL_SCANCODE_*)
    constexpr int SC_W = 26, SC_A = 4, SC_S = 22, SC_D = 7;
    constexpr int SC_Q = 20, SC_E = 8, SC_SHIFT = 225, SC_SPACE = 44;
    constexpr int SC_B = 5, SC_V = 25, SC_T = 23, SC_ESC = 41, SC_F3 = 60; // F3
    constexpr int SC_LEFT = 80, SC_RIGHT = 79, SC_UP = 82, SC_DOWN = 81;

    b.set(Action::Throttle, {K(SC_W), Binding{BindType::PadAxis, 5 /*RT*/, 1}});
    b.set(Action::Brake, {K(SC_S), Binding{BindType::PadAxis, 4 /*LT*/, 1}});
    b.set(Action::SteerLeft, {K(SC_A)});
    b.set(Action::SteerRight, {K(SC_D)});
    b.set(Action::PitchDown, {K(SC_W)});   // air: W pitches nose down (RL keyboard)
    b.set(Action::PitchUp, {K(SC_S)});
    b.set(Action::YawLeft, {K(SC_A)});
    b.set(Action::YawRight, {K(SC_D)});
    b.set(Action::AirRollLeft, {K(SC_Q), P(9 /*LB*/)});
    b.set(Action::AirRollRight, {K(SC_E), P(10 /*RB*/)});
    b.set(Action::AirRollFree, {K(SC_SHIFT), P(2 /*X*/)});
    b.set(Action::Powerslide, {K(SC_SHIFT), P(2 /*X*/)});
    b.set(Action::Boost, {M(1 /*LMB*/), P(1 /*B*/)});
    b.set(Action::Jump, {M(3 /*RMB*/), P(0 /*A*/)});
    b.set(Action::BallCam, {K(SC_SPACE), P(3 /*Y*/)});
    b.set(Action::SwivelLeft, {K(SC_LEFT)});
    b.set(Action::SwivelRight, {K(SC_RIGHT)});
    b.set(Action::SwivelUp, {K(SC_UP)});
    b.set(Action::SwivelDown, {K(SC_DOWN)});
    b.set(Action::LaunchBall, {K(SC_B), P(4 /*Back*/)});
    b.set(Action::Dribble, {K(SC_V), P(7 /*LS click*/)});
    b.set(Action::TakePosition, {K(SC_T), P(8 /*RS click*/)});
    b.set(Action::Pause, {K(SC_ESC), P(6 /*Start*/)});
    b.set(Action::ToggleStats, {K(SC_F3)});
    return b;
}

// ------------------------------------------------------------------ (de)serialize

static json bindingToJson(const Binding& b) {
    json j;
    switch (b.type) {
        case BindType::Key: j["t"] = "key"; break;
        case BindType::Mouse: j["t"] = "mouse"; break;
        case BindType::PadBtn: j["t"] = "pad"; break;
        case BindType::PadAxis: j["t"] = "axis"; break;
    }
    j["c"] = b.code;
    j["d"] = b.dir;
    return j;
}

static Binding bindingFromJson(const json& j) {
    Binding b;
    std::string t = j.value("t", "key");
    b.type = t == "mouse" ? BindType::Mouse : t == "pad" ? BindType::PadBtn
           : t == "axis" ? BindType::PadAxis : BindType::Key;
    b.code = j.value("c", 0);
    b.dir = j.value("d", 1);
    return b;
}

bool Settings::save(const std::string& path) const {
    json j;
    j["camera"] = {
        {"fov", cam.fov}, {"distance", cam.distance}, {"height", cam.height},
        {"angle", cam.angle}, {"stiffness", cam.stiffness},
        {"swivelSpeed", cam.swivelSpeed}, {"transitionSpeed", cam.transitionSpeed},
        {"shake", cam.shake}, {"invertSwivel", cam.invertSwivel},
        {"ballCamToggle", cam.ballCamToggle}, {"ballCamIndicator", cam.ballCamIndicator},
        {"ballArrow", cam.ballArrow}, {"ballFloorProjection", cam.ballFloorProjection},
        {"mouseSwivel", cam.mouseSwivel}, {"mouseSens", cam.mouseSens},
    };
    j["controls"] = {
        {"steerSens", ctrl.steerSens}, {"aerialSens", ctrl.aerialSens},
        {"deadzone", ctrl.deadzone}, {"dodgeDeadzone", ctrl.dodgeDeadzone},
        {"deadzoneShape", ctrl.deadzoneShape}, {"vibration", ctrl.vibration},
        {"keyboardSteerSmooth", ctrl.keyboardSteerSmooth},
    };
    j["freeplay"] = {
        {"launchSpeed", freeplay.launchSpeed}, {"launchAngle", freeplay.launchAngle},
        {"takePositionPreset", freeplay.takePositionPreset},
    };
    j["graphics"] = {
        {"vsync", gfx.vsync}, {"fpsCap", gfx.fpsCap},
        {"particleQuality", gfx.particleQuality}, {"msaa", gfx.msaa},
    };

    json jb;
    for (auto& [a, list] : binds.binds) {
        json arr = json::array();
        for (auto& b : list) arr.push_back(bindingToJson(b));
        jb[actionName(a)] = arr;
    }
    j["bindings"] = jb;

    std::ofstream f(path);
    if (!f) return false;
    f << j.dump(2);
    return true;
}

bool Settings::load(const std::string& path) {
    std::ifstream f(path);
    if (!f) return false;
    json j;
    try {
        f >> j;
    } catch (...) {
        return false;
    }

    if (j.contains("camera")) {
        auto& c = j["camera"];
        cam.fov = c.value("fov", cam.fov);
        cam.distance = c.value("distance", cam.distance);
        cam.height = c.value("height", cam.height);
        cam.angle = c.value("angle", cam.angle);
        cam.stiffness = c.value("stiffness", cam.stiffness);
        cam.swivelSpeed = c.value("swivelSpeed", cam.swivelSpeed);
        cam.transitionSpeed = c.value("transitionSpeed", cam.transitionSpeed);
        cam.shake = c.value("shake", cam.shake);
        cam.invertSwivel = c.value("invertSwivel", cam.invertSwivel);
        cam.ballCamToggle = c.value("ballCamToggle", cam.ballCamToggle);
        cam.ballCamIndicator = c.value("ballCamIndicator", cam.ballCamIndicator);
        cam.ballArrow = c.value("ballArrow", cam.ballArrow);
        cam.ballFloorProjection = c.value("ballFloorProjection", cam.ballFloorProjection);
        cam.mouseSwivel = c.value("mouseSwivel", cam.mouseSwivel);
        cam.mouseSens = c.value("mouseSens", cam.mouseSens);
    }
    if (j.contains("controls")) {
        auto& c = j["controls"];
        ctrl.steerSens = c.value("steerSens", ctrl.steerSens);
        ctrl.aerialSens = c.value("aerialSens", ctrl.aerialSens);
        ctrl.deadzone = c.value("deadzone", ctrl.deadzone);
        ctrl.dodgeDeadzone = c.value("dodgeDeadzone", ctrl.dodgeDeadzone);
        ctrl.deadzoneShape = c.value("deadzoneShape", ctrl.deadzoneShape);
        ctrl.vibration = c.value("vibration", ctrl.vibration);
        ctrl.keyboardSteerSmooth = c.value("keyboardSteerSmooth", ctrl.keyboardSteerSmooth);
    }
    if (j.contains("freeplay")) {
        auto& c = j["freeplay"];
        freeplay.launchSpeed = c.value("launchSpeed", freeplay.launchSpeed);
        freeplay.launchAngle = c.value("launchAngle", freeplay.launchAngle);
        freeplay.takePositionPreset = c.value("takePositionPreset", freeplay.takePositionPreset);
    }
    if (j.contains("graphics")) {
        auto& c = j["graphics"];
        gfx.vsync = c.value("vsync", gfx.vsync);
        gfx.fpsCap = c.value("fpsCap", gfx.fpsCap);
        gfx.particleQuality = c.value("particleQuality", gfx.particleQuality);
        gfx.msaa = c.value("msaa", gfx.msaa);
    }
    if (j.contains("bindings")) {
        auto def = Bindings::defaults();
        for (auto& [name, arr] : j["bindings"].items()) {
            // match by display name
            for (int i = 0; i < (int)Action::COUNT_; i++) {
                Action a = (Action)i;
                if (name == actionName(a)) {
                    BindList list;
                    for (auto& bj : arr) list.push_back(bindingFromJson(bj));
                    if (!list.empty()) binds.binds[a] = list;
                }
            }
        }
        // any action missing from file keeps defaults
        for (int i = 0; i < (int)Action::COUNT_; i++) {
            Action a = (Action)i;
            if (binds.binds.find(a) == binds.binds.end()) binds.binds[a] = def.get(a);
        }
    }
    return true;
}
