#include "input.h"

#include "RocketSim/src/Sim/CarControls.h"

#include <cstring>

using RocketSim::CarControls;

void InputSystem::init(SDL_Window* window, const Settings* settings) {
    settings_ = settings;
    binds_ = settings->binds;
    keys_ = SDL_GetKeyboardState(&keysLen_);
    SDL_SetRelativeMouseMode(SDL_FALSE);

    int num = SDL_NumJoysticks();
    for (int i = 0; i < num; i++) openController(i);
}

void InputSystem::shutdown() {
    for (auto* c : controllers_) SDL_GameControllerClose(c);
    controllers_.clear();
}

void InputSystem::openController(int deviceIndex) {
    if (!SDL_IsGameController(deviceIndex)) return;
    SDL_GameController* gc = SDL_GameControllerOpen(deviceIndex);
    if (gc && std::find(controllers_.begin(), controllers_.end(), gc) == controllers_.end())
        controllers_.push_back(gc);
}

std::string InputSystem::controllerName() const {
    if (controllers_.empty()) return "";
    const char* n = SDL_GameControllerName(controllers_[0]);
    return n ? n : "";
}

// ---------------------------------------------------------------- events

void InputSystem::handleEvent(const SDL_Event& e) {
    switch (e.type) {
        case SDL_MOUSEMOTION:
            if (!capture_) {
                mouseDX_ += (float)e.motion.xrel;
                mouseDY_ += (float)e.motion.yrel;
            }
            break;

        case SDL_CONTROLLERDEVICEADDED:
            openController(e.cdevice.which);
            break;

        case SDL_CONTROLLERDEVICEREMOVED: {
            SDL_GameController* gc = SDL_GameControllerFromInstanceID(e.cdevice.which);
            auto it = std::find(controllers_.begin(), controllers_.end(), gc);
            if (it != controllers_.end()) {
                SDL_GameControllerClose(*it);
                controllers_.erase(it);
            }
            break;
        }

        default: break;
    }

    if (!capture_) return;

    // Capture next input as a binding
    auto finish = [&](Binding b) {
        if (b.type == BindType::PadAxis) {
            // same axis+direction only (a stick's opposite direction is a
            // distinct binding); never compare against the slot being
            // replaced - that used to abort the capture whenever a trigger
            // was re-bound to itself, leaving the capture stuck.
            BindList& list = binds_.binds[captureAction_];
            for (int i = 0; i < (int)list.size(); i++) {
                if (i == captureSlot_) continue;
                const Binding& x = list[i];
                if (x.type == BindType::PadAxis && x.code == b.code && x.dir == b.dir)
                    return;
            }
        }
        if (captureSlot_ >= 0) {
            BindList& list = binds_.binds[captureAction_];
            if (captureSlot_ < (int)list.size()) {
                list[captureSlot_] = b;
                capture_ = false;
                return;
            }
        }
        binds_.binds[captureAction_].push_back(b);
        capture_ = false;
    };

    if (e.type == SDL_KEYDOWN) {
        if (e.key.keysym.scancode == SDL_SCANCODE_ESCAPE) {
            capture_ = false;
            swallowedEsc_ = true;
            return;
        }
        finish({BindType::Key, e.key.keysym.scancode, 1});
    } else if (e.type == SDL_MOUSEBUTTONDOWN) {
        finish({BindType::Mouse, e.button.button, 1});
    } else if (e.type == SDL_CONTROLLERBUTTONDOWN) {
        SDL_GameController* gc = SDL_GameControllerFromInstanceID(e.cbutton.which);
        if (gc && std::find(controllers_.begin(), controllers_.end(), gc) != controllers_.end())
            finish({BindType::PadBtn, (int)e.cbutton.button, 1});
    } else if (e.type == SDL_CONTROLLERAXISMOTION) {
        SDL_GameController* gc = SDL_GameControllerFromInstanceID(e.caxis.which);
        if (!gc || std::find(controllers_.begin(), controllers_.end(), gc) == controllers_.end()) return;
        const int ax = (int)e.caxis.axis;
        if (ax == SDL_CONTROLLER_AXIS_TRIGGERLEFT || ax == SDL_CONTROLLER_AXIS_TRIGGERRIGHT) {
            if (e.caxis.value > 16000) finish({BindType::PadAxis, ax, 1});
        } else if (ax == SDL_CONTROLLER_AXIS_LEFTX || ax == SDL_CONTROLLER_AXIS_LEFTY ||
                   ax == SDL_CONTROLLER_AXIS_RIGHTX || ax == SDL_CONTROLLER_AXIS_RIGHTY) {
            // stick directions: capture the side the stick was pushed to
            if (e.caxis.value > 16000 || e.caxis.value < -16000)
                finish({BindType::PadAxis, ax, e.caxis.value < 0 ? -1 : 1});
        }
    }
}

void InputSystem::beginCapture(Action a, int slot) {
    capture_ = true;
    captureAction_ = a;
    captureSlot_ = slot;
    mouseDX_ = mouseDY_ = 0;
}

void InputSystem::clearBinding(Action a, int slot) {
    BindList& list = binds_.binds[a];
    if (slot >= 0 && slot < (int)list.size()) list.erase(list.begin() + slot);
}

// ---------------------------------------------------------------- state

void InputSystem::rawParts(Action a, float& kb, float& ax) const {
    const BindList& list = binds_.get(a);
    kb = 0;
    ax = 0;
    for (const Binding& b : list) {
        switch (b.type) {
            case BindType::Key:
                if (keys_ && b.code < keysLen_ && keys_[b.code]) kb = 1.0f;
                break;
            case BindType::Mouse:
                if (b.code >= 0 && b.code < 8 && mouseDown_[b.code]) kb = 1.0f;
                break;
            case BindType::PadBtn:
                for (auto* gc : controllers_)
                    if (SDL_GameControllerGetButton(gc, (SDL_GameControllerButton)b.code)) {
                        kb = 1.0f;
                        break;
                    }
                break;
            case BindType::PadAxis: {
                const int axis = b.code;
                const bool isStick =
                    axis == SDL_CONTROLLER_AXIS_LEFTX || axis == SDL_CONTROLLER_AXIS_LEFTY ||
                    axis == SDL_CONTROLLER_AXIS_RIGHTX || axis == SDL_CONTROLLER_AXIS_RIGHTY;
                for (auto* gc : controllers_) {
                    float raw = SDL_GameControllerGetAxis(gc, (SDL_GameControllerAxis)axis) /
                                32767.0f;
                    float v;
                    if (isStick) {
                        // sticks get the per-axis deadzone, then resolve toward
                        // the captured direction (b.dir) - triggers stay raw
                        float ox, oy;
                        applyDeadzone(raw, 0.0f, ox, oy);
                        v = ox * (float)b.dir;
                    } else {
                        v = raw * (float)b.dir;
                    }
                    if (v > ax) ax = v;   // signed: opposite push reads negative
                }
                break;
            }
        }
    }
}

float InputSystem::actionRaw(Action a) const {
    float kb, ax;
    rawParts(a, kb, ax);
    return std::max(kb, ax);   // negatives (wrong-way stick) never count as held
}

bool InputSystem::axisBound(int axis) const {
    for (int i = 0; i < (int)Action::COUNT_; i++)
        for (const Binding& b : binds_.get(Action(i)))
            if (b.type == BindType::PadAxis && b.code == axis) return true;
    return false;
}

float InputSystem::applyDeadzone(float x, float y, float& ox, float& oy) const {
    float dz = settings_ ? settings_->ctrl.deadzone : 0.1f;
    int shape = settings_ ? settings_->ctrl.deadzoneShape : 0;
    if (shape == 1) {
        // circle / radial
        float l = std::sqrt(x * x + y * y);
        if (l <= dz) { ox = oy = 0; return 0; }
        float nl = (l - dz) / (1.0f - dz);
        nl = clampf(nl, 0, 1);
        ox = x / l * nl;
        oy = y / l * nl;
        return nl;
    }
    // cross (per-axis), RL community standard
    auto dz1 = [&](float v) {
        float a = std::fabs(v);
        if (a <= dz) return 0.0f;
        float n = (a - dz) / (1.0f - dz);
        return std::copysign(std::min(n, 1.0f), v);
    };
    ox = dz1(x);
    oy = dz1(y);
    return std::sqrt(ox * ox + oy * oy);
}

void InputSystem::update() {
    std::memcpy(prevValue_, value_, sizeof(value_));

    Uint32 mouseState = SDL_GetMouseState(nullptr, nullptr);
    for (int i = 0; i < 8; i++) mouseDown_[i] = (mouseState & SDL_BUTTON(i + 1)) != 0;

    for (int i = 0; i < int(Action::COUNT_); i++) {
        float v = actionRaw(Action(i));
        value_[i] = clampf(v, 0, 1);
    }
}

bool InputSystem::pressed(Action a) const {
    int i = int(a);
    return value_[i] > 0.5f && prevValue_[i] <= 0.5f;
}
bool InputSystem::held(Action a) const { return value_[int(a)] > 0.5f; }
float InputSystem::value(Action a) const { return value_[int(a)]; }

void InputSystem::leftStick(float& x, float& y) const {
    float sx = 0, sy = 0;
    for (auto* gc : controllers_) {
        float ax = SDL_GameControllerGetAxis(gc, SDL_CONTROLLER_AXIS_LEFTX) / 32767.0f;
        float ay = SDL_GameControllerGetAxis(gc, SDL_CONTROLLER_AXIS_LEFTY) / 32767.0f;
        if (std::fabs(ax) > std::fabs(sx)) sx = ax;
        if (std::fabs(ay) > std::fabs(sy)) sy = ay;
    }
    applyDeadzone(sx, sy, x, y);
}

void InputSystem::rawTrigger(float& rt, float& lt) const {
    rt = lt = 0;
    for (auto* gc : controllers_) {
        float r = SDL_GameControllerGetAxis(gc, SDL_CONTROLLER_AXIS_TRIGGERRIGHT) / 32767.0f;
        float l = SDL_GameControllerGetAxis(gc, SDL_CONTROLLER_AXIS_TRIGGERLEFT) / 32767.0f;
        if (r > rt) rt = r;
        if (l > lt) lt = l;
    }
}

void InputSystem::swivel(float& x, float& y) {
    x = 0;
    y = 0;
    // right stick
    float sx = 0, sy = 0;
    for (auto* gc : controllers_) {
        float ax = SDL_GameControllerGetAxis(gc, SDL_CONTROLLER_AXIS_RIGHTX) / 32767.0f;
        float ay = SDL_GameControllerGetAxis(gc, SDL_CONTROLLER_AXIS_RIGHTY) / 32767.0f;
        if (std::fabs(ax) > std::fabs(sx)) sx = ax;
        if (std::fabs(ay) > std::fabs(sy)) sy = ay;
    }
    float dx, dy;
    applyDeadzone(sx, sy, dx, dy);
    // right stick is hardcoded swivel unless a binding claimed its axes
    if (!axisBound(SDL_CONTROLLER_AXIS_RIGHTX)) x += dx;
    if (!axisBound(SDL_CONTROLLER_AXIS_RIGHTY)) y += -dy;   // stick up (+) = look up

    // keys
    x += value_[int(Action::SwivelRight)] - value_[int(Action::SwivelLeft)];
    y += value_[int(Action::SwivelUp)] - value_[int(Action::SwivelDown)];

    // mouse (consumed); mouseSens 1..100 (RL range), 10 == 0.016 rad/px
    if (settings_ && settings_->cam.mouseSwivel) {
        x += clampf(mouseDX_, -80, 80) * 0.0016f * settings_->cam.mouseSens;
        y += clampf(-mouseDY_, -80, 80) * 0.0016f * settings_->cam.mouseSens;
    }
    mouseDX_ = mouseDY_ = 0;
    x = clampf(x, -1, 1);
    y = clampf(y, -1, 1);
}

float InputSystem::steerCommand() const {
    const ControlSettings& cs = settings_->ctrl;
    float kbR, axR, kbL, axL;
    rawParts(Action::SteerRight, kbR, axR);
    rawParts(Action::SteerLeft, kbL, axL);
    float sx, sy;
    leftStick(sx, sy);                      // sy unused here
    if (axisBound(SDL_CONTROLLER_AXIS_LEFTX)) sx = 0;   // stick claimed by a binding
    return clampf((kbR - kbL) + (axR - axL + sx) * cs.steerSens, -1, 1);
}

void InputSystem::buildControls(RocketSim::CarControls& out) const {
    const ControlSettings& cs = settings_->ctrl;

    // Left stick (deadzoned): hardcoded fallback, suppressed per axis when a
    // binding has claimed it - the binding's directions take over that axis.
    float sx, sy;
    leftStick(sx, sy);  // x right, y down
    if (axisBound(SDL_CONTROLLER_AXIS_LEFTX)) sx = 0;
    if (axisBound(SDL_CONTROLLER_AXIS_LEFTY)) sy = 0;

    float kbR, axR, kbL, axL;
    rawParts(Action::PitchUp, kbR, axR);
    rawParts(Action::PitchDown, kbL, axL);
    const float pitchKey = kbR - kbL, pitchAx = axR - axL;
    rawParts(Action::YawRight, kbR, axR);
    rawParts(Action::YawLeft, kbL, axL);
    const float yawKey = kbR - kbL, yawAx = axR - axL;
    rawParts(Action::AirRollRight, kbR, axR);
    rawParts(Action::AirRollLeft, kbL, axL);
    const float rollDir = (kbR - kbL) + (axR - axL);

    const bool airRollFree = held(Action::AirRollFree);

    out.steer = steerCommand();
    // SDL stick down (+y) = pulled back = nose UP = positive pitch (matches
    // RocketSim: pitch<0 dodges forward, so stick-forward => front flip).
    out.pitch = clampf(pitchKey + (pitchAx + sy) * cs.aerialSens, -1, 1);

    if (airRollFree) {
        // free air roll: horizontal input becomes roll, yaw suppressed
        out.yaw = 0;
        out.roll = clampf(rollDir + yawKey + (yawAx + sx) * cs.aerialSens, -1, 1);
    } else {
        out.yaw = clampf(yawKey + (yawAx + sx) * cs.aerialSens, -1, 1);
        out.roll = clampf(rollDir, -1, 1);
    }

    float thr = value_[int(Action::Throttle)];
    float brk = value_[int(Action::Brake)];
    out.throttle = clampf(thr - brk, -1, 1);
    out.jump = value_[int(Action::Jump)] > 0.5f;
    out.boost = value_[int(Action::Boost)] > 0.5f;
    out.handbrake = value_[int(Action::Powerslide)] > 0.5f;
    out.ClampFix();
}

void InputSystem::rumble(float strong, float weak, Uint32 ms) {
    if (!settings_ || !settings_->ctrl.vibration) return;
    for (auto* gc : controllers_)
        SDL_GameControllerRumble(gc, (Uint16)(clampf(strong, 0, 1) * 0xFFFF),
                                 (Uint16)(clampf(weak, 0, 1) * 0xFFFF), ms);
}

std::string InputSystem::describe(const Binding& b) {
    switch (b.type) {
        case BindType::Key: {
            const char* n = SDL_GetScancodeName((SDL_Scancode)b.code);
            return n && *n ? n : "Key?";
        }
        case BindType::Mouse:
            switch (b.code) {
                case 1: return "Mouse L";
                case 2: return "Mouse M";
                case 3: return "Mouse R";
                case 4: return "Mouse X1";
                case 5: return "Mouse X2";
                default: return "Mouse ?";
            }
        case BindType::PadBtn:
            switch ((SDL_GameControllerButton)b.code) {
                case SDL_CONTROLLER_BUTTON_A: return "Pad A";
                case SDL_CONTROLLER_BUTTON_B: return "Pad B";
                case SDL_CONTROLLER_BUTTON_X: return "Pad X";
                case SDL_CONTROLLER_BUTTON_Y: return "Pad Y";
                case SDL_CONTROLLER_BUTTON_BACK: return "Pad Back";
                case SDL_CONTROLLER_BUTTON_START: return "Pad Start";
                case SDL_CONTROLLER_BUTTON_LEFTSHOULDER: return "Pad LB";
                case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER: return "Pad RB";
                case SDL_CONTROLLER_BUTTON_LEFTSTICK: return "Pad LS";
                case SDL_CONTROLLER_BUTTON_RIGHTSTICK: return "Pad RS";
                case SDL_CONTROLLER_BUTTON_DPAD_UP: return "D-Pad Up";
                case SDL_CONTROLLER_BUTTON_DPAD_DOWN: return "D-Pad Down";
                case SDL_CONTROLLER_BUTTON_DPAD_LEFT: return "D-Pad Left";
                case SDL_CONTROLLER_BUTTON_DPAD_RIGHT: return "D-Pad Right";
                default: return "Pad ?";
            }
        case BindType::PadAxis:
            switch (b.code) {
                case SDL_CONTROLLER_AXIS_LEFTX:
                    return b.dir < 0 ? "Pad LS Left" : "Pad LS Right";
                case SDL_CONTROLLER_AXIS_LEFTY:
                    return b.dir < 0 ? "Pad LS Up" : "Pad LS Down";
                case SDL_CONTROLLER_AXIS_RIGHTX:
                    return b.dir < 0 ? "Pad RS Left" : "Pad RS Right";
                case SDL_CONTROLLER_AXIS_RIGHTY:
                    return b.dir < 0 ? "Pad RS Up" : "Pad RS Down";
                case SDL_CONTROLLER_AXIS_TRIGGERRIGHT: return "Pad RT";
                default: return "Pad LT";
            }
    }
    return "?";
}
