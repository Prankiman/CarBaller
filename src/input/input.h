#pragma once
// Input: keyboard + mouse + SDL game controllers (Xbox, PowerA, other 3rd party via
// SDL's GameController mapping), deadzones/sensitivities per RL semantics,
// rebind capture, and translation into RocketSim CarControls.

#include "../settings/settings.h"
#include "../core/math.h"

#include <SDL.h>
#include <string>

namespace RocketSim { struct CarControls; }  // RocketSim

class InputSystem {
public:
    void init(SDL_Window* window, const Settings* settings);
    void shutdown();

    void handleEvent(const SDL_Event& e);   // feed every event
    void update();                          // finalize state for this frame

    bool pressed(Action a) const;           // edge: went down this frame
    bool held(Action a) const;
    float value(Action a) const;            // 0..1 analog (keys=1)

    // Signed steer command in [-1,1] - the single source used for both
    // CarControls.steer and the front-wheel visual: keyboard keys unscaled,
    // bound stick directions x steer sensitivity, plus the left stick when
    // no binding has claimed its X axis.
    float steerCommand() const;

    // Camera swivel in [-1,1]: x = right, y = +1 = look up.
    // (non-const: consumes accumulated mouse deltas)
    void swivel(float& x, float& y);

    // Processed left stick (deadzone + sensitivity), for building CarControls
    void leftStick(float& x, float& y) const;
    void rawTrigger(float& rt, float& lt) const;

    // Fill RocketSim controls from current input.
    // airRollFreeHeld etc. are resolved internally from bindings.
    void buildControls(RocketSim::CarControls& out) const;

    void setBindings(const Bindings& b) { binds_ = b; }
    const Bindings& bindings() const { return binds_; }

    // Rebinding capture
    void beginCapture(Action a, int slot);   // slot = -1 -> append new
    void cancelCapture() { capture_ = false; }
    bool capturing() const { return capture_; }
    // True (once) when ESC was consumed to cancel an active rebind, so the
    // app can swallow the resulting Pause edge.
    bool consumeSwallowedEsc() {
        bool v = swallowedEsc_;
        swallowedEsc_ = false;
        return v;
    }
    Action captureAction() const { return captureAction_; }
    int captureSlot() const { return captureSlot_; }
    void clearBinding(Action a, int slot);

    static std::string describe(const Binding& b);

    void rumble(float strong, float weak, Uint32 ms);
    bool hasController() const { return !controllers_.empty(); }
    std::string controllerName() const;

private:
    void openController(int deviceIndex);
    float actionRaw(Action a) const;
    // Split an action's raw input into key/mouse/pad-button (kb) and pad-axis
    // (ax) parts; actionRaw = max(kb, ax), commands compose them signed.
    void rawParts(Action a, float& kb, float& ax) const;
    // True when any binding claims this SDL axis (sticks: claiming an axis
    // takes it over - its hardcoded default contribution is suppressed).
    bool axisBound(int axis) const;
    float applyDeadzone(float x, float y, float& ox, float& oy) const;

    const Settings* settings_ = nullptr;
    Bindings binds_ = Bindings::defaults();

    const Uint8* keys_ = nullptr;
    int keysLen_ = 0;
    bool mouseDown_[8] = {};
    float mouseDX_ = 0, mouseDY_ = 0;

    std::vector<SDL_GameController*> controllers_;

    // per-action state
    float value_[int(Action::COUNT_)] = {};
    float prevValue_[int(Action::COUNT_)] = {};

    bool capture_ = false;
    Action captureAction_ = Action::Jump;
    int captureSlot_ = -1;
    bool swallowedEsc_ = false;

    friend class InputSystemImpl;
};
