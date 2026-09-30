#pragma once
// All user-configurable settings + JSON persistence.
// Ranges/defaults mirror Rocket League's settings menus.

#include <string>
#include <vector>
#include <map>

// ---------------------------------------------------------------- bindings

enum class Action {
    Throttle, Brake, SteerLeft, SteerRight,
    PitchDown, PitchUp,          // pitchDown = nose down (stick forward)
    YawLeft, YawRight,
    AirRollLeft, AirRollRight, AirRollFree,
    Powerslide, Boost, Jump, BallCam,
    SwivelLeft, SwivelRight, SwivelUp, SwivelDown,
    LaunchBall, Dribble, TakePosition,
    Pause, ToggleStats,
    COUNT_
};

const char* actionName(Action a);

enum class BindType { Key, Mouse, PadBtn, PadAxis };

struct Binding {
    BindType type = BindType::Key;
    int code = 0;   // SDL scancode / mouse button / SDL_GameControllerButton / SDL_GameControllerAxis
    int dir = 1;    // +1/-1 (PadAxis only: trigger direction contribution)
    bool operator==(const Binding& o) const { return type == o.type && code == o.code && dir == o.dir; }
};

using BindList = std::vector<Binding>;

struct Bindings {
    std::map<Action, BindList> binds;

    static Bindings defaults();
    void set(Action a, BindList list) { binds[a] = std::move(list); }
    // NB: was `return binds[a];` which cannot compile in a const method
    // (std::map::operator[] is non-const) and broke every TU including this
    // header. Same behavior, const-correct: missing actions yield an empty list.
    const BindList& get(Action a) const {
        auto it = binds.find(a);
        static const BindList empty;
        return it == binds.end() ? empty : it->second;
    }
};

// ---------------------------------------------------------------- settings

struct CameraSettings {
    float fov = 110.0f;            // 90..110 (horizontal), RL default display 110
    float distance = 270.0f;       // 230..400
    float height = 100.0f;         // 40..200
    float angle = -4.0f;           // -15..15 (deg)
    float stiffness = 0.5f;        // 0..1
    float swivelSpeed = 5.0f;      // 0..10
    float transitionSpeed = 1.2f;  // 0..2
    bool shake = false;
    bool invertSwivel = false;
    bool ballCamToggle = true;     // true=toggle, false=hold
    bool ballCamIndicator = true;
    bool ballArrow = true;         // car->ball arrow while in car cam
    bool ballFloorProjection = true;
    bool mouseSwivel = true;
    float mouseSens = 1.0f;
};

struct ControlSettings {
    float steerSens = 1.2f;        // 0..2
    float aerialSens = 1.2f;       // 0..2
    float deadzone = 0.10f;        // 0..0.7
    float dodgeDeadzone = 0.70f;   // 0..0.9 -> CarConfig.dodgeDeadzone
    int deadzoneShape = 0;         // 0 = cross, 1 = circle
    bool vibration = false;
    float keyboardSteerSmooth = 0.0f; // 0 = instant digital (RL-like)
};

struct FreeplaySettings {
    float launchSpeed = 2500.0f;   // uu/s
    float launchAngle = 10.0f;     // degrees upward
    int takePositionPreset = 0;    // index into presets
};

struct GraphicsSettings {
    bool vsync = true;
    int fpsCap = 0;                // 0 = unlimited
    int particleQuality = 1;       // 0 low, 1 med, 2 high
    int msaa = 0;                  // 0/4/8
};

struct Settings {
    CameraSettings cam;
    ControlSettings ctrl;
    FreeplaySettings freeplay;
    GraphicsSettings gfx;
    Bindings binds;

    bool load(const std::string& path);
    bool save(const std::string& path) const;
};
