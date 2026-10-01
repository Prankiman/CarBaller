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

// Ranges/defaults below mirror Rocket League's settings menus:
//   camera + deadzone ranges per Liquipedia's Settings article;
//   distance/stiffness/deadzone defaults per Psyonix patch v1.74 (2020-03-10).
struct CameraSettings {
    float fov = 110.0f;            // 60..110 (RL), horizontal FOV
    float distance = 270.0f;       // 100..400 (RL; 270 = v1.74 default)
    float height = 100.0f;         // 40..200 (RL)
    float angle = -4.0f;           // -45..0 (RL; negative looks down)
    float stiffness = 0.5f;        // 0..1 (RL; 0.5 = v1.74 default)
    float swivelSpeed = 5.0f;      // 1..10 (RL)
    float transitionSpeed = 1.2f;  // 1..2 (RL)
    bool shake = true;             // RL ships with camera shake ON
    bool invertSwivel = false;
    bool snap = true;              // "Snap Camera to Default" after swivel idle
    bool ballCamToggle = true;     // true=toggle, false=hold ("Hold Ball Camera")
    bool ballCamIndicator = true;
    bool ballArrow = true;         // car->ball arrow while in car cam
    bool ballFloorProjection = true;
    bool mouseSwivel = true;
    float mouseSens = 10.0f;       // 1..100 (RL); 10 = our 0.016 rad/px scale
};

struct ControlSettings {
    float steerSens = 1.0f;        // 1..10 (RL; default 1.00)
    float aerialSens = 1.0f;       // 1..10 (RL; default 1.00)
    float deadzone = 0.20f;        // 0..0.75 (RL; 0.20 = v1.74 default)
    float dodgeDeadzone = 0.80f;   // 0.10..1.00 (RL; 0.80 = v1.74 default)
    int deadzoneShape = 0;         // 0 = cross, 1 = circle
    bool vibration = true;         // RL ships with controller vibration ON
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
    float wallOpacity = 0.30f;     // 0..1 walls/ceiling transparency (always on)
    bool showHitboxes = false;     // outline the car/ball physics hitboxes
};

struct SoundSettings {
    float volume = 0.7f;        // 0..1 master SFX volume
};

struct Settings {
    CameraSettings cam;
    ControlSettings ctrl;
    FreeplaySettings freeplay;
    GraphicsSettings gfx;
    SoundSettings sound;
    Bindings binds;

    bool load(const std::string& path);
    bool save(const std::string& path) const;
};
