// Rocket League-style in-game settings menu (ImGui).
// Tabs: Camera / Controls / Freeplay / Graphics / Sound / Bindings / Reset.

#include "settings_ui.h"

#include <imgui.h>

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>

namespace {

// ------------------------------------------------------------------------
// RL-style widget helpers. Each one flips *changed (bool&) when the user
// edits a value, so draw() can report "something changed this frame".
// ------------------------------------------------------------------------

// Right-clicking a slider opens this popup to type the exact value in
// instead of dragging to it. Enter applies it and closes; Esc closes it and
// throws the edit away, which means it has to beat ImGui at its own game (see
// below). Text that isn't a number is ignored - the setting keeps the value
// it had - and a number past either end snaps to the end the slider would
// have clamped to anyway, so the typed path and the dragging path always
// agree on the range.
// (Ctrl+click/double-click already reach ImGui's own temp input; this is
// the mouse-only way round.)
//
// Esc is caught here, before the field sees it. ImGui's own Esc routing
// (imgui_widgets.cpp, InputText) never closes a popup that owns a text
// field: it rewrites the buffer back to what was there when focus was taken,
// which trips IsItemDeactivatedAfterEdit() and hands that reverted text to
// the setting - the opposite of cancelling. EscapeClearsAll suppresses the
// revert, so the field stays inert and the cancel below is the only thing
// that acts on the key.
//
// Clicking away is deliberately not a third way out. It is genuinely
// inconsistent: a click inside the settings window lands on the window that
// owns the popup, so ImGui leaves the popup up, while a click outside
// dismisses it - and the deactivation either way used to apply the value.
// Promising three ways to close something that closes reliably in one has
// cost more than it saves, so the hint below names Enter and Esc only.
template <typename T, typename Parse>
bool typedValuePopup(const char* id, T* v, T lo, T hi, Parse parse,
                     bool& entryOpen) {
    static char buf[32];
    if (ImGui::IsItemClicked(ImGuiMouseButton_Right)) {
        std::snprintf(buf, sizeof buf, "%.6g", double(*v));  // seed, plain digits
        ImGui::OpenPopup(id);
    }
    if (!ImGui::BeginPopup(id)) return false;

    // Esc before the field: also the frame the popup is raised, so a right
    // click followed by an Esc closes it rather than leaving it stranded.
    const bool cancel = ImGui::IsKeyPressed(ImGuiKey_Escape);
    // Only once BeginPopup() said yes, so other popups drawn the same frame
    // cannot raise the flag. Cleared on close (below) rather than at the top
    // of draw(): the menu's Escape toggle reads it *before* draw() runs, so
    // resetting there would give the popup a dead Escape on its first frame.
    entryOpen = true;

    bool applied = false;
    bool close = cancel;
    if (!cancel) {
        ImGui::SetKeyboardFocusHere();
        ImGui::SetNextItemWidth(120.0f);
        const bool enter =
            ImGui::InputText("##value", buf, sizeof buf,
                             ImGuiInputTextFlags_EnterReturnsTrue |
                                 ImGuiInputTextFlags_CharsDecimal |
                                 ImGuiInputTextFlags_AutoSelectAll |
                                 ImGuiInputTextFlags_EscapeClearsAll);
        if (enter) {
            char* end = nullptr;
            const double x = parse(buf, &end);
            if (end != buf && std::isfinite(x)) {  // "-"/""/garbage: leave it alone
                const T clamped = (x < double(lo)) ? lo
                                                   : (x > double(hi)) ? hi : T(x);
                if (clamped != *v) {
                    *v = clamped;
                    applied = true;
                }
            }
            close = true;
        }
    }
    if (close) {
        entryOpen = false;  // this frame's answer, read by app.cpp
        ImGui::CloseCurrentPopup();
    }
    ImGui::TextDisabled("Enter = apply, Esc = cancel");
    ImGui::EndPopup();
    return applied;
}

inline bool typedValuePopup(const char* id, float* v, float lo, float hi,
                            bool& entryOpen) {
    return typedValuePopup(
        id, v, lo, hi,
        [](const char* s, char** e) { return std::strtod(s, e); }, entryOpen);
}
inline bool typedValuePopup(const char* id, int* v, int lo, int hi,
                            bool& entryOpen) {
    return typedValuePopup(
        id, v, lo, hi,
        [](const char* s, char** e) { return double(std::strtol(s, e, 10)); },
        entryOpen);
}

// Label on the left, slider spanning the rest of the row.
void sliderF(const char* label, float* v, float lo, float hi, const char* fmt,
             bool& changed, bool& entryOpen, const char* tip = nullptr) {
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(label);
    ImGui::SameLine();
    ImGui::PushItemWidth(-FLT_MIN);
    const std::string id = std::string("##") + label;
    if (ImGui::SliderFloat(id.c_str(), v, lo, hi, fmt, ImGuiSliderFlags_AlwaysClamp))
        changed = true;
    // Tip first: the popup below replaces the tooltip's "last item".
    if (tip)
        ImGui::SetItemTooltip("%s\n\nRight-click the slider to type the value in.",
                              tip);
    if (typedValuePopup(id.c_str(), v, lo, hi, entryOpen)) changed = true;
    ImGui::PopItemWidth();
}

void check(const char* label, bool* v, bool& changed, const char* tip = nullptr) {
    if (ImGui::Checkbox(label, v)) changed = true;
    if (tip) ImGui::SetItemTooltip("%s", tip);
}

// Label on the left, combo spanning the rest of the row; picks an index.
void comboIndex(const char* label, int* v, int count, const char* const* items,
                bool& changed, const char* tip = nullptr) {
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(label);
    ImGui::SameLine();
    const std::string id = std::string("##") + label;
    const int cur = (*v >= 0 && *v < count) ? *v : 0;
    ImGui::PushItemWidth(-FLT_MIN);
    if (ImGui::BeginCombo(id.c_str(), items[cur])) {
        for (int i = 0; i < count; i++) {
            if (ImGui::Selectable(items[i], i == *v)) {
                *v = i;
                changed = true;
            }
        }
        ImGui::EndCombo();
    }
    ImGui::PopItemWidth();
    if (tip) ImGui::SetItemTooltip("%s", tip);
}

// Highlighted "waiting for input" button shown in place of a binding while
// InputSystem is capturing; the InputSystem consumes the event itself.
void capturePlaceholder() {
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.13f, 0.42f, 0.72f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.17f, 0.50f, 0.84f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.10f, 0.34f, 0.60f, 1.0f));
    ImGui::Button("... press a key / mouse / pad button or stick direction (Esc cancels)");
    ImGui::PopStyleColor(3);
}

// ------------------------------------------------------------------------
// Tabs
// ------------------------------------------------------------------------

void drawCameraTab(CameraSettings& c, bool& changed, bool& entryOpen) {
    // RL order (Liquipedia/Epic): shake first, then FOV, Height, Angle,
    // Distance, Stiffness, Swivel Speed, Transition Speed.
    check("Camera Shake", &c.shake, changed,
          "Shake the camera on impacts. Rocket League ships with this ON.");
    check("Invert Swivel", &c.invertSwivel, changed,
          "Invert the vertical camera swivel direction.");
    check("Snap Camera to Default", &c.snap, changed,
          "After you stop swiveling, the camera eases back to its default "
          "angle (Rocket League's snap behavior). Off keeps the swivel where "
          "you left it.");

    sliderF("FOV", &c.fov, 60.0f, 110.0f, "%.0f", changed, entryOpen,
            "Horizontal field of view. Rocket League: 60 - 110 (default 110).");
    sliderF("Height", &c.height, 40.0f, 200.0f, "%.0f", changed, entryOpen,
            "How high above the car the camera floats. Rocket League: 40 - 200 (default 100).");
    sliderF("Angle", &c.angle, -45.0f, 0.0f, "%.0f", changed, entryOpen,
            "Camera pitch relative to the car; negative looks down. Rocket League: -45 - 0 (default -4).");
    sliderF("Distance", &c.distance, 100.0f, 400.0f, "%.0f", changed, entryOpen,
            "How far the camera sits behind the car. Rocket League: 100 - 400 (default 270).");
    sliderF("Stiffness", &c.stiffness, 0.0f, 1.0f, "%.2f", changed, entryOpen,
            "How rigidly the camera follows the car. Higher = tighter, lower = floatier. Rocket League: 0 - 1 (default 0.50).");
    sliderF("Swivel Speed", &c.swivelSpeed, 1.0f, 10.0f, "%.1f", changed, entryOpen,
            "How fast the camera swivels when you look around. Rocket League: 1 - 10.");
    sliderF("Transition Speed", &c.transitionSpeed, 1.0f, 2.0f, "%.2f", changed, entryOpen,
            "How quickly the camera blends between car cam and ball cam. Rocket League: 1 - 2.");

    ImGui::SeparatorText("Options");

    // RL's wording is "Hold Ball Camera" (checked = hold); we store toggle-first.
    bool hold = !c.ballCamToggle;
    check("Hold Ball Camera", &hold, changed,
          "Rocket League's wording. On: hold the button to keep ball cam on. "
          "Off (default): press once to toggle ball cam.");
    c.ballCamToggle = !hold;
    check("Ball Cam Indicator", &c.ballCamIndicator, changed,
          "Show an indicator while ball cam is active.");
    check("Ball Arrow", &c.ballArrow, changed,
          "Draw an arrow pointing at the ball while in car cam (Rocket League's 'Ball Arrow').");
    check("Ball Floor Projection", &c.ballFloorProjection, changed,
          "Project the ball's position onto the ground as a shadow/marker. "
          "The marker carries an inner circle as a height cue - it opens up "
          "as the ball nears the floor and shrinks to a dot the higher the "
          "ball goes (Rocket League's ball indicator).");
    check("Flip Reset Indicator", &c.flipResetIndicator, changed,
          "Rocket League's flip reset indicator: a glowing white hollow disc "
          "that hangs under your car's underside while you are holding a reset "
          "taken off the BALL (all four wheels planted on it in mid-air). It "
          "flashes when the reset lands and stays until you use the flip or "
          "touch the ground. Resets taken off the wall or ceiling never show it.");
    check("Mouse Swivel", &c.mouseSwivel, changed,
          "Let the mouse swivel the camera while the settings menu is closed.");

    sliderF("Mouse Sensitivity", &c.mouseSens, 1.0f, 100.0f, "%.0f", changed, entryOpen,
            "How much mouse movement swivels the camera. Rocket League: 1 - 100.");
}

void drawControlsTab(ControlSettings& c, bool& changed, bool& entryOpen) {
    sliderF("Steering Sensitivity", &c.steerSens, 0.10f, 10.0f, "%.2f", changed, entryOpen,
            "Scales stick steering input (keys are never scaled). Rocket League's "
            "slider runs 1.00 - 10.00 (default 1.00); ours drops to 0.10 for "
            "finer-than-Rocket-League control.");
    sliderF("Aerial Sensitivity", &c.aerialSens, 0.10f, 10.0f, "%.2f", changed, entryOpen,
            "Scales pitch, yaw and air-roll stick input while airborne (keys are "
            "never scaled). Rocket League's slider runs 1.00 - 10.00 (default "
            "1.00); ours drops to 0.10 for finer-than-Rocket-League control.");
    sliderF("Controller Deadzone", &c.deadzone, 0.0f, 0.75f, "%.2f", changed, entryOpen,
            "Stick input inside the deadzone is neutral. Rocket League: 0.00 - 0.75 (default 0.20, patch v1.74).");
    sliderF("Dodge Deadzone", &c.dodgeDeadzone, 0.10f, 1.0f, "%.2f", changed, entryOpen,
            "How far the stick must move with jump to dodge instead of double-jumping. "
            "Rocket League: 0.10 - 1.00 (default 0.80, patch v1.74); applied to the car's dodgeDeadzone.");

    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Deadzone Shape");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(
            "Rocket League's Deadzone Shape: how far the stick must move, and how "
            "the deflection is rescaled past it. RL's default is Cross.\n\n"
            "Cross - each axis is deadzoned separately: a component inside the "
            "deadzone drops out while the other one passes through, and diagonals "
            "come out strong.\n"
            "Square - the same box-shaped deadzone, but the whole stick vector is "
            "scaled by one factor, so your direction through the deadzone is kept "
            "exactly.\n"
            "Circle - a radial deadzone: direction kept and output capped, so "
            "diagonals never outrun straight pushes.");
    ImGui::SameLine();
    if (ImGui::RadioButton("Cross", c.deadzoneShape == DeadzoneCross)) {
        c.deadzoneShape = DeadzoneCross;
        changed = true;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Square", c.deadzoneShape == DeadzoneSquare)) {
        c.deadzoneShape = DeadzoneSquare;
        changed = true;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Circle", c.deadzoneShape == DeadzoneCircle)) {
        c.deadzoneShape = DeadzoneCircle;
        changed = true;
    }

    check("Controller Vibration", &c.vibration, changed,
          "Rumble on boost activation, ball impacts and hard landings (requires a "
          "gamepad). Rocket League ships with vibration ON.");
}

void drawFreeplayTab(FreeplaySettings& f, bool& changed, bool& entryOpen) {
    static const char* kPresets[] = {
        resetShotPresetName(0), resetShotPresetName(1), resetShotPresetName(2),
        resetShotPresetName(3), resetShotPresetName(4), resetShotPresetName(5),
    };

    sliderF("Game Speed", &f.gameSpeed, 0.0f, 150.0f, "%.0f%%", changed, entryOpen,
            "How fast the simulation runs: 0% freezes the car and ball for a "
            "setup, 50% is half speed for learning mechanics, 100% is normal, "
            "150% is the ceiling. The camera keeps swiveling either way.");

    sliderF("Launch Ball Speed", &f.launchSpeed, 1000.0f, 5000.0f, "%.0f uu/s", changed, entryOpen,
            "Speed of the ball when 'Launch Ball' is pressed. The ball is fired "
            "from wherever it already is - it is never teleported back to the "
            "middle of the field.");
    sliderF("Launch Angle", &f.launchAngle, -90.0f, 90.0f, "%.0f deg", changed, entryOpen,
            "Angle of the launched ball: 0 = straight downfield, 90 = straight "
            "up, negative = angled back down.");

    int preset = f.resetShotPreset;
    comboIndex("Reset Shot", &preset, 6, kPresets, changed,
               "Where 'Reset Shot' teleports the car; the ball is always reset "
               "to the middle of the field.");
    f.resetShotPreset = preset;
}

void drawGraphicsTab(GraphicsSettings& g, bool& changed, bool& entryOpen) {
    check("VSync", &g.vsync, changed,
          "Synchronize frames to the display refresh rate.");

    // FPS limit: 0 means unlimited (called out under the slider).
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("FPS Limit");
    ImGui::SameLine();
    ImGui::PushItemWidth(-FLT_MIN);
    if (ImGui::SliderInt("##FPS Limit", &g.fpsCap, 0, 360, "%d", ImGuiSliderFlags_AlwaysClamp))
        changed = true;
    if (typedValuePopup("##FPS Limit", &g.fpsCap, 0, 360, entryOpen)) changed = true;
    ImGui::PopItemWidth();
    ImGui::SetItemTooltip("Cap the frame rate (0 = unlimited, 1 - 360). "
                          "Right-click the slider to type the value in.");
    if (g.fpsCap == 0) ImGui::TextDisabled("Unlimited");

    static const char* kParticles[] = {"Low", "Med", "High"};
    comboIndex("Particle Quality", &g.particleQuality, 3, kParticles, changed,
               "Density of boost, impact and supersonic particles.");

    static const char* kMsaa[] = {"Off", "4x", "8x"};
    int msaaIdx = (g.msaa == 4) ? 1 : (g.msaa == 8) ? 2 : 0;
    comboIndex("MSAA", &msaaIdx, 3, kMsaa, changed,
               "Multisample anti-aliasing on edges.");
    g.msaa = (msaaIdx == 1) ? 4 : (msaaIdx == 2) ? 8 : 0;
    ImGui::TextDisabled("MSAA changes require restarting the game to take effect.");

    sliderF("Wall Opacity", &g.wallOpacity, 0.0f, 1.0f, "%.2f", changed, entryOpen,
            "Opacity of the arena walls and ceiling (Rocket League keeps them "
            "see-through), so you can always see the car and ball through them. "
            "0 = invisible, 1 = solid. Default 0.30.");

    check("Show Hitboxes", &g.showHitboxes, changed,
          "Outline the physics hitboxes: an oriented box around the car "
          "(the exact Octane hitbox) and a sphere around the ball.");
}

void drawBindingsTab(Settings& settings, InputSystem& input, bool& changed) {
    // The InputSystem owns the live bindings (it performs capture/clear), so
    // pull its state back into Settings whenever it differs. This also picks
    // up a capture that completed between two calls to draw().
    auto sync = [&] {
        if (input.bindings().binds != settings.binds.binds) {
            settings.binds = input.bindings();
            changed = true;
        }
    };
    sync();

    const bool capturing = input.capturing();
    const Action capAction = input.captureAction();
    const int capSlot = input.captureSlot();

    ImGui::TextDisabled(
        "Click a binding to re-bind it, x removes it, + adds another. Esc cancels a capture. "
        "Stick directions can be bound too (e.g. LS Left -> Steer Left, or RS Down -> Boost); "
        "binding a stick direction claims that whole axis, so it stops doing its default "
        "steering/swivel job - bind the other direction as well to keep both.");

    for (int i = 0; i < (int)Action::COUNT_; i++) {
        const Action a = (Action)i;
        ImGui::PushID(i);

        const bool isCapRow = capturing && capAction == a;

        // Snapshot this row: clearBinding() below mutates the real vector.
        BindList row = settings.binds.get(a);

        // ---- header line: action name + optional "Clear all"
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(actionName(a));

        if (!row.empty()) {
            ImGui::SameLine();
            ImGui::PushID("clear");
            ImGui::BeginDisabled(capturing);   // no mutations while capturing
            if (ImGui::SmallButton("Clear all")) {
                // index into the snapshot; erase from the back so slots stay valid
                for (int s = (int)row.size() - 1; s >= 0; s--) input.clearBinding(a, s);
                sync();
                row.clear();
            }
            ImGui::EndDisabled();
            ImGui::PopID();
        }

        // An unbound menu action still reads Esc (InputSystem::actionRaw), so
        // say so rather than let the empty row look like the menu is gone.
        if (row.empty() && a == Action::Pause) {
            ImGui::SameLine();
            ImGui::TextDisabled("(unbound - Esc still opens the menu)");
        }

        // ---- binding buttons (next line)
        int removeSlot = -1;      // at most one of these fires per frame
        int recaptureSlot = -1;
        bool append = false;
        bool placeholderShown = false;

        for (int s = 0; s < (int)row.size(); s++) {
            ImGui::PushID(s);
            if (isCapRow && capSlot == s) {
                capturePlaceholder();           // re-binding this exact slot
                placeholderShown = true;
            } else {
                ImGui::BeginDisabled(capturing);
                const std::string lbl = InputSystem::describe(row[s]);
                if (ImGui::Button(lbl.c_str())) recaptureSlot = s;
                ImGui::SetItemTooltip("Click to re-bind this input.");
                ImGui::SameLine(0.0f, 2.0f);
                if (ImGui::SmallButton("x")) removeSlot = s;
                ImGui::SetItemTooltip("Remove this binding.");
                ImGui::SameLine(0.0f, 6.0f);
                ImGui::EndDisabled();
            }
            ImGui::PopID();
        }

        if (!placeholderShown) {
            if (isCapRow) {
                capturePlaceholder();           // appending a new binding (or stale slot)
            } else {
                ImGui::PushID("add");
                ImGui::BeginDisabled(capturing);
                if (ImGui::SmallButton("+")) append = true;
                ImGui::SetItemTooltip("Add another binding for this action.");
                ImGui::EndDisabled();
                ImGui::PopID();
            }
        }

        // ---- apply at most one mutation this frame (index came from the snapshot)
        if (removeSlot >= 0) {
            input.clearBinding(a, removeSlot);
            sync();
        } else if (recaptureSlot >= 0) {
            input.beginCapture(a, recaptureSlot);
        } else if (append) {
            input.beginCapture(a, -1);
        }

        ImGui::PopID();
    }
}

void drawSoundTab(SoundSettings& s, bool& changed, bool& entryOpen) {
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Volume");
    ImGui::SameLine();
    ImGui::PushItemWidth(-FLT_MIN);
    float pct = s.volume * 100.0f;
    if (ImGui::SliderFloat("##Volume", &pct, 0.0f, 100.0f, "%.0f%%",
                           ImGuiSliderFlags_AlwaysClamp)) {
        s.volume = pct / 100.0f;
        changed = true;
    }
    if (typedValuePopup("##Volume", &pct, 0.0f, 100.0f, entryOpen)) {
        s.volume = pct / 100.0f;
        changed = true;
    }
    ImGui::PopItemWidth();
    ImGui::SetItemTooltip(
        "Master volume for all sound effects (menu, boost, engine, jumps, "
        "ball impacts). Right-click the slider to type the value in.");
    ImGui::TextDisabled(
        "Sounds: menu clicks, boost, engine hum, jumps/flips and ball hits.");
}

void drawResetTab(Settings& settings, InputSystem& input, bool& changed) {
    ImGui::TextWrapped("Restore every setting to its Rocket League-style default.");
    if (ImGui::Button("Reset All to Defaults")) {
        Settings fresh;                    // defaults for camera/controls/bindings
        settings = fresh;
        input.setBindings(settings.binds);
        input.cancelCapture();
        changed = true;
    }
    ImGui::TextWrapped(
        "Note: this also resets bindings to the default keyboard/mouse + gamepad layout. "
        "VSync, MSAA and the particle quality are re-applied after closing this menu.");
}

} // namespace

// ------------------------------------------------------------------------ draw

bool SettingsUI::draw(Settings& settings, InputSystem& input, bool* pOpen, MapTabState& map) {
    anyChanged = false;
    bool& changed = anyChanged;
    // Recomputed every frame by whichever popup is up, so a closed menu or a
    // closed popup reports false and cannot leave Escape permanently dead.
    bool& entryOpen = valueEntryOpen_;

    ImGui::SetNextWindowSize(ImVec2(620.0f, 720.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Settings", pOpen, ImGuiWindowFlags_NoCollapse)) {
        ImGui::End();
        valueEntryOpen_ = false;
        return anyChanged;
    }

    if (ImGui::BeginTabBar("SettingsTabs")) {
        if (ImGui::BeginTabItem("Camera")) {
            ImGui::PushID("Camera");
            drawCameraTab(settings.cam, changed, entryOpen);
            ImGui::PopID();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Controls")) {
            ImGui::PushID("Controls");
            drawControlsTab(settings.ctrl, changed, entryOpen);
            ImGui::PopID();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Freeplay")) {
            ImGui::PushID("Freeplay");
            drawFreeplayTab(settings.freeplay, changed, entryOpen);
            ImGui::PopID();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Map", nullptr,
                                selectMapTab ? ImGuiTabItemFlags_SetSelected : 0)) {
            selectMapTab = false;
            ImGui::PushID("Map");
            drawMapTab(map, changed);
            ImGui::PopID();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Graphics")) {
            ImGui::PushID("Graphics");
            drawGraphicsTab(settings.gfx, changed, entryOpen);
            ImGui::PopID();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Sound")) {
            ImGui::PushID("Sound");
            drawSoundTab(settings.sound, changed, entryOpen);
            ImGui::PopID();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Bindings")) {
            ImGui::PushID("Bindings");
            drawBindingsTab(settings, input, changed);
            ImGui::PopID();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Reset")) {
            ImGui::PushID("Reset");
            drawResetTab(settings, input, changed);
            ImGui::PopID();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    ImGui::End();
    return anyChanged;
}

// ------------------------------------------------------------------- Map tab

namespace {

bool isMapExt(const std::string& name) {
    const size_t dot = name.find_last_of('.');
    if (dot == std::string::npos) return false;
    std::string e = name.substr(dot + 1);
    for (char& c : e) c = (char)std::tolower((unsigned char)c);
    return e == "udk" || e == "upk";
}

}  // namespace

void SettingsUI::refreshBrowse() {
    namespace fs = std::filesystem;
    std::error_code ec;

    if (browseDir_.empty()) {
        const char* first[] = {"downloaded_maps", "maps", "."};
        for (const char* c : first) {
            if (fs::is_directory(c, ec)) {
                browseDir_ = fs::absolute(c, ec).string();
                break;
            }
        }
        if (browseDir_.empty()) browseDir_ = fs::current_path(ec).string();
    }

    browseRows_.clear();
    browseError_ = false;

    std::vector<std::string> dirs, files;
    fs::directory_iterator it(browseDir_, ec);
    if (ec) {
        browseError_ = true;
        return;
    }
    const fs::directory_iterator end;
    for (; it != end; it.increment(ec)) {
        if (ec) {
            browseError_ = true;
            return;
        }
        const std::string name = it->path().filename().string();
        if (name.empty() || name[0] == '.') continue;  // skips "." and ".."
        std::error_code tec;
        if (it->is_directory(tec)) {
            dirs.push_back(name);
        } else if (!tec && isMapExt(name)) {
            files.push_back(name);
        }
    }

    std::sort(dirs.begin(), dirs.end());
    std::sort(files.begin(), files.end());

    const fs::path cur = fs::path(browseDir_).lexically_normal();
    if (cur.has_parent_path() && cur.parent_path() != cur) browseRows_.emplace_back("..", true);
    for (const std::string& d : dirs) browseRows_.emplace_back(d, true);
    for (const std::string& f : files) browseRows_.emplace_back(f, false);
}

void SettingsUI::drawMapTab(MapTabState& map, bool& changed) {
    (void)changed;  // the Map tab changes nothing that needs saving
    if (browseDir_.empty()) refreshBrowse();

    // ---- status -------------------------------------------------------
    if (map.loading) {
        ImGui::TextColored(ImVec4(1.0f, 0.80f, 0.25f, 1.0f), "Loading %s ...",
                           map.loadingName.c_str());
    } else if (map.loaded) {
        ImGui::TextColored(ImVec4(0.45f, 0.90f, 0.55f, 1.0f), "Playing: %s",
                           map.loadedName.c_str());
    } else {
        ImGui::TextUnformatted("Default arena (no custom map)");
    }

    if (map.loaded && map.sessions > 0) {
        ImGui::AlignTextToFramePadding();
        ImGui::Text("Checkpoint %d / %d", map.session + 1, map.sessions);
        ImGui::SameLine();
        if (ImGui::SmallButton("< Previous")) map.cycleRequest -= 1;
        ImGui::SameLine();
        if (ImGui::SmallButton("Next >")) map.cycleRequest += 1;
        ImGui::TextDisabled("Launch Ball / Take Possession step through these too");
        if (map.derivedSessions > 0)
            ImGui::TextDisabled("%d of these come from the map's navigation points",
                                map.derivedSessions);
    } else if (map.loaded) {
        ImGui::TextDisabled("This map has no checkpoints; Reset Shot returns to the start.");
    }

    if (!map.error.empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.45f, 0.42f, 1.0f));
        ImGui::TextWrapped("%s", map.error.c_str());
        ImGui::PopStyleColor();
    }

    ImGui::Separator();

    // ---- manual path entry -------------------------------------------
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Path:");
    ImGui::SameLine();
    // reserve room for whatever buttons follow on this row
    ImGui::PushItemWidth(map.loaded ? -196.0f : -92.0f);
    const bool enter =
        ImGui::InputText("##mappath", pathBuf_, sizeof(pathBuf_),
                         ImGuiInputTextFlags_EnterReturnsTrue);
    ImGui::PopItemWidth();
    ImGui::SameLine();

    bool doLoad = false;
    if (map.loading) {
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 0.5f);
        ImGui::Button("Load");
        ImGui::PopStyleVar();
    } else if (ImGui::Button("Load") || (enter && pathBuf_[0] != '\0')) {
        doLoad = true;
    }
    if (doLoad) map.loadRequest = pathBuf_;

    if (map.loaded) {
        ImGui::SameLine();
        if (ImGui::Button("Default arena")) map.unloadRequest = true;
    }

    ImGui::Separator();

    // ---- browser ------------------------------------------------------
    if (ImGui::SmallButton("Up")) {
        std::filesystem::path p = std::filesystem::path(browseDir_).parent_path();
        if (p.empty()) p = "/";
        browseDir_ = p.string();
        refreshBrowse();
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Refresh")) refreshBrowse();
    ImGui::SameLine();
    ImGui::TextDisabled("%s", browseDir_.c_str());

    if (browseError_) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.45f, 0.42f, 1.0f));
        ImGui::TextUnformatted("Cannot read this folder.");
        ImGui::PopStyleColor();
    }

    ImGui::BeginChild("##mapfiles", ImVec2(0, 230), true);
    for (size_t i = 0; i < browseRows_.size(); i++) {
        const std::string& name = browseRows_[i].first;
        const bool isDir = browseRows_[i].second;
        ImGui::PushID((int)i);
        const std::string label = (isDir ? "> " : "") + name;
        if (ImGui::Selectable(label.c_str(), false) && !map.loading) {
            const std::filesystem::path p = std::filesystem::path(browseDir_) / name;
            if (isDir) {
                browseDir_ = p.lexically_normal().string();
                refreshBrowse();
            } else {
                const std::string full = p.string();
                std::snprintf(pathBuf_, sizeof(pathBuf_), "%s", full.c_str());
                map.loadRequest = full;
            }
        }
        if (!isDir) ImGui::SetItemTooltip("Load this map");
        ImGui::PopID();
    }
    if (browseRows_.empty() && !browseError_)
        ImGui::TextDisabled("No .udk / .upk files in this folder.");
    ImGui::EndChild();
}
