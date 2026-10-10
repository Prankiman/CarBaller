// Rocket League-style in-game settings menu (ImGui).
// Tabs: Camera / Controls / Freeplay / Graphics / Sound / Bindings / Reset.

#include "settings_ui.h"

#include <imgui.h>

#include <cfloat>
#include <string>

namespace {

// ------------------------------------------------------------------------
// RL-style widget helpers. Each one flips *changed (bool&) when the user
// edits a value, so draw() can report "something changed this frame".
// ------------------------------------------------------------------------

// Label on the left, slider spanning the rest of the row.
void sliderF(const char* label, float* v, float lo, float hi,
             const char* fmt, bool& changed, const char* tip = nullptr) {
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(label);
    ImGui::SameLine();
    ImGui::PushItemWidth(-FLT_MIN);
    const std::string id = std::string("##") + label;
    if (ImGui::SliderFloat(id.c_str(), v, lo, hi, fmt, ImGuiSliderFlags_AlwaysClamp))
        changed = true;
    ImGui::PopItemWidth();
    if (tip) ImGui::SetItemTooltip("%s", tip);
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

void drawCameraTab(CameraSettings& c, bool& changed) {
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

    sliderF("FOV", &c.fov, 60.0f, 110.0f, "%.0f", changed,
            "Horizontal field of view. Rocket League: 60 - 110 (default 110).");
    sliderF("Height", &c.height, 40.0f, 200.0f, "%.0f", changed,
            "How high above the car the camera floats. Rocket League: 40 - 200 (default 100).");
    sliderF("Angle", &c.angle, -45.0f, 0.0f, "%.0f", changed,
            "Camera pitch relative to the car; negative looks down. Rocket League: -45 - 0 (default -4).");
    sliderF("Distance", &c.distance, 100.0f, 400.0f, "%.0f", changed,
            "How far the camera sits behind the car. Rocket League: 100 - 400 (default 270).");
    sliderF("Stiffness", &c.stiffness, 0.0f, 1.0f, "%.2f", changed,
            "How rigidly the camera follows the car. Higher = tighter, lower = floatier. Rocket League: 0 - 1 (default 0.50).");
    sliderF("Swivel Speed", &c.swivelSpeed, 1.0f, 10.0f, "%.1f", changed,
            "How fast the camera swivels when you look around. Rocket League: 1 - 10.");
    sliderF("Transition Speed", &c.transitionSpeed, 1.0f, 2.0f, "%.2f", changed,
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

    sliderF("Mouse Sensitivity", &c.mouseSens, 1.0f, 100.0f, "%.0f", changed,
            "How much mouse movement swivels the camera. Rocket League: 1 - 100.");
}

void drawControlsTab(ControlSettings& c, bool& changed) {
    sliderF("Steering Sensitivity", &c.steerSens, 1.0f, 10.0f, "%.2f", changed,
            "Scales stick steering input. Rocket League: 1.00 - 10.00 (default 1.00).");
    sliderF("Aerial Sensitivity", &c.aerialSens, 1.0f, 10.0f, "%.2f", changed,
            "Scales pitch and yaw stick input while airborne. Rocket League: 1.00 - 10.00 (default 1.00).");
    sliderF("Controller Deadzone", &c.deadzone, 0.0f, 0.75f, "%.2f", changed,
            "Stick input inside the deadzone is neutral. Rocket League: 0.00 - 0.75 (default 0.20, patch v1.74).");
    sliderF("Dodge Deadzone", &c.dodgeDeadzone, 0.10f, 1.0f, "%.2f", changed,
            "How far the stick must move with jump to dodge instead of double-jumping. "
            "Rocket League: 0.10 - 1.00 (default 0.80, patch v1.74); applied to the car's dodgeDeadzone.");

    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Deadzone Shape");
    ImGui::SameLine();
    if (ImGui::RadioButton("Cross", c.deadzoneShape == 0)) {
        c.deadzoneShape = 0;
        changed = true;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Circle", c.deadzoneShape == 1)) {
        c.deadzoneShape = 1;
        changed = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Cross: square deadzone (axis-by-axis). Circle: radial deadzone.");

    check("Controller Vibration", &c.vibration, changed,
          "Rumble on boost activation, ball impacts and hard landings (requires a "
          "gamepad). Rocket League ships with vibration ON.");
}

void drawFreeplayTab(FreeplaySettings& f, bool& changed) {
    static const char* kPresets[] = {
        "Blue Corner Kickoff",
        "Orange Corner Kickoff",
        "Midfield (Blue Half)",
        "Midfield (Orange Half)",
        "Aerial Drop",
        "Blue Goal Line",
    };

    sliderF("Game Speed", &f.gameSpeed, 0.0f, 150.0f, "%.0f%%", changed,
            "How fast the simulation runs: 0% freezes the car and ball for a "
            "setup, 50% is half speed for learning mechanics, 100% is normal, "
            "150% is the ceiling. The camera keeps swiveling either way.");

    sliderF("Launch Ball Speed", &f.launchSpeed, 1000.0f, 5000.0f, "%.0f uu/s", changed,
            "Speed of the ball when 'Launch Ball' is pressed. The ball is fired "
            "from wherever it already is - it is never teleported back to the "
            "middle of the field.");
    sliderF("Launch Angle", &f.launchAngle, -90.0f, 90.0f, "%.0f deg", changed,
            "Angle of the launched ball: 0 = straight downfield, 90 = straight "
            "up, negative = angled back down.");

    int preset = f.resetShotPreset;
    comboIndex("Reset Shot", &preset, 6, kPresets, changed,
               "Where 'Reset Shot' teleports the car; the ball is always reset "
               "to the middle of the field.");
    f.resetShotPreset = preset;
}

void drawGraphicsTab(GraphicsSettings& g, bool& changed) {
    check("VSync", &g.vsync, changed,
          "Synchronize frames to the display refresh rate.");

    // FPS limit: 0 means unlimited (called out under the slider).
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("FPS Limit");
    ImGui::SameLine();
    ImGui::PushItemWidth(-FLT_MIN);
    if (ImGui::SliderInt("##FPS Limit", &g.fpsCap, 0, 360, "%d", ImGuiSliderFlags_AlwaysClamp))
        changed = true;
    ImGui::PopItemWidth();
    ImGui::SetItemTooltip("Cap the frame rate (0 - 360).");
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

    sliderF("Wall Opacity", &g.wallOpacity, 0.0f, 1.0f, "%.2f", changed,
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

void drawSoundTab(SoundSettings& s, bool& changed) {
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
    ImGui::PopItemWidth();
    ImGui::SetItemTooltip(
        "Master volume for all sound effects (menu, boost, engine, jumps, "
        "ball impacts).");
    ImGui::TextDisabled(
        "Sounds: menu clicks, boost, engine hum, jumps/flips and ball hits.");
}

void drawResetTab(Settings& settings, InputSystem& input, bool& changed) {
    ImGui::TextWrapped("Restore every setting to its Rocket League-style default.");
    if (ImGui::Button("Reset All to Defaults")) {
        Settings fresh;                    // defaults for camera/controls/freeplay/graphics
        fresh.binds = Bindings::defaults();  // Settings{} leaves bindings empty
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

bool SettingsUI::draw(Settings& settings, InputSystem& input, bool* pOpen) {
    anyChanged = false;
    bool& changed = anyChanged;

    ImGui::SetNextWindowSize(ImVec2(620.0f, 720.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Settings", pOpen, ImGuiWindowFlags_NoCollapse)) {
        ImGui::End();
        return anyChanged;
    }

    if (ImGui::BeginTabBar("SettingsTabs")) {
        if (ImGui::BeginTabItem("Camera")) {
            ImGui::PushID("Camera");
            drawCameraTab(settings.cam, changed);
            ImGui::PopID();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Controls")) {
            ImGui::PushID("Controls");
            drawControlsTab(settings.ctrl, changed);
            ImGui::PopID();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Freeplay")) {
            ImGui::PushID("Freeplay");
            drawFreeplayTab(settings.freeplay, changed);
            ImGui::PopID();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Graphics")) {
            ImGui::PushID("Graphics");
            drawGraphicsTab(settings.gfx, changed);
            ImGui::PopID();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Sound")) {
            ImGui::PushID("Sound");
            drawSoundTab(settings.sound, changed);
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
