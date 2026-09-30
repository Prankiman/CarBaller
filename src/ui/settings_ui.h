#pragma once
#include "../settings/settings.h"
#include "../input/input.h"

class SettingsUI {
public:
    // Draws the settings window. Returns true if any setting value changed
    // (caller should re-apply things like vsync, and save).
    bool draw(Settings& settings, InputSystem& input, bool* pOpen);

    bool anyChanged = false;   // set by draw() when a value changed this frame
};
