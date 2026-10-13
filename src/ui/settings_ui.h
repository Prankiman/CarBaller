#pragma once
#include "../settings/settings.h"
#include "../input/input.h"

class SettingsUI {
public:
    // Draws the settings window. Returns true if any setting value changed
    // (caller should re-apply things like vsync, and save).
    bool draw(Settings& settings, InputSystem& input, bool* pOpen);

    // True while a right-click "type a value" popup is showing, as of the
    // most recent draw(). app.cpp holds the menu's Escape while this is set,
    // so Escape cancels the edit first and only a second press resumes.
    // Reset on both draw() exits, so a closed menu never reports true.
    bool valueEntryOpen() const { return valueEntryOpen_; }

    bool anyChanged = false;   // set by draw() when a value changed this frame

private:
    bool valueEntryOpen_ = false;
};
