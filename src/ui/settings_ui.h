#pragma once
#include "../input/input.h"
#include "../settings/settings.h"

#include <string>
#include <utility>
#include <vector>

// State shared between the Map tab and the app loop. The tab never touches
// the simulation or the renderer itself - it only sets one-shot requests
// (loadRequest / unloadRequest) which the app consumes each frame, and reads
// back what is currently loaded for display.
struct MapTabState {
    std::string loadRequest;     // non-empty: app should start loading this path
    bool unloadRequest = false;  // app should return to the default arena
    int cycleRequest = 0;        // -1 / +1: step through checkpoints
    bool loading = false;        // a parse job is in flight (tab shows "Loading..")
    std::string loadingName;     // file name of the in-flight load

    bool loaded = false;
    std::string loadedName;      // file name only, for the status line
    int sessions = 0;
    int session = 0;
    int derivedSessions = 0;     // checkpoints built from the map's PathNodes
    std::string error;           // most recent load failure
};

class SettingsUI {
public:
    // Draws the settings window. Returns true if any setting value changed
    // (caller should re-apply things like vsync, and save).
    bool draw(Settings& settings, InputSystem& input, bool* pOpen, MapTabState& map);

    // True while a right-click "type a value" popup is showing, as of the
    // most recent draw(). app.cpp holds the menu's Escape while this is set,
    // so Escape cancels the edit first and only a second press resumes.
    // Reset on both draw() exits, so a closed menu never reports true.
    bool valueEntryOpen() const { return valueEntryOpen_; }

    bool anyChanged = false;   // set by draw() when a value changed this frame

    // Test hook: force the Map tab to come to the front on the next draw
    // (consumed immediately, so it can't fight the user afterwards).
    bool selectMapTab = false;

private:
    // Map tab: in-app browser over .udk/.upk files plus a manual path box.
    void drawMapTab(MapTabState& map, bool& changed);
    void refreshBrowse();

    std::string browseDir_;                       // directory being listed
    std::vector<std::pair<std::string, bool>> browseRows_;  // (name, isDir)
    char pathBuf_[640] = {};
    bool browseError_ = false;
    bool valueEntryOpen_ = false;
};
