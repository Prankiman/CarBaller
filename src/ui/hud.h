#pragma once
// Minimal RL-style HUD: boost gauge, ball-cam indicator, car->ball arrow,
// optional stats overlay.

#include "../camera/rl_camera.h"
#include "../render/renderer.h"
#include "../settings/settings.h"
#include "../sim/sim.h"

class HUD {
public:
    void draw(const SimSnapshot& snap, const RLCamera& cam, const Renderer& renderer,
              const Settings& settings, int w, int h, float fps, bool statsVisible,
              bool menuOpen);
};
