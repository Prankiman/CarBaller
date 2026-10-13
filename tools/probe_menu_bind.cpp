// Escape-as-menu safety net probe: with no binds on Action::Pause, Esc must
// still read as the menu key (InputSystem::actionRaw); once Pause is bound
// again, Esc must go quiet. Run headless with SDL's dummy video driver.
//
// Build:
//   c++ -std=c++20 -O2 -w -I. -Isrc -Ithird_party $(sdl2-config --cflags) \
//       tools/probe_menu_bind.cpp src/input/input.cpp src/settings/settings.cpp \
//       $(sdl2-config --libs) -o probe_menu_bind
// Run (headless: no window, no real keyboard):
//   SDL_VIDEODRIVER=dummy ./probe_menu_bind
// exit 0 = all checks ok
#include "input/input.h"

#include <SDL.h>

#include <cstdio>

namespace {
int failed = 0;

void check(bool ok, const char* what, const char* detail) {
    if (!ok) failed++;
    std::printf("%-52s %s  %s\n", what, ok ? "ok  " : "FAIL", detail);
}

// actionRaw reads SDL's keyboard-state array (the canonical "is it held"
// source a real backend maintains during the event pump). The dummy video
// driver has no keyboard backend, so synthetic SDL_KEYDOWN events never
// reach that array here - poke it directly instead, which is the same
// memory a real backend would write.
Uint8* keyState() { return const_cast<Uint8*>(SDL_GetKeyboardState(nullptr)); }
void pressEsc() { keyState()[SDL_SCANCODE_ESCAPE] = 1; }
void releaseEsc() { keyState()[SDL_SCANCODE_ESCAPE] = 0; }
}  // namespace

int main() {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) != 0) {
        std::printf("SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }

    Settings s;
    s.binds = Bindings::defaults();   // what a shipped settings.json loads as
    InputSystem in;
    in.init(nullptr, &s);

    // ---- 1. strip both default binds (Esc key, pad Start) from Pause
    check(in.bindings().get(Action::Pause).size() == 2, "Pause starts with Esc + pad Start",
          "2 default binds");
    in.clearBinding(Action::Pause, 0);   // Esc key
    in.clearBinding(Action::Pause, 0);   // pad Start (slots shift up)
    check(in.bindings().get(Action::Pause).empty(), "Pause is now fully unbound",
          "bind list empty");

    in.update();
    pressEsc();
    in.update();
    check(in.value(Action::Pause) == 1.0f,
          "unbound Pause still reads Esc (no way to lock the menu away)",
          "value == 1 while Esc down");
    releaseEsc();
    in.update();

    // ---- 2. binding Pause to something else silences the Esc fallback
    in.beginCapture(Action::Pause, -1);
    {
        SDL_Event e{};
        e.type = SDL_KEYDOWN;
        e.key.state = SDL_PRESSED;
        e.key.keysym.scancode = SDL_SCANCODE_F5;
        e.key.keysym.sym = SDLK_F5;
        in.handleEvent(e);   // capture reads the event struct directly
    }
    in.update();
    check(in.bindings().get(Action::Pause).size() == 1 &&
              in.bindings().get(Action::Pause)[0].code == SDL_SCANCODE_F5,
          "Pause re-bound to F5", "1 bind, F5");
    check(in.value(Action::Pause) == 0.0f,
          "Esc is quiet again once Pause has a real bind",
          "value == 0 while Esc down");

    // ---- 3. clearing that real bind brings the Esc fallback straight back
    in.clearBinding(Action::Pause, 0);
    in.update();
    check(in.value(Action::Pause) == 0.0f, "Esc up with no bind: silent",
          "value == 0");
    pressEsc();
    in.update();
    check(in.value(Action::Pause) == 1.0f, "Esc down with no bind: menu key again",
          "value == 1");
    releaseEsc();
    in.update();

    // ---- 4. a file that pins Pause to an empty list self-heals on load: the
    // empty list is skipped and the default binds (Esc + pad Start) return.
    {
        const char* p = "/tmp/opencode/menu_bind.json";
        Settings w;
        w.binds = Bindings::defaults();
        w.binds.set(Action::Pause, {});
        if (!w.save(p)) { std::printf("save failed\n"); return 1; }
        Settings r;
        if (!r.load(p)) { std::printf("load failed\n"); return 1; }
        const BindList& l = r.binds.get(Action::Pause);
        bool hasEsc = false;
        for (const Binding& b : l)
            if (b.type == BindType::Key && b.code == SDL_SCANCODE_ESCAPE) hasEsc = true;
        check(l.size() == Bindings::defaults().get(Action::Pause).size() && hasEsc,
              "empty Pause list in the file comes back as the defaults", "Esc restored");
    }

    // ---- 5. a settings.json with no bindings block at all (and a first run
    // with no file) must still leave the player with working controls.
    {
        FILE* f = std::fopen("/tmp/opencode/menu_nobinds.json", "w");
        std::fputs("{\"camera\":{\"fov\":100}}", f);
        std::fclose(f);
        Settings bare;
        if (!bare.load("/tmp/opencode/menu_nobinds.json")) {
            std::printf("load failed\n");
            return 1;
        }
        check(!bare.binds.get(Action::Throttle).empty() &&
                  !bare.binds.get(Action::Pause).empty(),
              "no bindings block: every action keeps its default binds",
              "Throttle + Pause bound");

        // and the same for a first run, where there is no file at all
        Settings none;
        check(!none.load("/tmp/opencode/menu_nobinds_missing.json") &&
                  !none.binds.get(Action::Throttle).empty(),
              "no settings.json at all: controls are the RL defaults",
              "Throttle bound");

        InputSystem in2;
        in2.init(nullptr, &none);
        in2.update();
        pressEsc();
        in2.update();
        check(in2.value(Action::Pause) == 1.0f,
              "Esc still opens the menu with every bind gone",
              "value == 1");
        releaseEsc();
        in2.update();
        in2.shutdown();
    }

    in.shutdown();
    SDL_Quit();
    std::printf("%s\n", failed ? "MENU BIND REGRESSION" : "all menu bind checks ok");
    return failed ? 1 : 0;
}
