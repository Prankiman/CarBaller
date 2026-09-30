#include "app.h"

#include <SDL.h>
#include <GL/glew.h>
#include <imgui.h>
#include <backends/imgui_impl_opengl3.h>
#include <backends/imgui_impl_sdl2.h>

#include <cstdlib>
#include <filesystem>
#include <thread>
#include <cstdio>

#include "../camera/rl_camera.h"
#include "../fx/particles.h"
#include "../input/input.h"
#include "../render/renderer.h"
#include "../settings/settings.h"
#include "../sim/sim.h"
#include "../ui/hud.h"
#include "../ui/settings_ui.h"

#ifndef CARBALLER_SOURCE_DIR
#define CARBALLER_SOURCE_DIR "."
#endif

namespace {

struct App {
    Settings settings;
    InputSystem input;
    Sim sim;
    RLCamera cam;
    Renderer renderer;
    ParticleSystem particles;
    HUD hud;
    SettingsUI settingsUI;

    SDL_Window* window = nullptr;
    SDL_GLContext glctx = nullptr;

    bool menuOpen = false;
    bool statsVisible = false;
    bool camResetPending = true;
    bool needSave = false;
    int lastVsync = -1;

    // boost particle accumulator
    float boostAccum = 0;
};

std::string findMeshDir() {
    const char* candidates[] = {
        "assets/collision_meshes",
        "../assets/collision_meshes",
        CARBALLER_SOURCE_DIR "/assets/collision_meshes",
    };
    for (const char* c : candidates) {
        if (std::filesystem::exists(std::filesystem::path(c) / "soccar"))
            return c;
    }
    return candidates[0];
}

void applyGraphics(App& a) {
    int vsync = a.settings.gfx.vsync ? 1 : 0;
    if (vsync != a.lastVsync) {
        SDL_GL_SetSwapInterval(vsync);
        a.lastVsync = vsync;
    }
}

}  // namespace

int runApp(int argc, char** argv) {
    (void)argc;
    (void)argv;

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER | SDL_INIT_HAPTIC) != 0) {
        std::fprintf(stderr, "[app] SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

    App a;
    a.settings.load("settings.json");

    if (a.settings.gfx.msaa > 0) {
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, a.settings.gfx.msaa);
    }

    const bool selfTest = std::getenv("CARBALLER_SELFTEST") != nullptr;
    const bool stShow = std::getenv("CARBALLER_ST_SHOW") != nullptr;

    a.window = SDL_CreateWindow("carballer",
                                SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                1600, 900,
                                SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE |
                                    SDL_WINDOW_ALLOW_HIGHDPI |
                                    (selfTest && !stShow ? SDL_WINDOW_HIDDEN : 0));
    if (!a.window) {
        std::fprintf(stderr, "[app] SDL_CreateWindow failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }
    a.glctx = SDL_GL_CreateContext(a.window);
    if (!a.glctx) {
        std::fprintf(stderr, "[app] SDL_GL_CreateContext failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(a.window);
        SDL_Quit();
        return 1;
    }
    SDL_GL_MakeCurrent(a.window, a.glctx);
    SDL_GL_SetSwapInterval(1);
    a.lastVsync = 1;

    glewExperimental = GL_TRUE;
    GLenum glewErr = glewInit();
    (void)glewErr;  // may return GLEW_ERROR_NO_GLX_DISPLAY on some setups; ignore
    glGetError();   // consume spurious error from glewInit
    if (a.settings.gfx.msaa > 0) glEnable(GL_MULTISAMPLE);

    // ---- ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    {
        ImGuiStyle& st = ImGui::GetStyle();
        st.WindowRounding = 6.0f;
        st.FrameRounding = 4.0f;
        st.GrabRounding = 3.0f;
        st.WindowBorderSize = 1.0f;
        st.FramePadding = ImVec2(8, 4);
        st.ItemSpacing = ImVec2(8, 6);
        st.Colors[ImGuiCol_WindowBg] = ImVec4(0.06f, 0.07f, 0.09f, 0.96f);
        st.Colors[ImGuiCol_TitleBgActive] = ImVec4(0.10f, 0.11f, 0.14f, 1.0f);
        st.Colors[ImGuiCol_Button] = ImVec4(0.16f, 0.18f, 0.23f, 1.0f);
        st.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.22f, 0.25f, 0.32f, 1.0f);
        st.Colors[ImGuiCol_SliderGrab] = ImVec4(0.95f, 0.60f, 0.15f, 1.0f);
        st.Colors[ImGuiCol_CheckMark] = ImVec4(0.95f, 0.60f, 0.15f, 1.0f);
        st.Colors[ImGuiCol_TabActive] = ImVec4(0.24f, 0.19f, 0.11f, 1.0f);
    }
    ImGui_ImplSDL2_InitForOpenGL(a.window, a.glctx);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    // ---- input
    a.input.init(a.window, &a.settings);

    // ---- sim
    std::string meshDir = findMeshDir();
    if (!a.sim.init(meshDir)) {
        std::fprintf(stderr, "[app] sim init failed (mesh dir: %s)\n", meshDir.c_str());
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplSDL2_Shutdown();
        ImGui::DestroyContext();
        SDL_GL_DeleteContext(a.glctx);
        SDL_DestroyWindow(a.window);
        SDL_Quit();
        return 1;
    }
    a.sim.applyControlSettings(a.settings.ctrl);

    // ---- renderer
    if (!a.renderer.init(meshDir)) {
        std::fprintf(stderr, "[app] renderer init failed\n");
        a.sim.shutdown();
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplSDL2_Shutdown();
        ImGui::DestroyContext();
        SDL_GL_DeleteContext(a.glctx);
        SDL_DestroyWindow(a.window);
        SDL_Quit();
        return 1;
    }

    // ---- ball hit feedback
    a.sim.onBallHit = [&a](const BallHitEvent& ev) {
        int q = a.settings.gfx.particleQuality;
        a.particles.spawnImpact(ev.pos, ev.strength, q);
        a.cam.kickShake(clampf(ev.strength / 6000.0f, 0.05f, 0.9f));
        if (a.settings.ctrl.vibration) {
            float strong = clampf(ev.strength / 3500.0f, 0.15f, 1.0f);
            a.input.rumble(uint16_t(strong * 0xFFFF), 0, 140);
        }
    };

    applyGraphics(a);

    // ---- main loop
    bool running = true;
    uint64_t prevFreq = SDL_GetPerformanceFrequency();
    uint64_t prevCount = SDL_GetPerformanceCounter();
    float fpsSmoothed = 60.0f;

    while (running) {
        uint64_t nowCount = SDL_GetPerformanceCounter();
        float dt = float(nowCount - prevCount) / float(prevFreq);
        prevCount = nowCount;
        if (dt > 0.25f) dt = 0.25f;
        if (dt <= 0) dt = 1.0f / 60.0f;
        if (dt > 0.0001f)
            fpsSmoothed = fpsSmoothed * 0.92f + (1.0f / dt) * 0.08f;

        // ---------------- events
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            ImGui_ImplSDL2_ProcessEvent(&ev);
            if (ev.type == SDL_QUIT) running = false;
            if (ev.type == SDL_WINDOWEVENT &&
                ev.window.event == SDL_WINDOWEVENT_FOCUS_LOST) {
                SDL_SetRelativeMouseMode(SDL_FALSE);
            }
            a.input.handleEvent(ev);
        }

        a.input.update();

        // ---------------- action edges
        bool escSwallowed = a.input.consumeSwallowedEsc();
        if (a.input.pressed(Action::Pause) && !escSwallowed) {
            a.menuOpen = !a.menuOpen;
            if (a.menuOpen) {
                a.input.cancelCapture();
                SDL_SetRelativeMouseMode(SDL_FALSE);
            }
            if (!a.menuOpen) a.needSave = true;  // save when leaving the menu too
        }
        if (a.input.pressed(Action::ToggleStats)) a.statsVisible = !a.statsVisible;

        if (!a.menuOpen) {
            if (a.input.pressed(Action::LaunchBall)) a.sim.launchBall(a.settings.freeplay);
            if (a.input.pressed(Action::Dribble)) a.sim.startDribble();
            if (a.input.pressed(Action::TakePosition)) {
                a.sim.takePosition(a.settings.freeplay.takePositionPreset);
                a.camResetPending = true;
            }
        }

        // ball cam mode
        if (a.settings.cam.ballCamToggle) {
            if (a.input.pressed(Action::BallCam))
                a.cam.setMode(a.cam.mode() == CamMode::Ball ? CamMode::Car : CamMode::Ball);
        } else {
            a.cam.setMode(a.input.held(Action::BallCam) ? CamMode::Ball : CamMode::Car);
        }

        // mouse swivel: relative mode while playing
        bool wantRelative = !a.menuOpen && a.settings.cam.mouseSwivel;
        static int lastRelative = -1;
        int rel = wantRelative ? 1 : 0;
        if (rel != lastRelative) {
            SDL_SetRelativeMouseMode(rel ? SDL_TRUE : SDL_FALSE);
            lastRelative = rel;
        }

        // ---------------- controls + sim
        float steerVisual = 0;
        if (a.menuOpen) {
            a.sim.car()->controls = RocketSim::CarControls();
        } else {
            a.input.buildControls(a.sim.car()->controls);
            // mirror buildControls' steer for front-wheel visuals
            float sx = 0, sy = 0;
            a.input.leftStick(sx, sy);
            steerVisual = clampf(
                a.input.value(Action::SteerRight) - a.input.value(Action::SteerLeft) +
                    sx * a.settings.ctrl.steerSens,
                -1, 1);
        }

        // ---------------- scripted self-test (CARBALLER_SELFTEST=1)
        // Phase A: full throttle + steer right (verify nose drifts right on screen).
        // Phase B: brake to stop, then jump -> front flip -> hold opposite pitch
        //          (flip cancel must arrest rotation and never auto-roll/half-flip).
        static float stT = 0.0f;
        if (selfTest && !a.menuOpen) {
            stT += dt;
            a.cam.setMode(CamMode::Ball);  // fixed-view ground truth for nose drift
            auto& c = a.sim.car()->controls;
            c = RocketSim::CarControls();
            if (stT < 1.6f) {
                c.throttle = 1.0f;
                c.steer = 1.0f;  // steer right
            } else if (stT < 3.0f) {
                c.throttle = -1.0f;  // brake to a stop
            } else if (stT < 7.5f) {
                float u = stT - 3.0f;
                if (u < 0.05f) c.jump = true;                       // jump 1
                else if (u < 0.10f) {}                              // release
                else if (u < 0.16f) { c.jump = true; c.pitch = -1; } // dodge: front flip
                else if (u < 0.55f) c.pitch = -1;                   // let flip develop
                else c.pitch = 1;                                   // opposite pitch: cancel
            } else if (stT < 16.0f) {
                // visual phase: reset to kickoff, drive + boost toward the ball
                // (screenshots validate ball-cam centering + boost particles)
                static bool stReset = false;
                if (!stReset) {
                    a.sim.takePosition(0);
                    a.camResetPending = true;
                    stReset = true;
                }
                c.throttle = 1.0f;
                c.boost = true;
            } else {
                std::fprintf(stderr, "[st] selftest complete\n");
                running = false;
            }
            steerVisual = c.steer;
        }

        a.sim.paused = a.menuOpen;
        a.sim.advance(dt);

        SimSnapshot snap = a.sim.snapshot(a.sim.paused ? 1.0f
                                                       : clampf(float(a.sim.accumAlpha()), 0, 1));

        if (a.camResetPending) {
            a.cam.reset(snap);
            a.camResetPending = false;
        }

        // camera
        float swX = 0, swY = 0;
        a.input.swivel(swX, swY);
        a.cam.update(dt, a.settings.cam, snap, swX, swY);

        // particles
        a.particles.update(dt);
        if (!a.menuOpen && snap.boosting) {
            float rate = a.settings.gfx.particleQuality == 0 ? 220.0f
                        : a.settings.gfx.particleQuality == 1 ? 380.0f
                                                              : 560.0f;
            a.boostAccum += rate * dt;
            V3 exhaust = snap.carPos + snap.carF * -44.0f + snap.carU * 16.0f;
            V3 dirBack = snap.carF * -1.0f;
            while (a.boostAccum >= 1.0f) {
                a.boostAccum -= 1.0f;
                a.particles.spawnBoost(exhaust, dirBack, 1);
            }
        }

        // ---------------- render
        int dw = 0, dh = 0;
        SDL_GL_GetDrawableSize(a.window, &dw, &dh);

        RenderParams rp;
        rp.width = dw;
        rp.height = dh;
        rp.dt = dt;
        rp.snap = &snap;
        rp.steerInput = steerVisual;
        rp.fovX = a.settings.cam.fov * (float)M_PI / 180.0f;
        rp.showBallRing = a.settings.cam.ballFloorProjection;
        rp.showShadows = true;
        a.renderer.render(a.cam, rp, a.particles);

        // ---------------- self-test telemetry
        if (selfTest) {
            static float nextLog = 0.1f;
            if (stT >= nextLog) {
                const auto& c = a.sim.car()->controls;
                float cx = 0, cy = 0, nx = 0, ny = 0;
                bool ok = a.renderer.project(snap.carPos, cx, cy) &&
                          a.renderer.project(snap.carPos + snap.carF * 300.0f, nx, ny);
                float yaw = std::atan2(snap.carF.y, snap.carF.x);
                V3 db = snap.ballPos - snap.carPos;
                float dist = std::sqrt(db.x * db.x + db.y * db.y + db.z * db.z);
                float bx = 0, by = 0;
                bool okB = a.renderer.project(snap.ballPos, bx, by);
                std::fprintf(stderr,
                             "[st] t=%5.2f yaw=%6.2f Fz=%+.2f Uz=%+.2f Rz=%+.3f "
                             "pR=%+6.2f fR=%+5.2f yR=%+6.2f d=%4.0f ndx=%s%+5.0f "
                             "ball=%s%3.0f,%3.0f fps=%3.0f "
                             "ctl(t%+.0f s%+.0f p%+.0f j%.0f)\n",
                             stT, yaw, snap.carF.z, snap.carU.z, snap.carR.z,
                             snap.carAngVel.dot(snap.carR),
                             snap.carAngVel.dot(snap.carF),
                             snap.carAngVel.dot(snap.carU),
                             dist, ok ? "" : "na ", ok ? nx - cx : 0.0f,
                             okB ? "" : "na ", okB ? bx : 0.0f, okB ? by : 0.0f,
                             fpsSmoothed,
                             c.throttle, c.steer, c.pitch, c.jump ? 1.0f : 0.0f);
                nextLog += 0.1f;
            }
        }

        // ---------------- UI
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        a.hud.draw(snap, a.cam, a.renderer, a.settings, dw, dh, fpsSmoothed,
                   a.statsVisible, a.menuOpen);

        if (a.menuOpen) {
            // pause hint
            ImGui::SetNextWindowPos(ImVec2(dw * 0.5f, 24), ImGuiCond_Always, ImVec2(0.5f, 0));
            ImGui::SetNextWindowBgAlpha(0.55f);
            if (ImGui::Begin("##pause", nullptr,
                             ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
                                 ImGuiWindowFlags_AlwaysAutoResize)) {
                ImGui::TextUnformatted("PAUSED  -  Esc to resume");
            }
            ImGui::End();

            bool pOpen = true;
            if (a.settingsUI.draw(a.settings, a.input, &pOpen)) {
                a.needSave = true;
                a.sim.applyControlSettings(a.settings.ctrl);
                applyGraphics(a);
            }
            if (!pOpen) {
                a.menuOpen = false;
                a.needSave = true;
            }
        }

        if (a.needSave) {
            a.settings.binds = a.input.bindings();
            a.settings.save("settings.json");
            a.needSave = false;
        }

        ImGui::Render();
        glViewport(0, 0, dw, dh);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        SDL_GL_SwapWindow(a.window);

        // ---------------- frame cap (when vsync is off)
        if (!a.settings.gfx.vsync && a.settings.gfx.fpsCap > 0) {
            uint64_t end = SDL_GetPerformanceCounter();
            double target = 1.0 / double(a.settings.gfx.fpsCap);
            double elapsed = double(end - nowCount) / double(prevFreq);
            if (elapsed < target) {
                double remain = (target - elapsed) * 1000.0;
                if (remain > 0.5)
                    SDL_Delay(uint32_t(remain));
                else
                    std::this_thread::yield();
            }
        }
    }

    // ---------------- shutdown
    a.settings.binds = a.input.bindings();
    a.settings.save("settings.json");

    a.renderer.shutdown();
    a.sim.shutdown();
    a.input.shutdown();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_GL_DeleteContext(a.glctx);
    SDL_DestroyWindow(a.window);
    SDL_Quit();
    return 0;
}
