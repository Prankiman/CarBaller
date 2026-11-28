#include "hud.h"

#include <imgui.h>
#include <cfloat>
#include <cmath>
#include <cstdio>

namespace {

ImU32 col4(int r, int g, int b, int a) { return IM_COL32(r, g, b, a); }

void arc(ImDrawList* dl, ImVec2 c, float radius, float a0, float a1, ImU32 color, float thick) {
    dl->PathClear();
    dl->PathArcTo(c, radius, a0, a1, 48);
    dl->PathStroke(color, false, thick);
}

// Custom maps: show every checkpoint of the loaded map, with the one to head
// for picked out. Their routes run for kilometres across maps made of floating
// platforms, so without markers the proximity trigger is pure guesswork.
void checkpointMarkers(ImDrawList* dl, ImFont* font, const Renderer& renderer, const Sim& sim,
                       const SimSnapshot& snap, int w, int h) {
    if (sim.sessionCount() < 2) return;  // default arena: nothing to show
    const std::vector<maploader::MapSession>& sessions = sim.mapData().sessions;
    int cur = sim.sessionIndex();
    if (cur < 0) cur = 0;
    if (cur >= (int)sessions.size()) cur = (int)sessions.size() - 1;
    const int nxt = cur + 1 < (int)sessions.size() ? cur + 1 : -1;

    // the rest of the route, dim and small
    for (size_t i = 0; i < sessions.size(); i++) {
        if ((int)i == cur || (int)i == nxt) continue;
        float sx, sy;
        if (!renderer.project(sessions[i].carPos, sx, sy)) continue;
        if (sx < -30.0f || sy < -30.0f || sx > w + 30.0f || sy > h + 30.0f) continue;
        const float dist = (sessions[i].carPos - snap.carPos).len();
        const float fade = clampf(1.0f - dist / 80000.0f, 0.35f, 1.0f);
        dl->AddCircle(ImVec2(sx, sy), 5.0f, col4(150, 200, 255, int(80 * fade)), 16, 1.5f);
    }

    // where the car is standing right now
    if (cur < (int)sessions.size()) {
        float sx, sy;
        if (renderer.project(sessions[(size_t)cur].carPos, sx, sy) && sx >= 0.0f && sy >= 0.0f &&
            sx <= w && sy <= h)
            dl->AddCircle(ImVec2(sx, sy), 9.0f, col4(120, 220, 255, 190), 24, 2.0f);
    }

    if (nxt < 0) return;  // end of the route
    const maploader::MapSession& s = sessions[(size_t)nxt];
    const float dist = (s.carPos - snap.carPos).len();
    float sx = 0.0f, sy = 0.0f;
    bool onScreen =
        renderer.project(s.carPos, sx, sy) && sx > 40.0f && sy > 40.0f && sx < w - 40.0f &&
        sy < h - 40.0f;
    if (!onScreen) {
        // Off the side of the screen: pin it to the frame so the way round is
        // still visible. (Behind the camera it projects to nothing at all.)
        if (!renderer.project(s.carPos, sx, sy)) return;
        sx = clampf(sx, 48.0f, float(w) - 48.0f);
        sy = clampf(sy, 48.0f, float(h) - 48.0f);
    }
    const ImU32 orange = col4(255, 168, 44, 235);
    dl->AddCircleFilled(ImVec2(sx, sy), onScreen ? 7.0f : 5.0f, orange, 24);
    if (onScreen) dl->AddCircle(ImVec2(sx, sy), 13.0f, col4(255, 168, 44, 120), 32, 2.0f);
    char buf[32];
    std::snprintf(buf, sizeof buf, "%s %d m", onScreen ? "NEXT" : "", int(dist / 100.0f + 0.5f));
    const ImVec2 ts = font->CalcTextSizeA(15.0f, FLT_MAX, 0.0f, buf);
    dl->AddText(ImVec2(sx - ts.x * 0.5f, sy + (onScreen ? 17.0f : 11.0f)), orange, buf);
}

// Built-in aim trainer: streak board at the top centre, with a short HIT/MISS
// flash while the next attempt is being set up.
void aimTrainerHud(ImDrawList* dl, ImFont* font, const AimTrainerState& a, int w) {
    if (!a.active) return;

    char streak[32];
    std::snprintf(streak, sizeof streak, "%d", a.streak);
    char sub[96];
    const int acc = a.attempts > 0 ? (a.hits * 100 + a.attempts / 2) / a.attempts : 0;
    std::snprintf(sub, sizeof sub, "BEST %d     HITS %d/%d     %d%%", a.best, a.hits,
                  a.attempts, acc);

    const float cx = float(w) * 0.5f;
    const ImU32 green = col4(80, 255, 105, 240);
    const ImU32 dim = col4(205, 215, 230, 190);

    // board
    dl->AddRectFilled(ImVec2(cx - 150.0f, 12.0f), ImVec2(cx + 150.0f, 104.0f),
                      col4(8, 12, 16, 185), 10.0f);
    dl->AddRect(ImVec2(cx - 150.0f, 12.0f), ImVec2(cx + 150.0f, 104.0f),
                col4(80, 255, 105, 70), 10.0f, 0, 1.5f);

    auto centered = [&](const char* text, float size, float y, ImU32 c) {
        const ImVec2 ts = font->CalcTextSizeA(size, FLT_MAX, 0.0f, text);
        dl->AddText(font, size, ImVec2(cx - ts.x * 0.5f, y), c, text);
    };
    centered("AIM TRAINER", 13.0f, 20.0f, green);
    centered(streak, 38.0f, 34.0f, col4(255, 255, 255, 245));
    centered(sub, 13.0f, 80.0f, dim);

    if (a.lastResult != 0 && a.lastResultAge < 0.55f) {
        const char* text = a.lastResult > 0 ? "HIT!" : "MISS";
        const ImU32 c = a.lastResult > 0 ? green : col4(255, 85, 75, 240);
        const ImVec2 ts = font->CalcTextSizeA(34.0f, FLT_MAX, 0.0f, text);
        dl->AddText(font, 34.0f, ImVec2(cx - ts.x * 0.5f, 124.0f), c, text);
    }
}

}  // namespace

void HUD::draw(const SimSnapshot& snap, const RLCamera& cam, const Renderer& renderer,
               const Settings& settings, const Sim& sim, int w, int h, float fps,
               bool statsVisible, bool menuOpen) {
    if (w <= 0 || h <= 0) return;
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    ImFont* font = ImGui::GetFont();

    // ---------------- map checkpoints (only when a custom map is loaded)
    checkpointMarkers(dl, font, renderer, sim, snap, w, h);

    // ---------------- aim trainer streak board
    aimTrainerHud(dl, font, sim.aimState(), w);

    // ---------------- boost gauge (bottom right)
    {
        ImVec2 c(float(w) - 108.0f, float(h) - 96.0f);
        const float R = 58.0f;
        const float A0 = 0.75f * (float)M_PI;          // bottom-left of gap
        const float SPAN = 1.5f * (float)M_PI;
        arc(dl, c, R, A0, A0 + SPAN, col4(8, 10, 14, 170), 9.0f);
        float boost = clampf(snap.boost / 100.0f, 0, 1);
        if (boost > 0.001f) {
            arc(dl, c, R, A0, A0 + SPAN * boost,
                snap.supersonic ? col4(255, 214, 90, 250) : col4(255, 158, 34, 245), 9.0f);
        }
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%d", int(snap.boost + 0.5f));
        ImVec2 ts = font->CalcTextSizeA(30.0f, FLT_MAX, 0, buf);
        dl->AddText(font, 30.0f, ImVec2(c.x - ts.x * 0.5f, c.y - ts.y * 0.5f),
                    col4(255, 255, 255, 235), buf);
    }

    // ---------------- ball cam indicator (bottom left)
    if (settings.cam.ballCamIndicator && cam.mode() == CamMode::Ball && !menuOpen) {
        ImVec2 pos(24.0f, float(h) - 66.0f);
        float alpha = cam.transitioning() ? 0.65f : 1.0f;
        dl->AddRectFilled(ImVec2(pos.x - 4, pos.y - 4), ImVec2(pos.x + 138, pos.y + 34),
                          IM_COL32(10, 14, 20, int(150 * alpha)), 8.0f);
        dl->AddCircleFilled(ImVec2(pos.x + 14, pos.y + 13), 9.0f,
                            IM_COL32(235, 240, 248, int(230 * alpha)));
        dl->AddCircleFilled(ImVec2(pos.x + 17, pos.y + 16), 4.0f,
                            IM_COL32(120, 130, 145, int(200 * alpha)));
        dl->AddText(ImVec2(pos.x + 30, pos.y + 2),
                    IM_COL32(255, 255, 255, int(235 * alpha)), "BALL CAM");
    }

    // ---------------- ball target area (Settings > Tools): a hot-pink
    // latitude ring on the ball. The near half is solid; the far half is
    // dimmed rather than depth-clipped so the whole circle stays readable.
    if (settings.misc.ballTargetArea) {
        constexpr float kBallR = GameAssets::kBallRadius;
        const V3 c = snap.ballPos;
        const float t = std::fmax(-1.0f, std::fmin(1.0f, settings.misc.ballTargetY));
        const float zOff = t * kBallR;
        const float ringR = kBallR * std::sqrt(std::fmax(0.0f, 1.0f - t * t));
        const ImU32 nearCol = IM_COL32(255, 40, 160, 235);
        const ImU32 farCol = IM_COL32(255, 40, 160, 70);

        if (ringR < 2.0f) {
            // At either pole the circle collapses to a dot.
            float sx, sy;
            if (renderer.project(V3(c.x, c.y, c.z + zOff), sx, sy))
                dl->AddCircleFilled(ImVec2(sx, sy), 3.5f, nearCol, 12);
        } else {
            constexpr int kSegs = 64;
            bool havePrev = false;
            ImVec2 prev;
            for (int i = 0; i <= kSegs; i++) {
                const float a = 2.0f * (float)M_PI * float(i) / kSegs;
                const V3 p(c.x + std::cos(a) * ringR,
                           c.y + std::sin(a) * ringR,
                           c.z + zOff);
                float sx, sy;
                const bool ok = renderer.project(p, sx, sy);
                if (!ok) {
                    havePrev = false;
                    continue;
                }
                if (havePrev) {
                    // Midpoint normal tells whether this segment is on the
                    // hemisphere facing the camera.
                    const float am = 2.0f * (float)M_PI * (float(i) - 0.5f) / kSegs;
                    const V3 n(std::cos(am) * ringR, std::sin(am) * ringR, zOff);
                    const float facing = n.x * (cam.eye.x - c.x) +
                                         n.y * (cam.eye.y - c.y) +
                                         n.z * (cam.eye.z - c.z);
                    const bool near = facing >= 0.0f;
                    dl->AddLine(prev, ImVec2(sx, sy), near ? nearCol : farCol,
                                near ? 2.5f : 1.5f);
                }
                prev = ImVec2(sx, sy);
                havePrev = true;
            }
        }
    }

    // ---------------- hit marker (same ring tool): a cross where the car
    // touched the ball, green below the ring's Y level and red at/above it.
    if (settings.misc.ballTargetArea && settings.misc.hitMarker) {
        constexpr float kHitLife = 0.45f;
        float sx, sy;
        if (snap.ballHitAge < kHitLife &&
            renderer.project(snap.ballHitContact, sx, sy)) {
            const bool good = snap.ballHitY < settings.misc.ballTargetY;
            const float fade = 1.0f - snap.ballHitAge / kHitLife;
            const ImU32 col = good ? IM_COL32(60, 255, 120, int(255 * fade))
                                   : IM_COL32(255, 60, 70, int(255 * fade));
            const ImU32 shadow = IM_COL32(0, 0, 0, int(170 * fade));
            const float s = 13.0f;
            const ImVec2 a(sx - s, sy - s), b(sx + s, sy + s);
            const ImVec2 c(sx - s, sy + s), d(sx + s, sy - s);
            dl->AddLine(a, b, shadow, 6.0f);
            dl->AddLine(c, d, shadow, 6.0f);
            dl->AddLine(a, b, col, 3.0f);
            dl->AddLine(c, d, col, 3.0f);
        }
    }

    // ---------------- car -> ball arrow (car cam only)
    if (settings.cam.ballArrow && cam.mode() == CamMode::Car) {
        float cx, cy, bx, by;
        if (renderer.project(snap.carPos, cx, cy) && renderer.project(snap.ballPos, bx, by)) {
            const float margin = 46.0f;
            float ex = clampf(bx, margin, float(w) - margin);
            float ey = clampf(by, margin, float(h) - margin);
            bool clamped = (ex != bx) || (ey != by);

            float dx = ex - cx, dy = ey - cy;
            float len = std::sqrt(dx * dx + dy * dy);
            if (len > 60.0f) {
                dx /= len; dy /= len;
                // start a bit away from the car center
                float sx = cx + dx * 46.0f, sy = cy + dy * 46.0f;
                ImU32 lineCol = IM_COL32(255, 176, 60, clamped ? 200 : 220);
                dl->AddLine(ImVec2(sx, sy), ImVec2(ex, ey), lineCol, 3.0f);
                // arrow head
                ImVec2 pts[3];
                float ah = 16.0f, aw = 8.0f;
                pts[0] = ImVec2(ex + dx * ah, ey + dy * ah);
                pts[1] = ImVec2(ex - dx * ah * 0.2f - dy * aw, ey - dy * ah * 0.2f + dx * aw);
                pts[2] = ImVec2(ex - dx * ah * 0.2f + dy * aw, ey - dy * ah * 0.2f - dx * aw);
                dl->AddConvexPolyFilled(pts, 3, lineCol);
            }
        }
    }

    // ---------------- FPS counter (Settings > Tools)
    if (settings.misc.showFps) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "FPS %.0f", fps);
        const float fs = ImGui::GetFontSize();
        ImVec2 ts = font->CalcTextSizeA(fs, FLT_MAX, 0.0f, buf);
        ImVec2 pos(float(w) - ts.x - 16.0f, 12.0f);
        dl->AddText(font, fs, ImVec2(pos.x + 1.0f, pos.y + 1.0f),
                    IM_COL32(0, 0, 0, 170), buf);
        dl->AddText(font, fs, pos, IM_COL32(210, 230, 255, 220), buf);
    }

    // ---------------- stats
    if (statsVisible) {
        char buf[256];
        float speed = snap.carVel.len();
        float ballSpeed = snap.ballVel.len();
        // Game speed (Settings > Freeplay) only when it isn't the default,
        // so the overlay stays as short as possible during normal play.
        char gameSpeedLine[32] = "";
        if (std::fabs(settings.freeplay.gameSpeed - 100.0f) > 0.5f)
            std::snprintf(gameSpeedLine, sizeof(gameSpeedLine), "\nSPEED %.0f%%",
                          settings.freeplay.gameSpeed);
        std::snprintf(buf, sizeof(buf),
                      "FPS %.0f\nCAR %.0f uu/s (%.0f km/h)\nBALL %.0f uu/s\n%s%s%s",
                      fps, speed, speed * 0.036f, ballSpeed,
                      snap.onGround ? "GROUND" : (snap.flipping ? "FLIP" : "AIR"),
                      snap.supersonic ? "  SUPERSONIC" : "", gameSpeedLine);
        dl->AddText(ImVec2(16, 12), IM_COL32(210, 230, 255, 220), buf);
    }

    // ---------------- supersonic flare text (above boost gauge)
    if (snap.supersonic && !menuOpen) {
        char buf[8] = "MAX";
        ImFont* f = ImGui::GetFont();
        ImVec2 ts = f->CalcTextSizeA(18.0f, FLT_MAX, 0, buf);
        ImVec2 c(float(w) - 108.0f, float(h) - 96.0f);
        dl->AddText(f, 18.0f, ImVec2(c.x - ts.x * 0.5f, c.y - 58.0f - 30.0f),
                    IM_COL32(255, 255, 255, 150), buf);
    }
}
