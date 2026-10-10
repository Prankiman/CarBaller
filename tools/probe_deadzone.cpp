// Deadzone shape probe: exercises applyDeadzoneShape() - Rocket League's
// "Deadzone Shape" math (src/input/input.cpp) - for all three shapes across
// the stick range, and pins down the properties that make them different:
//
//   cross   (RL default) per-axis: each component is deadzoned and rescaled
//           on its own, so a component inside the deadzone drops out while
//           the other passes through, and the vector is pulled towards the
//           dominant axis.
//   square  the SAME box-shaped neutral region, but one uniform scale for
//           the whole vector, so the stick's direction survives exactly.
//   circle  radial: direction survives and the output magnitude is capped at
//           1, so a diagonal can never outrun a straight push.
//
// Build (SDL2 only because input.cpp lives with it):
//   c++ -std=c++20 -O2 -w -I. -Isrc -Ithird_party $(sdl2-config --cflags) \
//       tools/probe_deadzone.cpp src/input/input.cpp src/settings/settings.cpp \
//       $(sdl2-config --libs) -o probe_deadzone && ./probe_deadzone
// exit 0 = all checks ok
#include "input/input.h"

#include <cmath>
#include <cstdio>

namespace {
int failed = 0;

void check(bool ok, const char* what, const char* detail) {
    if (!ok) failed++;
    std::printf("%-52s %s  %s\n", what, ok ? "ok  " : "FAIL", detail);
}

float deg(float r) { return r * 57.2957795f; }
float angleOf(float x, float y) { return std::atan2(y, x); }
// Shortest angular difference in degrees.
float angDiff(float x0, float y0, float x1, float y1) {
    float d = angleOf(x0, y0) - angleOf(x1, y1);
    while (d > (float)M_PI) d -= 2 * (float)M_PI;
    while (d < -(float)M_PI) d += 2 * (float)M_PI;
    return std::fabs(deg(d));
}
float mag(float x, float y) { return std::sqrt(x * x + y * y); }
}  // namespace

int main() {
    const int kShapes[] = {DeadzoneCross, DeadzoneSquare, DeadzoneCircle};
    const float DZ = 0.20f;  // RL default controller deadzone

    // ---- 1. neutral at the centre, for every shape
    bool center = true;
    for (int s : kShapes) {
        float ox = 9, oy = 9;
        applyDeadzoneShape(DZ, s, 0.0f, 0.0f, ox, oy);
        if (ox != 0 || oy != 0) center = false;
    }
    check(center, "centre (0,0) is neutral for all three shapes", "ox=oy=0");

    // ---- 2. the dead regions differ where box meets circle: a shallow
    // diagonal sits OUTSIDE a circle of the same dz but INSIDE the box.
    {
        float cx, cy, sx, sy, xx, xy;
        applyDeadzoneShape(DZ, DeadzoneCircle, 0.15f, 0.15f, cx, cy);
        applyDeadzoneShape(DZ, DeadzoneSquare, 0.15f, 0.15f, sx, sy);
        applyDeadzoneShape(DZ, DeadzoneCross, 0.15f, 0.15f, xx, xy);
        const bool ok = mag(cx, cy) > 0 && sx == 0 && sy == 0 && xx == 0 && xy == 0;
        char d[128];
        std::snprintf(d, sizeof d,
                      "(0.15,0.15): circle %.3f live, square+cross dead",
                      mag(cx, cy));
        check(ok, "box (cross/square) is dead where the circle is not", d);
    }
    {
        bool allDead = true;
        for (int s : kShapes) {
            float ox = 9, oy = 9;
            applyDeadzoneShape(DZ, s, 0.10f, 0.10f, ox, oy);
            if (ox != 0 || oy != 0) allDead = false;
        }
        check(allDead, "deep inside (0.10,0.10) is dead for all three", "ox=oy=0");
    }

    // ---- 3. a single bound axis behaves IDENTICALLY under every shape
    // (raw, 0) is how bound stick axes are read - InputSystem::rawParts.
    {
        bool same = true;
        float ref = 0, t = 0;
        applyDeadzoneShape(DZ, DeadzoneCross, 0.75f, 0.0f, ref, t);
        for (int s : kShapes) {
            float ox = 9, oy = 9;
            applyDeadzoneShape(DZ, s, 0.75f, 0.0f, ox, oy);
            if (std::fabs(ox - ref) > 1e-6f || oy != 0) same = false;
        }
        char d[64];
        std::snprintf(d, sizeof d, "all give x=%.4f at raw 0.75", ref);
        check(same, "one-axis input is shape-independent", d);
    }

    // ---- 4. direction: square and circle keep the stick's angle exactly,
    // cross pulls it towards the dominant axis. (0.9, 0.3) = 18.43 deg.
    {
        const float X = 0.9f, Y = 0.3f;
        float sqx, sqy, cix, ciy, crx, cry;
        applyDeadzoneShape(DZ, DeadzoneSquare, X, Y, sqx, sqy);
        applyDeadzoneShape(DZ, DeadzoneCircle, X, Y, cix, ciy);
        applyDeadzoneShape(DZ, DeadzoneCross, X, Y, crx, cry);
        const float dSq = angDiff(X, Y, sqx, sqy);
        const float dCi = angDiff(X, Y, cix, ciy);
        const float dCr = angDiff(X, Y, crx, cry);
        char d[160];
        std::snprintf(d, sizeof d, "angle error: square %.3f, circle %.3f, cross %.2f deg",
                      dSq, dCi, dCr);
        check(dSq < 0.01f && dCi < 0.01f && dCr > 1.0f,
              "square+circle preserve direction, cross pulls to the axis", d);
        std::printf("      (0.9,0.3) -> cross (%.3f,%.3f) | square (%.3f,%.3f)"
                    " | circle (%.3f,%.3f)\n",
                    crx, cry, sqx, sqy, cix, ciy);
    }

    // ---- 5. full sweep: no NaN, no component past +-1, magnitudes inside
    // what each shape is defined to produce (circle <= 1, box shapes <= sqrt2).
    {
        bool bounds = true, circleCap = true;
        for (float dz : {0.0f, 0.20f, 0.75f}) {
            for (int s : kShapes) {
                for (int i = -10; i <= 10; i++) {
                    for (int j = -10; j <= 10; j++) {
                        const float x = i / 10.0f, y = j / 10.0f;
                        float ox = 9, oy = 9;
                        applyDeadzoneShape(dz, s, x, y, ox, oy);
                        if (!std::isfinite(ox) || !std::isfinite(oy) ||
                            std::fabs(ox) > 1.00001f || std::fabs(oy) > 1.00001f)
                            bounds = false;
                        const float m = mag(ox, oy);
                        if (s == DeadzoneCircle && m > 1.00001f) circleCap = false;
                        if (s != DeadzoneCircle && m > 1.4143f) bounds = false;
                    }
                }
            }
        }
        check(bounds, "21x21 sweep x 3 dz x 3 shapes: finite, |component| <= 1",
              "no NaN, no boosted component");
        check(circleCap, "circle output magnitude stays <= 1 everywhere",
              "|out| capped");
    }

    // ---- 6. max deadzone still reaches full throw straight out
    {
        bool full = true;
        for (int s : kShapes) {
            float ox = 0, oy = 0;
            applyDeadzoneShape(0.75f, s, 1.0f, 0.0f, ox, oy);
            if (std::fabs(ox - 1.0f) > 1e-5f) full = false;
        }
        check(full, "deadzone 0.75, stick at 1.0 -> output 1.0 on every shape",
              "ox=1.0");
    }

    // ---- 7. deadzone 0.00: box shapes are the identity. Circle is the
    // identity too while the stick is inside the unit circle - but it keeps
    // normalising the CORNER of the square (|in| > 1) back down to magnitude
    // 1, which is the whole point of a radial shape.
    {
        bool ident = true, cap = true;
        for (int s : {DeadzoneCross, DeadzoneSquare}) {
            for (int i = -10; i <= 10; i += 2) {
                for (int j = -10; j <= 10; j += 2) {
                    const float x = i / 10.0f, y = j / 10.0f;
                    float ox = 9, oy = 9;
                    applyDeadzoneShape(0.0f, s, x, y, ox, oy);
                    if (std::fabs(ox - x) > 1e-5f || std::fabs(oy - y) > 1e-5f)
                        ident = false;
                }
            }
        }
        for (int i = -10; i <= 10; i += 2) {
            for (int j = -10; j <= 10; j += 2) {
                const float x = i / 10.0f, y = j / 10.0f;
                float ox = 9, oy = 9;
                applyDeadzoneShape(0.0f, DeadzoneCircle, x, y, ox, oy);
                if (mag(x, y) <= 1.0f) {
                    if (std::fabs(ox - x) > 1e-5f || std::fabs(oy - y) > 1e-5f)
                        ident = false;
                } else if (mag(ox, oy) > 1.00001f) {
                    cap = false;
                }
            }
        }
        check(ident, "deadzone 0.00 passes the stick through untouched",
              "out == in (circle: within the unit circle)");
        check(cap, "deadzone 0.00, circle still folds the corner to |out| = 1",
              "(1,1) -> (0.707,0.707)");
    }

    // ---- 8. the practical difference on an ASYMMETRIC push: cross crushes
    // the minor axis (it is near the deadzone edge), square keeps it - the
    // stick's direction is the thing square protects. Equal on a perfectly
    // symmetric diagonal, where the two coincide by construction.
    {
        const float X = 0.9f, Y = 0.3f;
        float crx, cry, sqx, sqy;
        applyDeadzoneShape(DZ, DeadzoneCross, X, Y, crx, cry);
        applyDeadzoneShape(DZ, DeadzoneSquare, X, Y, sqx, sqy);
        char d[128];
        std::snprintf(d, sizeof d, "minor axis out: cross %.3f, square %.3f (in 0.30)",
                      cry, sqy);
        check(sqy > cry + 0.1f && std::fabs(sqx - crx) < 0.05f,
              "square keeps the minor axis cross would crush", d);
    }

    std::printf("\nshapes: cross (default, per-axis) | square (box, direction-true)"
                " | circle (radial, capped)\n");
    std::printf("%s\n", failed ? "DEADZONE SHAPE REGRESSION" : "all deadzone shape checks ok");
    return failed ? 1 : 0;
}
