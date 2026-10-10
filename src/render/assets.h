#pragma once
// Procedural + data-driven game assets: arena (planes + RL .cmf collision
// meshes), low-poly Fennec-style car on the Octane hitbox, ball, markings.

#include "gl_utils.h"
#include "mesh_builder.h"

#include <string>
#include <vector>

// One wheel attach point in car space; ground contact at rest is pos.z - r.
struct WheelMount {
    V3 pos;
    float r;
    bool front;
};

struct GameAssets {
    GpuMesh arenaFloor;  // floor quad only: always opaque, drawn first
    GpuMesh arenaShell;  // ceiling, side walls, .cmf ramps/nets (see-through)
    GpuMesh markings;     // field lines (unlit)
    GpuMesh carBody;      // Fennec body (lit); procedural fallback
    GpuMesh wheelFront;   // front wheels (lit), drawn 2x with steer+spin
    GpuMesh wheelBack;    // rear wheels (lit), drawn 2x with spin
    WheelMount wheelMounts[4];  // FR, FL, RR, RL (from the model's arches)
    GpuMesh ball;         // UV sphere (lit, textured)
    GpuMesh indicator;    // ball floor-projection ring (unlit)
    GpuMesh indicatorInner;  // ring's inner circle: the ball's height cue,
                             // built at kInnerR and scaled down by the
                             // renderer from the ball's height off the floor
    GpuMesh shadowDisc;   // soft blob disc radius 1 (unlit)
    GpuMesh flipDisc;     // flip reset indicator: hollow disc beneath the car

    // Ball floor-indicator geometry (assets.cpp builds it, renderer.cpp
    // scales it). The outer ring's soft band is 74..91..108 - centred on
    // the ball's radius, so it reads as the ball's footprint - and its hole
    // is therefore radius 74, which kInnerR has to stay clear of.
    static constexpr float kBallRadius = 91.25f;  // soccar ball, uu
    static constexpr float kInnerR = 60.0f;       // inner circle at contact

    Texture2D ballTex;

    bool build(const std::string& meshDir, const std::string& modelDir);
    void destroy();
};
