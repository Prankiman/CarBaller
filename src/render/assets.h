#pragma once
// Procedural + data-driven game assets: arena (planes + RL .cmf collision
// meshes), low-poly Fennec-style car on the Octane hitbox, ball, markings.

#include "gl_utils.h"
#include "mesh_builder.h"

#include <string>

struct GameAssets {
    GpuMesh arena;        // floor/walls/ceiling planes + .cmf meshes (lit, world grid)
    GpuMesh markings;     // field lines (unlit)
    GpuMesh carBody;      // Fennec-ish body (lit)
    GpuMesh wheelFront;   // front wheels (lit), drawn 2x with steer+spin
    GpuMesh wheelBack;    // rear wheels (lit), drawn 2x with spin
    GpuMesh ball;         // UV sphere (lit, textured)
    GpuMesh indicator;    // ball floor-projection ring (unlit)
    GpuMesh shadowDisc;   // soft blob disc radius 1 (unlit)

    Texture2D ballTex;

    bool build(const std::string& meshDir);
    void destroy();
};
