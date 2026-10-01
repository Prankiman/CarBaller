#pragma once
// Scene renderer: lit pass (arena, car, wheels, ball), blended pass
// (markings, indicators, blob shadows), additive particle pass.

#include "../camera/rl_camera.h"
#include "../fx/particles.h"
#include "assets.h"

struct RenderParams {
    int width = 1280, height = 720;
    float dt = 1.0f / 60;
    const SimSnapshot* snap = nullptr;
    float steerInput = 0;     // -1..1, for front-wheel visuals
    float fovX = 110.0f * (float)M_PI / 180.0f;  // horizontal FOV (radians)
    bool showBallRing = true;
    bool showShadows = true;
    float wallOpacity = 1.0f;   // 0..1 shell opacity when camera is outside
};

class Renderer {
public:
    bool init(const std::string& meshDir, const std::string& modelDir);
    void shutdown();

    void render(const RLCamera& cam, const RenderParams& p, const ParticleSystem& ps);

    // World -> screen (pixels). Returns false when behind the camera.
    bool project(const V3& world, float& sx, float& sy) const;

    float lastFovX = 110.0f * (float)M_PI / 180.0f;

private:
    void drawLit(const GpuMesh& mesh, const M4& model, int mode, float alpha = 1.0f);
    void drawUnlit(const GpuMesh& mesh, const M4& model, float alphaMul,
                   bool additive, bool textured = false);

    GameAssets assets_;
    Shader lit_, unlit_;
    M4 view_, proj_;
    int w_ = 0, h_ = 0;

    GpuMesh particleMesh_;
    float wheelSpin_ = 0;
};
