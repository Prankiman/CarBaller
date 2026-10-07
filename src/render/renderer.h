#pragma once
// Scene renderer: lit pass (arena, car, wheels, ball), blended pass
// (markings, indicators, blob shadows), additive particle pass.

#include "../camera/rl_camera.h"
#include "../fx/particles.h"
#include "../maploader/map_data.h"
#include "assets.h"

struct RenderParams {
    int width = 1280, height = 720;
    float dt = 1.0f / 60;
    const SimSnapshot* snap = nullptr;
    float steerInput = 0;     // -1..1, for front-wheel visuals
    float fovX = 110.0f * (float)M_PI / 180.0f;  // horizontal FOV (radians)
    bool showBallRing = true;
    bool showShadows = true;
    bool showHitboxes = false;     // outline car/ball physics hitboxes
    bool showFlipReset = true;     // RL's flip reset disc under the car
    float wallOpacity = 1.0f;   // 0..1 shell opacity when camera is outside
};

class Renderer {
public:
    bool init(const std::string& meshDir, const std::string& modelDir);
    void shutdown();

    // Hitbox outline geometry (Settings > Graphics > Show Hitboxes).
    // carSize/carOffset: the physics box in car-local uu (RocketSim config);
    // ballRadius: collision radius in uu. Zeroed dims disable the outlines.
    void setHitboxDims(const V3& carSize, const V3& carOffset, float ballRadius);

    // Custom map geometry (Map tab): one VBO per unique mesh, drawn once per
    // placement with that placement's transform. While active the default
    // arena, its shell and its field markings are not drawn at all. Passing
    // null (or an empty map) restores the arena.
    void setMap(const maploader::MapData* data);
    bool hasMap() const { return !mapInst_.empty(); }

    void render(const RLCamera& cam, const RenderParams& p, const ParticleSystem& ps);

    // World -> screen (pixels). Returns false when behind the camera.
    bool project(const V3& world, float& sx, float& sy) const;

    float lastFovX = 110.0f * (float)M_PI / 180.0f;

private:
    void drawLit(const GpuMesh& mesh, const M4& model, int mode, float alpha = 1.0f);
    void drawUnlit(const GpuMesh& mesh, const M4& model, float alphaMul,
                   bool additive, bool textured = false);

    // One drawn instance of custom map geometry: its local -> world transform
    // plus the world AABB that transform produces, for frustum culling.
    struct MapInst {
        M4 xform;
        V3 bmin, bmax;
        uint32_t mesh = 0;
    };
    void buildFrustum();
    bool aabbVisible(const V3& mn, const V3& mx) const;

    GameAssets assets_;
    Shader lit_, unlit_;
    M4 view_, proj_;
    int w_ = 0, h_ = 0;

    std::vector<GpuMesh> mapMesh_;   // one per MapShape
    std::vector<MapInst> mapInst_;   // one per visible placement
    float mapFar_ = 0;               // far-plane extent needed by the map
    float frustum_[6][4] = {};       // clip-space planes, rebuilt each frame

    GpuMesh particleMesh_;
    GpuMesh hitboxCar_, hitboxBall_;
    float wheelSpin_ = 0;
};
