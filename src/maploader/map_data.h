#pragma once
// Geometry/collision/session data produced by the map loader.
// This header is deliberately free of GL and Bullet dependencies so the parser
// can run on a worker thread.

#include <cstdint>
#include <string>
#include <vector>

#include "../core/math.h"

namespace maploader {

// Rigid-body collision channel of a placement (UE3 RBChannel enum mapped onto
// RocketSim/Bullet collision groups).
enum class CollChannel : uint8_t {
    All = 0,         // blocks car and ball
    VehicleOnly = 1, // RBCC_VehicleBlocker: car yes, ball no
    BallOnly = 2,    // RBCC_BallBlocker: ball yes, car no
    None = 3,        // no rigid body collision at all
};

// Byte-identical to Vertex in render/gl_utils.h (checked with a static_assert
// where the renderer consumes it).
struct MapVertex {
    float px, py, pz;
    float nx, ny, nz;
    float u, v;
    float r, g, b, a;
};

// One unique piece of geometry in local space: a UStaticMesh LOD0 or a convex
// collision element. Many placements may share a shape.
struct MapShape {
    std::vector<MapVertex> verts;
    std::vector<uint32_t> idx;   // triangle list
    V3 boundsMin, boundsMax;     // local AABB (for an early-out / sanity check)
    std::string name;            // source object name (debugging)
};

struct MapPlacement {
    int32_t shape = -1;
    M4 xform;                    // local -> world
    CollChannel coll = CollChannel::All;
    bool visible = true;         // drawn
    bool collide = false;        // added as a Bullet static body
};

// A spawn/checkpoint: car pose + where the ball is placed when it is reached.
struct MapSession {
    std::string name;            // "01", "02", ... or "PlayerStart"
    V3 carPos;
    float carYawRad = 0.f;
    bool hasCar = false;
    V3 ballPos;
    bool hasBall = false;
    // Built by sanitizeSessions() from the map's PathNodes rather than from a
    // numbered spawn series, and placed on solid ground by it.
    bool derived = false;
    // How close the car must get for this checkpoint to count as reached.
    float reach = 600.f;
};

struct MapStats {
    int32_t exports = 0;
    int32_t objectsParsed = 0;
    int32_t objectsFailed = 0;
    int32_t staticMeshes = 0;
    int32_t staticMeshesFailed = 0;
    int32_t shapes = 0;
    int32_t placements = 0;
    int32_t collidePlacements = 0;
    int32_t visiblePlacements = 0;
    int32_t collisionTris = 0;
    int32_t renderTris = 0;
    int32_t convexElems = 0;
    int32_t bspTris = 0;         // triangles taken from the level's BSP model
    int32_t uniqueTris = 0;
    int32_t sessions = 0;
    int32_t derivedSessions = 0;  // checkpoints built from the map's PathNodes
    int32_t spawnActors = 0;
    int32_t ballSpawns = 0;
    int32_t chan[4] = {0, 0, 0, 0};  // indexed by CollChannel
    V3 boundsMin, boundsMax;
    bool hasBounds = false;
};

struct MapData {
    std::string path;
    std::string fileName;        // "r3.udk" (used in error toasts)

    std::vector<MapShape> shapes;
    std::vector<MapPlacement> placements;
    std::vector<MapSession> sessions;

    V3 boundsMin, boundsMax;     // world AABB over all placements
    bool hasBounds = false;

    MapStats stats;

    bool empty() const { return placements.empty(); }
};

}  // namespace maploader
