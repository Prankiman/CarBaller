#include "map_physics.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <tuple>
#include <vector>

#include "RocketSim/src/RocketSim.h"
#include "RocketSim/libsrc/bullet3-3.24/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"

namespace maploader {
namespace {

using RocketSim::Arena;

// UE3 units -> RocketSim's Bullet world units (RocketSim's UU_TO_BT = 1/50).
constexpr float kUuToBt = 1.0f / 50.0f;

// RocketSim registers the car under (1 | DROPSHOT_FLOOR) and the ball under
// (1 | HOOPS_NET | DROPSHOT_TILE), both masking everything. Bullet resolves a
// pair as (a.mask & b.group) && (b.mask & a.group), so the three UE3 RBCC_*
// channels below are exactly the masks that reproduce their UE3 semantics.
constexpr int kGroupDefault = 1;                 // btBroadphaseProxy::DefaultFilter
constexpr int kBallExtraBits = (1 << 8) | (1 << 9);  // HOOPS_NET | DROPSHOT_TILE
constexpr int kCarExtraBits = (1 << 10);         // DROPSHOT_FLOOR

// Bullet units are 32-bit unless built with double precision.
static_assert(sizeof(btScalar) == 4, "map collision assumes single-precision Bullet");

// One unique Bullet mesh: local vertices with the placement's per-axis scale
// already baked in, so every placement sharing a scale can share the shape
// and carry only rotation + translation on its rigid body.
struct BuiltShape {
    std::vector<btScalar> verts;  // xyz, Bullet units
    std::vector<int32_t> tris;    // 3 indices per triangle
    btTriangleIndexVertexArray* iface = nullptr;
    btBvhTriangleMeshShape* shape = nullptr;
};

// An arena body saved while the map owns the collision, with the broadphase
// filter it was registered under so it can be restored verbatim.
struct SavedBody {
    btRigidBody* rb;
    int group;
    int mask;
};

// Placement transform split into "scale baked into shared geometry" and
// "rotation + translation on the rigid body".
struct Decomposed {
    bool ok = false;                 // false -> shear (or degenerate): bake it all
    float S[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};  // local -> shape space
    btTransform body;                // shape space -> world, Bullet units
};

// Splits M4's linear part into a right-handed orthonormal basis times a
// per-axis (possibly mirrored) scale. UE3 placements are always
// actorRot * actorScale * compRot * compScale; measured across three cooked
// maps every one of them stays orthogonal, so anything else falls back to
// baking the whole matrix into a placement-private shape.
Decomposed decompose(const M4& x) {
    Decomposed d;
    const btVector3 c0(x.m[0], x.m[1], x.m[2]);
    const btVector3 c1(x.m[4], x.m[5], x.m[6]);
    const btVector3 c2(x.m[8], x.m[9], x.m[10]);
    const float l0 = c0.length(), l1 = c1.length(), l2 = c2.length();

    if (l0 >= 1e-6f && l1 >= 1e-6f && l2 >= 1e-6f) {
        btVector3 u0 = c0 / l0, u1 = c1 / l1, u2 = c2 / l2;
        const float od =
            std::max({std::fabs(u0.dot(u1)), std::fabs(u0.dot(u2)), std::fabs(u1.dot(u2))});
        if (od <= 1e-3f) {
            float s0 = l0, s1 = l1, s2 = l2;
            if (u0.dot(u1.cross(u2)) < 0) {  // mirrored placement: keep the basis proper
                u0 = -u0;
                s0 = -s0;
            }
            d.S[0] = s0;
            d.S[4] = s1;
            d.S[8] = s2;
            d.body.setBasis(btMatrix3x3(u0.x(), u1.x(), u2.x(), u0.y(), u1.y(), u2.y(), u0.z(),
                                        u1.z(), u2.z()));
            d.ok = true;
        }
    }

    if (!d.ok) {  // shear / degenerate: the body carries translation only
        d.S[0] = x.m[0]; d.S[1] = x.m[4]; d.S[2] = x.m[8];
        d.S[3] = x.m[1]; d.S[4] = x.m[5]; d.S[5] = x.m[9];
        d.S[6] = x.m[2]; d.S[7] = x.m[6]; d.S[8] = x.m[10];
        d.body.setBasis(btMatrix3x3::getIdentity());
    }
    d.body.setOrigin(btVector3(x.m[12] * kUuToBt, x.m[13] * kUuToBt, x.m[14] * kUuToBt));
    return d;
}

// Placements whose scales differ only by float noise (their rotations perturb
// the column lengths) must still share a shape, so scales are keyed by log2
// with a 1/16384 relative step - a few orders of magnitude tighter than the
// geometry they are baked into, and far looser than the float error.
int32_t quantizeScale(float s) {
    const float a = std::fabs(s);
    if (a < 1e-30f) return INT32_MIN;
    return (int32_t)std::lround(std::log2((double)a) * 16384.0);
}

// (shape, |sx|, |sy|, |sz|, sign bits)
using ShapeKey = std::tuple<int32_t, int32_t, int32_t, int32_t, int32_t>;

// Map geometry -> Bullet collision groups. Returns false for channels that
// should not get a rigid body at all.
bool channelFilter(CollChannel c, int* group, int* mask) {
    switch (c) {
        case CollChannel::VehicleOnly:  // car yes, ball no
            *group = kGroupDefault;
            *mask = kCarExtraBits;
            return true;
        case CollChannel::BallOnly:  // ball yes, car no
            *group = kBallExtraBits;
            *mask = kBallExtraBits;
            return true;
        case CollChannel::All:
            *group = kGroupDefault;
            *mask = -1;
            return true;
        default:
            return false;
    }
}

// Builds one Bullet mesh from a MapShape, applying `S` (local -> shape space).
bool makeShape(const MapShape& src, const float S[9], BuiltShape* out, std::string* err) {
    const size_t nverts = src.verts.size();
    const size_t ntris = src.idx.size() / 3;
    if (nverts < 3 || ntris < 1) {
        *err = "shape '" + src.name + "' has no triangles";
        return false;
    }

    out->verts.resize(nverts * 3);
    for (size_t i = 0; i < nverts; i++) {
        const MapVertex& v = src.verts[i];
        const float px = v.px, py = v.py, pz = v.pz;
        out->verts[i * 3 + 0] = (btScalar)((S[0] * px + S[1] * py + S[2] * pz) * kUuToBt);
        out->verts[i * 3 + 1] = (btScalar)((S[3] * px + S[4] * py + S[5] * pz) * kUuToBt);
        out->verts[i * 3 + 2] = (btScalar)((S[6] * px + S[7] * py + S[8] * pz) * kUuToBt);
    }

    out->tris.resize(ntris * 3);
    for (size_t t = 0; t < ntris * 3; t++) {
        const uint32_t i = src.idx[t];
        if (i >= nverts) {
            *err = "shape '" + src.name + "' has an out-of-range index";
            return false;
        }
        out->tris[t] = (int32_t)i;
    }

    out->iface = new btTriangleIndexVertexArray((int)ntris, out->tris.data(), 3 * (int)sizeof(int32_t),
                                                (int)nverts, out->verts.data(),
                                                3 * (int)sizeof(btScalar));
    out->shape = new btBvhTriangleMeshShape(out->iface, /*useQuantizedAabbCompression=*/true);
    return true;
}

struct Job {
    const MapPlacement* pl = nullptr;
    int group = 0, mask = 0;
    Decomposed tr;
    bool shared = false;   // false -> the shape is private to this placement
    ShapeKey key = ShapeKey();
    int32_t shapeIdx = -1;
};

}  // namespace

struct MapCollision {
    std::vector<SavedBody> original;   // arena collision, removed while active
    std::vector<BuiltShape> shapes;    // ours
    std::vector<btRigidBody*> bodies;  // ours; also held by arena->_worldCollisionRBs
    bool stripped = false;

    ~MapCollision() {
        // Bodies are owned by the arena while applied and are removed (and
        // freed) by releaseMapCollision(); shapes are always ours.
        for (BuiltShape& s : shapes) {
            delete s.shape;
            delete s.iface;
        }
    }
};

MapCollision* applyMapCollision(Arena* arena, const MapData& data, std::string* err) {
    std::string local;
    if (!err) err = &local;
    err->clear();
    if (!arena) {
        *err = "no arena";
        return nullptr;
    }

    std::vector<Job> jobs;
    int sheared = 0;

    for (const MapPlacement& pl : data.placements) {
        if (!pl.collide || pl.shape < 0 || pl.shape >= (int32_t)data.shapes.size()) continue;
        Job job;
        job.pl = &pl;
        if (!channelFilter(pl.coll, &job.group, &job.mask)) continue;
        job.tr = decompose(pl.xform);
        if (job.tr.ok) {
            const float sx = job.tr.S[0], sy = job.tr.S[4], sz = job.tr.S[8];
            const int32_t signs = (sx < 0 ? 1 : 0) | (sy < 0 ? 2 : 0) | (sz < 0 ? 4 : 0);
            job.key = ShapeKey(pl.shape, quantizeScale(sx), quantizeScale(sy), quantizeScale(sz),
                               signs);
            job.shared = true;
        } else {
            sheared++;  // placement-private geometry (none seen in three maps)
        }
        jobs.push_back(job);
    }

    if (jobs.empty()) {
        *err = "map has no collidable geometry";
        return nullptr;
    }

    auto* h = new MapCollision();
    std::map<ShapeKey, int32_t> built;

    // One Bullet mesh per unique (geometry, scale); everything else shares it.
    for (Job& job : jobs) {
        if (job.shared) {
            const auto it = built.find(job.key);
            if (it != built.end()) {
                job.shapeIdx = it->second;
                continue;
            }
        }
        BuiltShape bs;
        std::string serr;
        if (!makeShape(data.shapes[(size_t)job.pl->shape], job.tr.S, &bs, &serr)) {
            *err = serr;
            delete h;
            return nullptr;
        }
        h->shapes.push_back(std::move(bs));
        job.shapeIdx = (int32_t)h->shapes.size() - 1;
        if (job.shared) built.emplace(job.key, job.shapeIdx);
    }

    // Everything below this point only touches the arena, and cannot fail.
    for (btRigidBody* rb : arena->_worldCollisionRBs) {
        int group = kGroupDefault, mask = -1;
        if (btBroadphaseProxy* proxy = rb->getBroadphaseHandle()) {
            group = proxy->m_collisionFilterGroup;
            mask = proxy->m_collisionFilterMask;
        }
        arena->_bulletWorld.removeRigidBody(rb);
        h->original.push_back({rb, group, mask});
    }
    h->stripped = true;

    size_t tris = 0;
    for (const BuiltShape& s : h->shapes) tris += s.tris.size() / 3;
    for (const Job& job : jobs) {
        BuiltShape* bs = &h->shapes[(size_t)job.shapeIdx];
        btRigidBody* rb = arena->_AddStaticCollisionShape(bs->shape, btVector3(0, 0, 0), job.group,
                                                          job.mask);
        rb->setWorldTransform(job.tr.body);
        arena->_bulletWorld.updateSingleAabb(rb);
        h->bodies.push_back(rb);
    }

    if (std::getenv("MAPLOADER_VERBOSE")) {
        size_t verts = 0;
        for (const BuiltShape& s : h->shapes) verts += s.verts.size() / 3;
        std::fprintf(stderr,
                     "[maploader] collision: %zu bodies from %zu shapes "
                     "(%zu verts, %zu tris), %zu arena bodies stripped, %d sheared\n",
                     h->bodies.size(), h->shapes.size(), verts, tris, h->original.size(), sheared);
    }
    return h;
}

void releaseMapCollision(Arena* arena, MapCollision* h) {
    if (!h) return;
    if (arena) {
        if (h->stripped) {  // put the arena back exactly as we found it
            for (SavedBody& s : h->original)
                if (s.rb) arena->_bulletWorld.addRigidBody(s.rb, s.group, s.mask);
            h->stripped = false;
            h->original.clear();
        }
        std::vector<btRigidBody*>& owned = arena->_worldCollisionRBs;
        for (btRigidBody* rb : h->bodies) {
            arena->_bulletWorld.removeRigidBody(rb);
            owned.erase(std::remove(owned.begin(), owned.end(), rb), owned.end());
            delete rb;
        }
    }
    h->bodies.clear();   // if arena == nullptr they stay arena-owned for ~Arena
    for (BuiltShape& s : h->shapes) {
        delete s.shape;  // Bullet never frees BVH shapes it did not build...
        delete s.iface;
        s.shape = nullptr;
        s.iface = nullptr;
    }
    h->shapes.clear();
    delete h;
}

}  // namespace maploader
