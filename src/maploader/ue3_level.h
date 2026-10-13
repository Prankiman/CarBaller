#pragma once
// Level scan: actors (transforms + component links), components (mesh/channel
// data) and blocking-volume convex collision.

#include <cstdint>
#include <string>
#include <vector>

#include "map_data.h"
#include "ue3_package.h"

namespace ue3 {

// UE3 rotators are three int32 in 65536ths of a turn (Pitch, Yaw, Roll).
inline double unitsToRad(int32_t u) {
    return (double)u * 6.283185307179586 / 65536.0;
}

// local -> world: translate(loc) * rot(pitch, yaw, roll) * scale
M4 makeXform(const V3& loc, int32_t pitch, int32_t yaw, int32_t roll, const V3& scale);

struct ActorRec {
    bool valid = false;
    int32_t exportIdx = -1;
    std::string name;
    std::string className;
    std::string tag;         // AActor.Tag (name), e.g. "PlayerSpawn07"
    V3 loc;
    int32_t pitch = 0, yaw = 0, roll = 0;
    V3 drawScale3D{1, 1, 1};
    float drawScale = 1.f;
    bool hidden = false;     // HiddenGame: editor-only decoration
    bool disableAllRigidBody = false;  // renders, never blocks (Ice Rings rings)
};

// `actors` is indexed by export index (invalid entries are not actors).
// `ownerOf` maps a component export index to the owning actor export index.
bool scanActors(const Package& pk, std::vector<ActorRec>* actors,
                std::vector<int32_t>* ownerOf, std::string* err);

struct ComponentRec {
    bool valid = false;
    int32_t exportIdx = -1;
    std::string className;
    int32_t ownerIdx = -1;
    V3 relativeLoc;
    int32_t relPitch = 0, relYaw = 0, relRoll = 0;
    V3 scale3D{1, 1, 1};
    int32_t staticMeshRef = -1;  // export index of the StaticMesh, -1 when none
    bool blockRigidBody = true;  // UE3 default when the property is absent
    bool disableAllRigidBody = false;
    bool hidden = false;         // HiddenGame: not drawn, no collision
    CollChannel coll = CollChannel::All;
    bool hasAggGeom = false;     // BrushAggGeom / AggGeom present in the chain
};

// headerLen: 8 for UActorComponent subclasses, -1 to auto-detect.
bool readComponent(const Package& pk, int32_t exportIdx, int headerLen, ComponentRec* out,
                   std::string* err);

struct ConvexElem {
    std::vector<V3> verts;
    std::vector<int32_t> tris;
};

// Extracts agg-geom convex elements (KAggregateGeom.ConvexElems) from a
// component/BodySetup whose chain holds BrushAggGeom / AggGeom.
bool readConvexElems(const std::vector<Prop>& chain, std::vector<ConvexElem>* out,
                     std::string* err);
bool readComponentConvexElems(const Package& pk, int32_t exportIdx,
                              std::vector<ConvexElem>* out, std::string* err);

// Resolve an RBChannel ByteProperty payload (an FName in cooked packages).
CollChannel channelFromRBChannel(const Package& pk, const Prop* p);

}  // namespace ue3
