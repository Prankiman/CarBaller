#include "ue3_level.h"

#include <cmath>
#include <cstring>

namespace ue3 {

namespace {
bool endsWith(const std::string& s, const char* suffix) {
    size_t n = std::strlen(suffix);
    return s.size() >= n && s.compare(s.size() - n, n, suffix) == 0;
}

bool propVec3(const Prop* p, V3* out) {
    if (!p) return false;
    float f[3];
    if (p->vec3(f)) {
        out->x = f[0];
        out->y = f[1];
        out->z = f[2];
        return true;
    }
    return false;
}

bool propRot(const Prop* p, int32_t* pitch, int32_t* yaw, int32_t* roll) {
    if (!p || p->data.size() < 12) return false;
    int32_t r[3];
    std::memcpy(r, p->data.data(), 12);
    *pitch = r[0];
    *yaw = r[1];
    *roll = r[2];
    return true;
}
}  // namespace

M4 makeXform(const V3& loc, int32_t pitch, int32_t yaw, int32_t roll, const V3& scale) {
    double cy = std::cos(unitsToRad(pitch)), sy = std::sin(unitsToRad(pitch));
    double cz = std::cos(unitsToRad(yaw)), sz = std::sin(unitsToRad(yaw));
    // UE3 rotators are not a plain right-handed rotation: the engine's matrix
    // is Rz(yaw) * Ry(-pitch) * Rx(-roll) - pitch up is positive and roll
    // banks the opposite way from the naive composition (validated against the
    // reference loader: with +roll the StartingPlatform/checkmark/Goalposts
    // placements come out mirrored).
    double cx = std::cos(-unitsToRad(roll)), sx = std::sin(-unitsToRad(roll));

    // R = Rz(yaw) * Ry(-pitch) * Rx(-roll), row-major
    double R[3][3];
    R[0][0] = cz * cy;
    R[0][1] = -cz * sy * sx - sz * cx;
    R[0][2] = -cz * sy * cx + sz * sx;
    R[1][0] = sz * cy;
    R[1][1] = -sz * sy * sx + cz * cx;
    R[1][2] = -sz * sy * cx - cz * sx;
    R[2][0] = sy;
    R[2][1] = cy * sx;
    R[2][2] = cy * cx;

    M4 m;
    float* a = m.m;
    a[0] = (float)(R[0][0] * scale.x);
    a[1] = (float)(R[1][0] * scale.x);
    a[2] = (float)(R[2][0] * scale.x);
    a[3] = 0.f;
    a[4] = (float)(R[0][1] * scale.y);
    a[5] = (float)(R[1][1] * scale.y);
    a[6] = (float)(R[2][1] * scale.y);
    a[7] = 0.f;
    a[8] = (float)(R[0][2] * scale.z);
    a[9] = (float)(R[1][2] * scale.z);
    a[10] = (float)(R[2][2] * scale.z);
    a[11] = 0.f;
    a[12] = loc.x;
    a[13] = loc.y;
    a[14] = loc.z;
    a[15] = 1.f;
    return m;
}

bool scanActors(const Package& pk, std::vector<ActorRec>* actors,
                std::vector<int32_t>* ownerOf, std::string* err) {
    int32_t n = pk.numExports();
    actors->assign((size_t)n, ActorRec{});
    ownerOf->assign((size_t)n, -1);
    for (int32_t i = 0; i < n; i++) {
        std::string cls = pk.exportClassName(i);
        if (cls.empty() || endsWith(cls, "Component")) continue;
        int headerLen = 0;
        int32_t chainEnd = 0;
        std::vector<Prop> chain;
        if (!pk.parseObject(i, &headerLen, &chain, &chainEnd, nullptr)) continue;
        const Prop* loc = findProp(chain, "Location");
        if (!loc || loc->type != "StructProperty") continue;

        ActorRec a;
        a.valid = true;
        a.exportIdx = i;
        a.name = pk.exportName(i);
        a.className = cls;
        const Prop* tag = findProp(chain, "Tag");
        if (tag && tag->data.size() >= 4) a.tag = pk.nameAt(tag->i32(0));
        if (!propVec3(loc, &a.loc)) a.loc = V3{0, 0, 0};
        propRot(findProp(chain, "Rotation"), &a.pitch, &a.yaw, &a.roll);
        const Prop* d3 = findProp(chain, "DrawScale3D");
        if (d3) propVec3(d3, &a.drawScale3D);
        const Prop* ds = findProp(chain, "DrawScale");
        if (ds && ds->data.size() >= 4) a.drawScale = ds->f32();
        const Prop* ahid = findProp(chain, "HiddenGame");
        if (ahid && ahid->type == "BoolProperty") a.hidden = ahid->boolVal != 0;
        const Prop* adis = findProp(chain, "bDisableAllRigidBody");
        if (adis && adis->type == "BoolProperty")
            a.disableAllRigidBody = adis->boolVal != 0;
        (*actors)[(size_t)i] = a;

        for (const Prop& p : chain) {
            if (p.type != "ObjectProperty" && p.type != "ComponentProperty") continue;
            if (p.data.size() < 4) continue;
            int32_t ref = p.i32(0);
            int32_t target = refToIndex(ref);
            if (ref > 0 && target >= 0 && target < n && (*ownerOf)[(size_t)target] < 0)
                (*ownerOf)[(size_t)target] = i;
        }
    }
    return true;
}

bool readComponent(const Package& pk, int32_t exportIdx, int headerLen, ComponentRec* out,
                   std::string* err) {
    *out = ComponentRec();
    if (exportIdx < 0 || exportIdx >= pk.numExports()) {
        if (err) *err = "component export out of range";
        return false;
    }
    std::vector<Prop> chain;
    int32_t chainEnd = 0;
    bool ok;
    if (headerLen > 0)
        ok = pk.parseObjectAt(exportIdx, headerLen, &chain, &chainEnd, err);
    else {
        int h = 0;
        ok = pk.parseObject(exportIdx, &h, &chain, &chainEnd, err);
    }
    if (!ok) return false;

    out->valid = true;
    out->exportIdx = exportIdx;
    out->className = pk.exportClassName(exportIdx);

    const Prop* sm = findProp(chain, "StaticMesh");
    if (sm && sm->data.size() >= 4) {
        int32_t ref = sm->i32(0);
        if (ref > 0) out->staticMeshRef = ref - 1;
    }
    out->coll = channelFromRBChannel(pk, findProp(chain, "RBChannel"));
    const Prop* brb = findProp(chain, "BlockRigidBody");
    if (brb && brb->type == "BoolProperty") out->blockRigidBody = brb->boolVal != 0;
    const Prop* dis = findProp(chain, "bDisableAllRigidBody");
    if (dis && dis->type == "BoolProperty") out->disableAllRigidBody = dis->boolVal != 0;
    if (out->disableAllRigidBody) out->blockRigidBody = false;

    propVec3(findProp(chain, "RelativeLocation"), &out->relativeLoc);
    propRot(findProp(chain, "RelativeRotation"), &out->relPitch, &out->relYaw, &out->relRoll);
    const Prop* sc = findProp(chain, "RelativeScale3D");
    if (!sc) sc = findProp(chain, "Scale3D");
    if (sc) {
        propVec3(sc, &out->scale3D);
    } else {
        // Cooked UE3 sometimes serializes a uniform float Scale instead of a
        // vector (Leth's 1054 ice floor tiles carry Scale=2048); without it
        // those placements come out at 1:1 and effectively vanish.
        const Prop* su = findProp(chain, "Scale");
        if (su && su->data.size() >= 4 &&
            (su->type == "FloatProperty" || su->type == "IntProperty")) {
            float u = 1.f;
            if (su->type == "FloatProperty") {
                std::memcpy(&u, su->data.data(), 4);
            } else {
                int32_t iv = 1;
                std::memcpy(&iv, su->data.data(), 4);
                u = (float)iv;
            }
            out->scale3D = V3{u, u, u};
        }
    }
    const Prop* hid = findProp(chain, "HiddenGame");
    if (hid && hid->type == "BoolProperty") out->hidden = hid->boolVal != 0;

    out->hasAggGeom = findProp(chain, "BrushAggGeom") != nullptr ||
                      findProp(chain, "AggGeom") != nullptr;
    return true;
}

bool readConvexElems(const std::vector<Prop>& chain, std::vector<ConvexElem>* out,
                     std::string* err) {
    out->clear();
    const Prop* agg = findProp(chain, "BrushAggGeom");
    if (!agg) agg = findProp(chain, "AggGeom");
    if (!agg) return true;
    const Prop* elems = findProp(agg->nested, "ConvexElems");
    if (!elems || !elems->isArray) return true;
    for (const std::vector<Prop>& elem : elems->structArray) {
        const Prop* vd = findProp(elem, "VertexData");
        const Prop* ft = findProp(elem, "FaceTriData");
        if (!vd || !vd->isArray || vd->fixedElemSize != 12) continue;
        ConvexElem ce;
        ce.verts.reserve(vd->fixedArray.size());
        for (const std::vector<uint8_t>& raw : vd->fixedArray) {
            if (raw.size() != 12) continue;
            V3 v;
            std::memcpy(&v, raw.data(), 12);
            ce.verts.push_back(v);
        }
        if (ce.verts.empty()) continue;
        if (ft && ft->isArray && !ft->intArray.empty()) {
            ce.tris = ft->intArray;
            bool bad = false;
            for (int32_t t : ce.tris)
                if (t < 0 || (size_t)t >= ce.verts.size()) {
                    bad = true;
                    break;
                }
            if (bad || ce.tris.size() % 3 != 0) ce.tris.clear();
        }
        if (ce.tris.empty()) continue;
        out->push_back(std::move(ce));
    }
    return true;
}

bool readComponentConvexElems(const Package& pk, int32_t exportIdx,
                              std::vector<ConvexElem>* out, std::string* err) {
    out->clear();
    int headerLen = 8;
    std::vector<Prop> chain;
    int32_t chainEnd = 0;
    if (!pk.parseObjectAt(exportIdx, headerLen, &chain, &chainEnd, nullptr)) {
        int h = 0;
        if (!pk.parseObject(exportIdx, &h, &chain, &chainEnd, err)) return false;
    }
    return readConvexElems(chain, out, err);
}

CollChannel channelFromRBChannel(const Package& pk, const Prop* p) {
    if (!p || p->type != "ByteProperty") return CollChannel::All;
    if (p->data.size() >= 8) {
        int32_t nameIdx = 0;
        std::memcpy(&nameIdx, p->data.data(), 4);
        const std::string& s = pk.nameAt(nameIdx);
        if (s == "RBCC_VehicleBlocker") return CollChannel::VehicleOnly;
        if (s == "RBCC_BallBlocker") return CollChannel::BallOnly;
        if (s == "RBCC_None") return CollChannel::None;
        return CollChannel::All;  // Default / BlockingVolume / Balls / Vehicle
    }
    if (p->data.size() == 1) {
        uint8_t v = p->data[0];
        if (v == 0) return CollChannel::None;
    }
    return CollChannel::All;
}

}  // namespace ue3
