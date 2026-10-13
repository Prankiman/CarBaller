#include "ue3_mesh.h"

#include <cstring>

namespace ue3 {

namespace {

constexpr int32_t BULKDATA_Unused = 0x20;
constexpr int32_t BULKDATA_SeparateData = 0x40;

struct MeshErr {
    std::string msg;
};

float halfToFloat(uint16_t h) {
    int sign = (h >> 15) & 1;
    int exp = (h >> 10) & 0x1F;
    int man = h & 0x3FF;
    float v;
    if (exp == 0) {
        if (man == 0) {
            v = 0.f;
        } else {
            // subnormal: (-1)^s * man * 2^-24
            v = (float)man * 5.9604644775390625e-8f;
        }
    } else if (exp == 31) {
        v = 1e30f;  // inf/nan are meaningless for UVs
    } else {
        float m = 1.f + (float)man / 1024.f;
        int e = exp - 15;
        v = (e >= 0) ? m * (float)(1 << e) : m / (float)(1 << -e);
    }
    return sign ? -v : v;
}

// FByteBulkData::Skip (UE3, ArVer >= 266). Returns false on overflow.
bool skipBulk(Cursor& c) {
    if (!c.need(16)) return false;
    int32_t flags = c.i32();
    int32_t elemCount = c.i32();
    int32_t sizeOnDisk = c.i32();
    int32_t offset = c.i32();
    if (flags & BULKDATA_Unused) return true;
    if (flags & BULKDATA_SeparateData) return true;
    if (elemCount > 0 && offset == c.o) {
        if (!c.need(sizeOnDisk)) return false;
        c.skip(sizeOnDisk);
    }
    return true;
}

// BulkSerialize: [i32 ElementSize][i32 Count][data]. Returns byte pointer + count.
bool bulk(Cursor& c, int expectElemSize, int32_t* elemSize, int32_t* count,
          const uint8_t** data) {
    if (!c.need(8)) return false;
    int32_t es = c.i32();
    int32_t cnt = c.i32();
    if (expectElemSize >= 0 && es != expectElemSize) return false;
    if (cnt < 0 || es <= 0) return false;
    int64_t bytes = (int64_t)cnt * es;
    if (bytes > c.left()) return false;
    *elemSize = es;
    *count = cnt;
    *data = c.take((int32_t)bytes);
    return *data != nullptr;
}

bool parseSections(Cursor& c, std::vector<MeshSection>* out) {
    if (!c.need(4)) return false;
    int32_t n = c.i32();
    if (n < 0 || n > 4096) return false;
    out->clear();
    out->reserve((size_t)n);
    for (int32_t i = 0; i < n; i++) {
        if (!c.need(9 * 4 + 4 + 1)) return false;
        MeshSection s;
        s.materialRef = c.i32();
        c.i32();  // f10
        c.i32();  // f14
        c.i32();  // shadow (ArVer >= 473)
        s.firstIndex = c.i32();
        s.numFaces = c.i32();
        c.i32();  // f24
        c.i32();  // f28
        c.i32();  // index (ArVer >= 492)
        int32_t arr = c.i32();  // TArray<TArray<int,2>> (ArVer >= 514)
        if (arr < 0 || !c.need(arr * 8)) return false;
        c.skip(arr * 8);
        uint8_t ps3 = c.u8();
        if (ps3) {
            for (int k = 0; k < 8; k++) {
                if (!c.need(4)) return false;
                int32_t cn = c.i32();
                if (cn < 0) return false;
                int elem = (k < 2) ? 4 : 2;
                if (!c.need(cn * elem)) return false;
                c.skip(cn * elem);
            }
        }
        out->push_back(s);
    }
    return true;
}

struct Lod {
    std::vector<V3> pos;
    std::vector<float> uv;
    std::vector<uint32_t> idx;
    std::vector<MeshSection> sections;
};

bool parseLod(Cursor& c, int arver, Lod* lod) {
    if (!skipBulk(c)) return false;
    if (!parseSections(c, &lod->sections)) return false;

    // vertex stream
    if (!c.need(8)) return false;
    c.i32();  // vertex size header
    c.i32();  // vertex count header
    int32_t vElem = 0, vCount = 0;
    const uint8_t* vData = nullptr;
    if (!bulk(c, 12, &vElem, &vCount, &vData)) return false;
    lod->pos.resize((size_t)vCount);
    if (vCount) std::memcpy(lod->pos.data(), vData, (size_t)vCount * 12);

    // UV stream
    if (!c.need(16)) return false;
    int32_t ntc = c.i32();
    int32_t itemSizeHdr = c.i32();
    int32_t nv2 = c.i32();
    int32_t fullUV = (arver >= 474) ? c.i32() : 1;
    (void)itemSizeHdr;
    (void)nv2;
    if (ntc < 0 || ntc > 32) return false;
    int32_t uvElem = 0, uvCount = 0;
    const uint8_t* uvData = nullptr;
    if (!bulk(c, -1, &uvElem, &uvCount, &uvData)) return false;

    // color stream (ArVer >= 615)
    if (!c.need(8)) return false;
    c.i32();  // item size
    int32_t cNv = c.i32();
    if (cNv) {
        int32_t es = 0, cnt = 0;
        const uint8_t* d = nullptr;
        if (!bulk(c, -1, &es, &cnt, &d)) return false;
    }
    if (arver < 686) {
        if (!c.need(8)) return false;
        c.i32();
        c.i32();
        int32_t es = 0, cnt = 0;
        const uint8_t* d = nullptr;
        if (!bulk(c, -1, &es, &cnt, &d)) return false;
    }

    if (!c.need(4)) return false;
    c.i32();  // num verts header

    // index buffers
    if (!c.need(8)) return false;
    int32_t idElem = c.i32();
    int32_t idCount = c.i32();
    if (idCount < 0 || idElem <= 0 || idElem > 4) return false;
    int64_t idBytes = (int64_t)idCount * idElem;
    if (idBytes > c.left()) return false;
    const uint8_t* idData = c.take((int32_t)idBytes);
    if (!idData) return false;
    lod->idx.resize((size_t)idCount);
    if (idElem == 2) {
        for (int32_t i = 0; i < idCount; i++) {
            uint16_t v;
            std::memcpy(&v, idData + (size_t)i * 2, 2);
            lod->idx[i] = v;
        }
    } else if (idElem == 4) {
        for (int32_t i = 0; i < idCount; i++) {
            uint32_t v;
            std::memcpy(&v, idData + (size_t)i * 4, 4);
            lod->idx[i] = v;
        }
    }
    // wireframe index buffer
    {
        int32_t es2 = 0, cnt2 = 0;
        const uint8_t* d2 = nullptr;
        if (!bulk(c, -1, &es2, &cnt2, &d2)) return false;
    }
    if (arver < 686) {
        int32_t es3 = 0, cnt3 = 0;
        const uint8_t* d3 = nullptr;
        if (!bulk(c, -1, &es3, &cnt3, &d3)) return false;
        if (!c.need(4)) return false;
        c.i32();
    }
    if (arver >= 841) {
        int32_t es4 = 0, cnt4 = 0;
        const uint8_t* d4 = nullptr;
        if (!bulk(c, -1, &es4, &cnt4, &d4)) return false;
    }

    // unpack UV items: [FPackedNormal x2][ntc texcoord pairs]
    if (uvCount && uvData) {
        bool full = fullUV != 0;
        int uvLen = full ? 8 : 4;
        int expect = 8 + ntc * uvLen;
        if (uvElem != expect) return false;
        lod->uv.assign((size_t)uvCount * 2, 0.f);
        const uint8_t* p = uvData;
        for (int32_t i = 0; i < uvCount; i++) {
            p += 8;  // FPackedNormal[0], FPackedNormal[2]
            float u = 0, v = 0;
            if (full) {
                std::memcpy(&u, p, 4);
                std::memcpy(&v, p + 4, 4);
            } else {
                uint16_t uh, vh;
                std::memcpy(&uh, p, 2);
                std::memcpy(&vh, p + 2, 2);
                u = halfToFloat(uh);
                v = halfToFloat(vh);
            }
            lod->uv[(size_t)i * 2 + 0] = u;
            lod->uv[(size_t)i * 2 + 1] = v;
            p += ntc * uvLen;
        }
    }
    return true;
}

}  // namespace

bool parseStaticMesh(const Package& pk, int32_t exportIdx, StaticMeshParse* out,
                     std::string* err) {
    *out = StaticMeshParse();
    if (exportIdx < 0 || exportIdx >= pk.numExports()) {
        out->err = "bad export";
        if (err) *err = out->err;
        return false;
    }
    const ExportInfo& e = pk.ex(exportIdx);
    int32_t off = e.serialOffset;
    int32_t end = e.serialOffset + e.serialSize;
    if (e.serialSize <= 0) {
        out->err = "empty export";
        if (err) *err = out->err;
        return false;
    }

    // UStaticMesh is a plain UObject: 4-byte NetIndex + property chain.
    std::vector<Prop> chain;
    int32_t chainEnd = off + 4;
    bool chainOk = true;
    if (e.serialSize > 4 && !pk.parseObjectAt(exportIdx, 4, &chain, &chainEnd)) {
        chainEnd = off + 4;  // fall back like the reference parser
        chain.clear();
        chainOk = false;
    }
    out->headerLen = chainEnd - off;

    // Static collision designation (UE3 UStaticMesh flags; cooked packages
    // only serialize non-default values, so the class default applies):
    //  - UseSimpleRigidBodyCollision=false: rigid bodies use the render
    //    triangles as their collision mesh (every solid floor/ring).
    //  - =true (the class default, so absent on goal sheets / sky sphere /
    //    signs): only the BodySetup's simple shapes collide - and with no
    //    BodySetup object the mesh has no collision at all in the real game.
    // Such meshes still render; they just never block.
    if (chainOk) {
        bool useSimple = true;
        const Prop* rb = findProp(chain, "UseSimpleRigidBodyCollision");
        if (rb && rb->type == "BoolProperty") useSimple = rb->boolVal != 0;
        out->rigidBody = true;
        if (useSimple) {
            const Prop* bs = findProp(chain, "BodySetup");
            out->rigidBody =
                bs && bs->type == "ObjectProperty" && bs->data.size() >= 4 &&
                bs->i32(0) > 0;
        }
    }

    Cursor c(pk.bytes(), chainEnd, end);
    if (!c.need(28)) {
        out->err = "mesh payload too small";
        if (err) *err = out->err;
        return false;
    }
    out->origin = c.f3();
    out->boxExtent = c.f3();
    out->sphereRadius = c.f32();
    out->bodySetupRef = c.i32();
    c.f3();
    c.f3();  // kDOP bounds

    int32_t es = 0, cnt = 0;
    const uint8_t* d = nullptr;
    if (!bulk(c, 6, &es, &cnt, &d)) {
        out->err = "kDOP node bulk";
        if (err) *err = out->err;
        return false;
    }
    out->kdopNodeCount = cnt;
    if (!bulk(c, 8, &es, &cnt, &d)) {
        out->err = "kDOP triangle bulk";
        if (err) *err = out->err;
        return false;
    }
    out->kdopTriCount = cnt;

    if (!c.need(4)) {
        out->err = "internal version";
        if (err) *err = out->err;
        return false;
    }
    c.i32();  // internal version

    int arver = pk.version();
    Lod dummy;
    if (arver >= 823) {
        if (!c.need(4)) {
            out->err = "unk823";
            if (err) *err = out->err;
            return false;
        }
        int32_t unkFlag = c.i32();
        if (unkFlag) {
            if (!parseLod(c, arver, &dummy)) {
                out->err = "extra LOD parse";
                if (err) *err = out->err;
                return false;
            }
        }
        if (!c.need(4)) {
            out->err = "arr74";
            if (err) *err = out->err;
            return false;
        }
        int32_t ac = c.i32();
        int rec = (arver >= 829) ? 7 : 4;
        if (ac < 0 || !c.need(ac * rec)) {
            out->err = "arr74 payload";
            if (err) *err = out->err;
            return false;
        }
        c.skip(ac * rec);
        if (!c.need(4)) {
            out->err = "f74";
            if (err) *err = out->err;
            return false;
        }
        c.i32();
    }
    if (arver >= 859) {
        if (!c.need(4)) {
            out->err = "unk859";
            if (err) *err = out->err;
            return false;
        }
        c.i32();
    }

    if (!c.need(4)) {
        out->err = "lod count";
        if (err) *err = out->err;
        return false;
    }
    int32_t nlods = c.i32();
    if (nlods < 0 || nlods > 32) {
        out->err = "bad lod count";
        if (err) *err = out->err;
        return false;
    }
    out->numLods = nlods;
    Lod lod0;
    for (int32_t i = 0; i < nlods; i++) {
        Lod tmp;
        if (!parseLod(c, arver, &tmp)) {
            out->err = "LOD parse failed";
            if (err) *err = out->err;
            return false;
        }
        if (i == 0) lod0 = std::move(tmp);
    }
    out->bytesLeft = end - c.o;

    out->pos = std::move(lod0.pos);
    out->uv = std::move(lod0.uv);
    out->idx = std::move(lod0.idx);
    out->sections = std::move(lod0.sections);

    if (!out->pos.empty()) {
        V3 mn(1e30f, 1e30f, 1e30f), mx(-1e30f, -1e30f, -1e30f);
        for (const V3& p : out->pos) {
            if (p.x < mn.x) mn.x = p.x;
            if (p.y < mn.y) mn.y = p.y;
            if (p.z < mn.z) mn.z = p.z;
            if (p.x > mx.x) mx.x = p.x;
            if (p.y > mx.y) mx.y = p.y;
            if (p.z > mx.z) mx.z = p.z;
        }
        out->boundsMin = mn;
        out->boundsMax = mx;
        out->hasBounds = true;
    }
    out->ok = true;
    return true;
}

}  // namespace ue3
