#include "ue3_bsp.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>

namespace ue3 {
namespace {

// FBspNode, 64 bytes (UE3 UnModel.h):
//   FPlane Plane(0) | INT iPlane(16) | INT iSurf(20) | INT iVertPool(24) |
//   WORD ComponentIndex(28) | WORD ComponentNodeIndex(30) |
//   INT ComponentElementIndex(32) | INT iChildren[2](36) | INT iBounds(44) |
//   INT iCollisionBound(48) | BYTE iZone[2](52) | BYTE NumVertices(54) |
//   BYTE NodeFlags(55) | INT iLeaf[2](56)
//
// The vertex pool index is the INT at +24, not +16: it is exactly contiguous
// with NumVertices across every node and stays inside the vertex buffer, while
// the value at +16 overruns it (Dribble: 2246 vs. 1102 vertices).
constexpr int32_t kNodeStride = 64;
constexpr int32_t kVertStride = 36;  // FModelVertex: Position + 2 packed normals + 2 UVs

struct BspNode {
    float plane[4];
    int32_t ivp = 0;
    int32_t isurf = 0;
    int32_t nv = 0;
};

int32_t rdI32(const uint8_t* p) {
    int32_t v;
    std::memcpy(&v, p, 4);
    return v;
}
float rdF32(const uint8_t* p) {
    float v;
    std::memcpy(&v, p, 4);
    return v;
}
bool unitNormal(float x, float y, float z) {
    double m = std::sqrt((double)x * x + (double)y * y + (double)z * z);
    return std::fabs(m - 1.0) < 1e-3;
}

// Signed distance of a vertex to a node's own plane. BSP faces are built from
// the pool their node points at, so every vertex of a node must lie on it -
// that invariant is what identifies the two arrays we have to locate by scan.
float planeDist(const BspNode& n, const uint8_t* v) {
    return n.plane[0] * rdF32(v) + n.plane[1] * rdF32(v + 4) +
           n.plane[2] * rdF32(v + 8) - n.plane[3];
}

// Locates "INT count; FBspNode[count]" inside the model's native payload.
bool findNodes(const uint8_t* pay, int32_t plen, const uint8_t** out, int32_t* count) {
    int32_t bestCount = 0;
    const uint8_t* best = nullptr;
    for (int32_t o = 0; o + kNodeStride <= plen - 4; o += 4) {
        int32_t n = rdI32(pay + o);
        if (n < 1 || n > (1 << 20)) continue;
        if ((int64_t)o + 4 + (int64_t)n * kNodeStride > plen) continue;
        const uint8_t* base = pay + o + 4;
        int probe = n < 8 ? n : 8;
        bool ok = true;
        for (int k = 0; k < probe; k++) {
            const uint8_t* r = base + (int64_t)k * kNodeStride;
            if (!unitNormal(rdF32(r), rdF32(r + 4), rdF32(r + 8)) ||
                rdI32(r + 20) < 0 || rdI32(r + 24) < 0 || r[54] > 64) {
                ok = false;
                break;
            }
        }
        if (ok && n > bestCount) {
            bestCount = n;
            best = base;
        }
    }
    if (!best) return false;
    *out = best;
    *count = bestCount;
    return true;
}

// Locates "INT count; FModelVertex[count]" by scoring how many node vertices
// land on their own plane. Returns the payload offset of the records.
bool findVerts(const uint8_t* pay, int32_t plen, const std::vector<BspNode>& nodes,
               int32_t* outBase, int32_t* outCount) {
    int32_t need = 0;
    for (const BspNode& n : nodes) {
        if (n.ivp < 0 || n.nv <= 0) continue;
        need = std::max(need, n.ivp + n.nv);
    }
    if (need <= 0 || (int64_t)need * kVertStride > plen) return false;

    int probe = -1;
    for (size_t i = 0; i < nodes.size(); i++)
        if (nodes[i].nv > 0) {
            probe = (int)i;
            break;
        }
    if (probe < 0) return false;

    int32_t bestBase = -1, bestCount = 0, bestScore = -1;
    for (int32_t o = 0; o + 4 <= plen; o += 4) {
        int32_t m = rdI32(pay + o);
        if (m < need) continue;
        if ((int64_t)o + 4 + (int64_t)m * kVertStride > plen) continue;
        const uint8_t* v = pay + o + 4;
        // "<= 0.5" rather than "> 0.5 continue": a candidate whose probe plane
        // is garbage yields NaN, and NaN would otherwise pass the test and pay
        // for a full score over every node vertex.
        if (!(std::fabs(planeDist(nodes[(size_t)probe],
                                  v + (int64_t)nodes[(size_t)probe].ivp * kVertStride)) <=
              0.5f))
            continue;
        int good = 0, tot = 0;
        for (const BspNode& n : nodes) {
            if (n.nv <= 0) continue;
            for (int32_t k = 0; k < n.nv; k++) {
                tot++;
                if (std::fabs(planeDist(n, v + (int64_t)(n.ivp + k) * kVertStride)) < 0.5f)
                    good++;
            }
        }
        if (!tot) continue;
        int score = good * 100 / tot;
        if (score > bestScore) {
            bestScore = score;
            bestBase = o + 4;
            bestCount = m;
        }
    }
    if (bestBase < 0 || bestScore < 95) return false;
    *outBase = bestBase;
    *outCount = bestCount;
    return true;
}

// "INT count; FBspSurf[count]" - found by requiring every record to carry the
// unit surface plane at a fixed offset. Optional: without it the BSP just gets
// one colour instead of one per material.
bool findSurfs(const uint8_t* pay, int32_t plen, int32_t nSurf, int32_t* outStride,
               int32_t* outPlaneOff, int32_t* outBase) {
    if (nSurf <= 0) return false;
    for (int32_t o = 0; o + 4 <= plen; o += 4) {
        if (rdI32(pay + o) != nSurf) continue;
        for (int32_t st = 48; st <= 84; st += 4) {
            if ((int64_t)o + 4 + (int64_t)nSurf * st > plen) continue;
            for (int32_t po = 16; po <= 48; po += 4) {
                if (po + 16 > st) continue;
                bool ok = true;
                for (int32_t k = 0; k < nSurf; k++) {
                    const uint8_t* r = pay + o + 4 + (int64_t)k * st + po;
                    if (!unitNormal(rdF32(r), rdF32(r + 4), rdF32(r + 8))) {
                        ok = false;
                        break;
                    }
                }
                if (!ok) continue;
                *outStride = st;
                *outPlaneOff = po;
                *outBase = o + 4;
                return true;
            }
        }
    }
    return false;
}

// UModelComponent payload: INT Model, INT Zone, INT NumElements, INT zero,
// then per element (INT ComponentIndex, INT Material, INT NumNodes,
// WORD Nodes[], INT NumShapeElems, ... 4 * NumShapeElems, INT MinInstTri,
// WORD MySelf[, WORD pad]) and finally INT NumTrailing, WORD Trailing[].
// Everything is validated against the export bounds and the node count, so a
// package that lays this out differently simply yields no BSP.
bool readComponentNodes(const Package& pk, int32_t idx, int32_t chainEnd, int32_t nnodes,
                        std::vector<uint8_t>* used) {
    const ExportInfo& e = pk.ex(idx);
    const uint8_t* d = pk.bytes() + chainEnd;
    const int64_t len = (int64_t)e.serialOffset + e.serialSize - chainEnd;
    int64_t o = 0;
    auto i32 = [&]() -> int32_t {
        if (o + 4 > len) return -1;
        int32_t v = rdI32(d + o);
        o += 4;
        return v;
    };
    auto u16 = [&]() -> int32_t {
        if (o + 2 > len) return -1;
        int32_t v;
        uint16_t u;
        std::memcpy(&u, d + o, 2);
        o += 2;
        v = u;
        return v;
    };
    i32();                       // Model (checked by the caller)
    i32();                       // Zone
    int32_t nelem = i32();
    i32();                       // reserved
    if (nelem < 0 || nelem > 4096) return false;
    auto takeNodes = [&](int32_t count) -> bool {
        if (count < 0 || count > 1 << 20) return false;
        for (int32_t k = 0; k < count; k++) {
            int32_t n = u16();
            if (n < 0) return false;
            if (n >= nnodes) return false;
            (*used)[(size_t)n] = 1;
        }
        return true;
    };
    for (int32_t k = 0; k < nelem; k++) {
        i32();                    // ComponentIndex
        i32();                    // Material
        if (!takeNodes(i32())) return false;
        int32_t nsh = i32();
        if (nsh < 0 || (o += (int64_t)nsh * 4) > len) return false;
        i32();                    // MinInstTri
        if (u16() < 0) return false;  // MySelf
        if (k + 1 < nelem && u16() < 0) return false;  // element separator
    }
    // The trailing WORDs index nodes in some cooks and are opaque data in
    // others: take them when they all resolve, ignore them otherwise. They
    // must never fail a component that decoded cleanly up to here.
    int32_t nTrail = i32();
    if (nTrail >= 0 && nTrail <= (1 << 20)) {
        std::vector<int32_t> tmp;
        bool ok = true;
        for (int32_t k = 0; k < nTrail && ok; k++) {
            int32_t n = u16();
            if (n < 0 || n >= nnodes)
                ok = false;
            else
                tmp.push_back(n);
        }
        if (ok)
            for (int32_t n : tmp) (*used)[(size_t)n] = 1;
    }
    return o == len;
}

}  // namespace

bool readLevelBsp(const Package& pk, std::vector<BspPiece>* out, std::string* err) {
    out->clear();
    auto fail = [&](const char* why) {
        if (err) *err = why;
        return false;
    };

    std::vector<int32_t> mcs;
    for (int32_t i = 0; i < pk.numExports(); i++)
        if (pk.exportClassName(i) == "ModelComponent") mcs.push_back(i);
    if (mcs.empty()) return fail("package has no BSP render components");

    // Every ModelComponent names the level UModel; header length is confirmed
    // by requiring that reference to resolve to a Model export.
    int32_t mcIdx = -1, modelIdx = -1;
    std::vector<Prop> chain;
    for (int32_t h : {8, 4, 12, 26}) {
        for (int32_t idx : mcs) {
            int32_t ce = 0;
            if (!pk.parseObjectAt(idx, h, &chain, &ce, nullptr)) continue;
            const ExportInfo& e = pk.ex(idx);
            if ((int64_t)e.serialOffset + e.serialSize - ce < 16) continue;
            int32_t ref = rdI32(pk.bytes() + ce);
            if (ref <= 0 || ref - 1 >= pk.numExports()) continue;
            if (pk.exportClassName(ref - 1) != "Model") continue;
            mcIdx = idx;
            modelIdx = ref - 1;
            break;
        }
        if (mcIdx >= 0) break;
    }
    if (mcIdx < 0) return fail("ModelComponent does not reference the level Model");

    const ExportInfo& me = pk.ex(modelIdx);
    int32_t ms = -1;
    for (int32_t h : {4, 8, 12, 26}) {
        int32_t ce = 0;
        if (pk.parseObjectAt(modelIdx, h, &chain, &ce, nullptr)) {
            ms = ce;
            break;
        }
    }
    const int32_t meEnd = me.serialOffset + me.serialSize;
    if (ms < 0 || meEnd - ms < kNodeStride + 4) return fail("level Model has no native payload");

    const uint8_t* pay = pk.bytes() + ms;
    const int32_t plen = meEnd - ms;

    const uint8_t* nodeBase = nullptr;
    int32_t nnodes = 0;
    if (!findNodes(pay, plen, &nodeBase, &nnodes)) return fail("level Model has no BSP nodes");

    std::vector<BspNode> nodes((size_t)nnodes);
    for (int32_t i = 0; i < nnodes; i++) {
        const uint8_t* r = nodeBase + (int64_t)i * kNodeStride;
        BspNode& n = nodes[(size_t)i];
        std::memcpy(n.plane, r, 16);
        n.isurf = rdI32(r + 20);
        n.ivp = rdI32(r + 24);
        n.nv = r[54];
    }

    int32_t vBase = -1, nverts = 0;
    if (!findVerts(pay, plen, nodes, &vBase, &nverts))
        return fail("level Model vertex buffer not found");

    // Which nodes the ModelComponents actually draw (a component that fails to
    // decode must not leave half-marked indices behind).
    std::vector<uint8_t> used((size_t)nnodes, 0);
    int anyComponent = 0;
    for (int32_t idx : mcs) {
        int32_t ce = 0;
        if (!pk.parseObjectAt(idx, 8, &chain, &ce, nullptr) &&
            !pk.parseObjectAt(idx, 4, &chain, &ce, nullptr))
            continue;
        std::vector<uint8_t> part((size_t)nnodes, 0);
        if (!readComponentNodes(pk, idx, ce, nnodes, &part)) continue;
        for (size_t k = 0; k < part.size(); k++) used[k] |= part[k];
        anyComponent++;
    }
    if (!anyComponent) return fail("ModelComponent node lists could not be decoded");

    int32_t nSurf = 0;
    for (const BspNode& n : nodes) nSurf = std::max(nSurf, n.isurf + 1);
    int32_t sStride = 0, sPlaneOff = 0, sBase = -1;
    if (!findSurfs(pay, plen, nSurf, &sStride, &sPlaneOff, &sBase)) nSurf = 0;

    const int32_t defaultMat = modelIdx;
    std::map<int32_t, size_t> group;
    for (int32_t i = 0; i < nnodes; i++) {
        if (!used[(size_t)i]) continue;
        const BspNode& n = nodes[(size_t)i];
        if (n.nv < 3 || n.ivp < 0 || (int64_t)n.ivp + n.nv > nverts) continue;
        int32_t mat = defaultMat;
        if (nSurf > 0 && n.isurf >= 0 && n.isurf < nSurf)
            mat = rdI32(pay + sBase + (int64_t)n.isurf * sStride);
        auto it = group.find(mat);
        size_t gi;
        if (it == group.end()) {
            out->push_back(BspPiece());
            out->back().materialRef = mat;
            gi = out->size() - 1;
            group[mat] = gi;
        } else {
            gi = it->second;
        }
        BspPiece& piece = (*out)[gi];
        uint32_t first = (uint32_t)piece.verts.size();
        for (int32_t k = 0; k < n.nv; k++) {
            const uint8_t* v = pay + vBase + (int64_t)(n.ivp + k) * kVertStride;
            piece.verts.push_back(V3(rdF32(v), rdF32(v + 4), rdF32(v + 8)));
        }
        for (int32_t k = 1; k + 1 < n.nv; k++) {
            piece.tris.push_back(first);
            piece.tris.push_back(first + (uint32_t)k);
            piece.tris.push_back(first + (uint32_t)k + 1);
        }
    }
    out->erase(std::remove_if(out->begin(), out->end(),
                              [](const BspPiece& p) { return p.tris.empty(); }),
               out->end());
    if (out->empty()) return fail("level BSP carries no drawable nodes");
    return true;
}

}  // namespace ue3
