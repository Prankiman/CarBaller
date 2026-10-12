#include "map_loader.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <unordered_map>

#include "map_sessions.h"
#include "ue3_bsp.h"
#include "ue3_level.h"
#include "ue3_mesh.h"
#include "ue3_package.h"

namespace maploader {

namespace {

using ue3::ComponentRec;
using ue3::ConvexElem;
using ue3::Package;
using ue3::Prop;
using ue3::StaticMeshParse;

std::string baseName(const std::string& p) {
    size_t s = p.find_last_of("/\\");
    return (s == std::string::npos) ? p : p.substr(s + 1);
}

bool startsWith(const std::string& s, const char* p) {
    size_t n = std::strlen(p);
    return s.size() >= n && s.compare(0, n, p) == 0;
}

std::string toLower(const std::string& s) {
    std::string r = s;
    for (char& c : r)
        if (c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');
    return r;
}

int trailingNumber(const std::string& s) {
    int i = (int)s.size() - 1;
    int v = -1, mul = 1;
    while (i >= 0 && s[(size_t)i] >= '0' && s[(size_t)i] <= '9') {
        if (v < 0) v = 0;
        v += (s[(size_t)i] - '0') * mul;
        mul *= 10;
        i--;
    }
    return v;
}

// Muted, well-separated colour per material slot (textures are out of scope).
void colorForRef(int32_t ref, float* r, float* g, float* b) {
    uint32_t h = (uint32_t)ref * 2654435761u;
    h ^= h >> 15;
    h *= 2246822519u;
    h ^= h >> 13;
    float hue = (h % 3600) / 3600.f;
    float sat = 0.28f + ((h >> 8) & 0x1F) / 31.f * 0.22f;
    float lig = 0.52f + ((h >> 13) & 0x1F) / 31.f * 0.16f;
    float q = lig < 0.5f ? lig * (1 + sat) : lig + sat - lig * sat;
    float p = 2 * lig - q;
    auto hue2rgb = [](float pp, float qq, float t) {
        if (t < 0) t += 1;
        if (t > 1) t -= 1;
        if (t < 1.f / 6.f) return qq + (pp - qq) * 6 * t;
        if (t < 1.f / 2.f) return pp;
        if (t < 2.f / 3.f) return qq + (pp - qq) * (2.f / 3.f - t) * 6;
        return qq;
    };
    *r = hue2rgb(p, q, hue + 1.f / 3.f);
    *g = hue2rgb(p, q, hue);
    *b = hue2rgb(p, q, hue - 1.f / 3.f);
}

// Builds a render shape: indexed vertex soup + angle-limited smooth normals
// (same ~50 degree rule the arena meshes use) + one colour per material slot.
void buildShape(const std::vector<V3>& sv, const std::vector<uint32_t>& tris,
                const std::vector<float>& uv, const std::vector<ue3::MeshSection>& secs,
                const std::string& name, int32_t defaultColorRef, MapShape* out) {
    out->name = name;
    const size_t nv = sv.size();
    out->verts.resize(nv);
    out->boundsMin = V3(1e30f, 1e30f, 1e30f);
    out->boundsMax = V3(-1e30f, -1e30f, -1e30f);

    float dr = 0.5f, dg = 0.5f, db = 0.5f;
    colorForRef(defaultColorRef, &dr, &dg, &db);
    for (size_t i = 0; i < nv; i++) {
        MapVertex& v = out->verts[i];
        v.px = sv[i].x;
        v.py = sv[i].y;
        v.pz = sv[i].z;
        v.nx = 0;
        v.ny = 0;
        v.nz = 1;
        v.u = (i * 2 + 1 < uv.size()) ? uv[i * 2] : 0.f;
        v.v = (i * 2 + 1 < uv.size()) ? uv[i * 2 + 1] : 0.f;
        v.r = dr;
        v.g = dg;
        v.b = db;
        v.a = 1.f;
        if (v.px < out->boundsMin.x) out->boundsMin.x = v.px;
        if (v.py < out->boundsMin.y) out->boundsMin.y = v.py;
        if (v.pz < out->boundsMin.z) out->boundsMin.z = v.pz;
        if (v.px > out->boundsMax.x) out->boundsMax.x = v.px;
        if (v.py > out->boundsMax.y) out->boundsMax.y = v.py;
        if (v.pz > out->boundsMax.z) out->boundsMax.z = v.pz;
    }
    if (nv == 0) {
        out->boundsMin = V3(0, 0, 0);
        out->boundsMax = V3(0, 0, 0);
    }

    // per-section colours
    for (const ue3::MeshSection& s : secs) {
        float r, g, b;
        colorForRef(s.materialRef, &r, &g, &b);
        size_t first = (size_t)std::max(0, s.firstIndex);
        size_t last = first + (size_t)std::max(0, s.numFaces) * 3;
        if (last > tris.size()) last = tris.size();
        for (size_t t = first; t + 2 < last && t + 2 < tris.size(); t += 3) {
            for (int k = 0; k < 3; k++) {
                uint32_t vi = tris[t + (size_t)k];
                if (vi >= nv) continue;
                out->verts[vi].r = r;
                out->verts[vi].g = g;
                out->verts[vi].b = b;
            }
        }
    }

    out->idx.clear();
    out->idx.reserve(tris.size());
    std::vector<V3> acc(nv, V3(0, 0, 0));
    std::vector<std::vector<V3>> adj(nv);

    for (size_t t = 0; t + 2 < tris.size(); t += 3) {
        uint32_t i0 = tris[t], i1 = tris[t + 1], i2 = tris[t + 2];
        if (i0 >= nv || i1 >= nv || i2 >= nv) continue;
        const MapVertex& a = out->verts[i0];
        const MapVertex& b = out->verts[i1];
        const MapVertex& c = out->verts[i2];
        V3 e1(b.px - a.px, b.py - a.py, b.pz - a.pz);
        V3 e2(c.px - a.px, c.py - a.py, c.pz - a.pz);
        V3 fn = e1.cross(e2);
        if (fn.len() < 1e-9f) continue;
        acc[i0] += fn;
        acc[i1] += fn;
        acc[i2] += fn;
        adj[i0].push_back(fn.norm());
        adj[i1].push_back(fn.norm());
        adj[i2].push_back(fn.norm());
        out->idx.push_back(i0);
        out->idx.push_back(i1);
        out->idx.push_back(i2);
    }

    for (size_t i = 0; i < nv; i++) {
        if (adj[i].empty()) continue;
        V3 ref = acc[i].norm();
        if (ref.len() < 0.5f) ref = adj[i][0];
        V3 sum(0, 0, 0);
        for (const V3& n : adj[i])
            if (n.dot(ref) > 0.64f) sum += n;
        V3 outn = sum.norm();
        if (outn.len() < 0.5f) outn = ref;
        out->verts[i].nx = outn.x;
        out->verts[i].ny = outn.y;
        out->verts[i].nz = outn.z;
    }
}

bool isBlockingVolumeClass(const std::string& cls) {
    return cls.find("BlockingVolume") != std::string::npos;
}

int32_t findOwner(const Package& pk, int32_t compIdx,
                  const std::vector<int32_t>& ownerOf,
                  const std::vector<ue3::ActorRec>& actors) {
    if (compIdx >= 0 && (size_t)compIdx < ownerOf.size() && ownerOf[(size_t)compIdx] >= 0)
        return ownerOf[(size_t)compIdx];
    int32_t outer = pk.ex(compIdx).outerRef;
    for (int guard = 0; guard < 8 && outer > 0; guard++) {
        int32_t oi = outer - 1;
        if (oi < 0 || oi >= (int32_t)actors.size()) break;
        if (actors[(size_t)oi].valid) return oi;
        outer = pk.ex(oi).outerRef;
    }
    return -1;
}

M4 actorXform(const ue3::ActorRec& a) {
    V3 sc(a.drawScale3D.x * a.drawScale, a.drawScale3D.y * a.drawScale,
          a.drawScale3D.z * a.drawScale);
    return ue3::makeXform(a.loc, a.pitch, a.yaw, a.roll, sc);
}

std::string fmtStr(const char* fmt, ...) {
    char b[512];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    return std::string(b);
}

void expandBounds(MapData* out, const MapPlacement& pl) {
    const MapShape& s = out->shapes[(size_t)pl.shape];
    for (int corner = 0; corner < 8; corner++) {
        V3 p((corner & 1) ? s.boundsMax.x : s.boundsMin.x,
             (corner & 2) ? s.boundsMax.y : s.boundsMin.y,
             (corner & 4) ? s.boundsMax.z : s.boundsMin.z);
        V3 w = pl.xform.mulPoint(p);
        if (!out->hasBounds) {
            out->boundsMin = out->boundsMax = w;
            out->hasBounds = true;
        } else {
            if (w.x < out->boundsMin.x) out->boundsMin.x = w.x;
            if (w.y < out->boundsMin.y) out->boundsMin.y = w.y;
            if (w.z < out->boundsMin.z) out->boundsMin.z = w.z;
            if (w.x > out->boundsMax.x) out->boundsMax.x = w.x;
            if (w.y > out->boundsMax.y) out->boundsMax.y = w.y;
            if (w.z > out->boundsMax.z) out->boundsMax.z = w.z;
        }
    }
}

}  // namespace

bool loadMap(const std::string& path, MapData* out, std::string* err) {
    std::string localErr;
    if (!err) err = &localErr;
    err->clear();
    *out = MapData();
    out->path = path;
    out->fileName = baseName(path);

    Package pk;
    if (!pk.load(path, err)) {
        if (err->empty()) *err = "failed to read package";
        return false;
    }

    std::vector<ue3::ActorRec> actors;
    std::vector<int32_t> ownerOf;
    if (!ue3::scanActors(pk, &actors, &ownerOf, err)) {
        if (err->empty()) *err = "failed to scan level actors";
        return false;
    }

    std::unordered_map<int32_t, int32_t> meshShape;  // static mesh export -> shape index
    meshShape.reserve(64);

    const int32_t n = pk.numExports();
    int verbose = std::getenv("MAPLOADER_VERBOSE") ? 1 : 0;
    int componentsSeen = 0;
    int aggGeomSeen = 0;
    std::string firstFail;
    for (int32_t i = 0; i < n; i++) {
        const std::string cls = pk.exportClassName(i);
        if (cls.empty()) continue;
        if (cls.size() < 9 || cls.compare(cls.size() - 9, 9, "Component") != 0) continue;
        componentsSeen++;

        ComponentRec comp;
        std::string cerr;
        if (!ue3::readComponent(pk, i, 8, &comp, &cerr) &&
            !ue3::readComponent(pk, i, -1, &comp, &cerr)) {
            out->stats.objectsFailed++;
            if (firstFail.empty()) firstFail = cls + ": " + cerr;
            continue;
        }
        out->stats.objectsParsed++;
        comp.ownerIdx = findOwner(pk, i, ownerOf, actors);

        M4 xform = M4::identity();
        if (comp.ownerIdx >= 0 && comp.ownerIdx < (int32_t)actors.size() &&
            actors[(size_t)comp.ownerIdx].valid)
            xform = actorXform(actors[(size_t)comp.ownerIdx]);
        xform = xform * ue3::makeXform(comp.relativeLoc, comp.relPitch, comp.relYaw,
                                       comp.relRoll, comp.scale3D);

        if (comp.staticMeshRef >= 0) {
            int32_t shape = -1;
            auto it = meshShape.find(comp.staticMeshRef);
            if (it != meshShape.end()) {
                shape = it->second;
            } else {
                StaticMeshParse mp;
                std::string merr;
                if (ue3::parseStaticMesh(pk, comp.staticMeshRef, &mp, &merr)) {
                    MapShape s;
                    buildShape(mp.pos, mp.idx, mp.uv, mp.sections,
                               pk.exportName(comp.staticMeshRef), comp.staticMeshRef, &s);
                    shape = (int32_t)out->shapes.size();
                    out->shapes.push_back(std::move(s));
                    out->stats.staticMeshes++;
                } else {
                    out->stats.staticMeshesFailed++;
                }
                meshShape[comp.staticMeshRef] = shape;
            }
            if (shape < 0) continue;

            MapPlacement pl;
            pl.shape = shape;
            pl.xform = xform;
            pl.coll = comp.coll;
            pl.visible = true;
            pl.collide = comp.blockRigidBody && comp.coll != CollChannel::None;
            out->placements.push_back(pl);
            out->stats.placements++;
            out->stats.chan[(int)pl.coll]++;
            if (pl.visible) {
                out->stats.visiblePlacements++;
                expandBounds(out, pl);
            }
            if (pl.collide) out->stats.collidePlacements++;
            continue;
        }

        // Blocking volumes: invisible convex collision only.
        if (comp.hasAggGeom) aggGeomSeen++;
        if (!comp.hasAggGeom || comp.ownerIdx < 0 || comp.ownerIdx >= (int32_t)actors.size())
            continue;
        const ue3::ActorRec& owner = actors[(size_t)comp.ownerIdx];
        if (verbose)
            std::fprintf(stderr, "[maploader] aggGeom comp %d owner='%s' cls='%s'\n", i,
                         owner.name.c_str(), owner.className.c_str());
        if (!owner.valid || !isBlockingVolumeClass(owner.className)) continue;

        std::vector<ConvexElem> elems;
        if (!ue3::readComponentConvexElems(pk, i, &elems, &cerr)) continue;
        for (const ConvexElem& ce : elems) {
            std::vector<V3> sv;
            std::vector<uint32_t> tris;
            sv.reserve(ce.verts.size());
            tris.reserve(ce.tris.size());
            for (const V3& v : ce.verts) sv.push_back(v);
            for (int32_t t : ce.tris) tris.push_back((uint32_t)t);
            if (tris.empty()) continue;
            MapShape s;
            buildShape(sv, tris, {}, {}, owner.name + "/convex", 0, &s);
            int32_t shape = (int32_t)out->shapes.size();
            out->shapes.push_back(std::move(s));
            out->stats.convexElems++;

            MapPlacement pl;
            pl.shape = shape;
            pl.xform = xform;
            pl.coll = comp.coll;
            pl.visible = false;
            pl.collide = comp.blockRigidBody && comp.coll != CollChannel::None;
            out->placements.push_back(pl);
            out->stats.placements++;
            out->stats.chan[(int)pl.coll]++;
            if (pl.collide) out->stats.collidePlacements++;
        }
    }

    out->stats.exports = n;
    if (verbose)
        std::fprintf(stderr, "[maploader] %s: components=%d parsed=%d aggGeom=%d convex=%d\n",
                     out->fileName.c_str(), componentsSeen, out->stats.objectsParsed,
                     aggGeomSeen, out->stats.convexElems);

    // ---- level BSP --------------------------------------------------------
    // Maps that build part of their geometry with brushes instead of static
    // meshes expose it through UModelComponent exports; skipping it would leave
    // holes in both the rendering and the collision.
    {
        std::vector<ue3::BspPiece> bsp;
        std::string berr;
        if (ue3::readLevelBsp(pk, &bsp, &berr)) {
            for (const ue3::BspPiece& piece : bsp) {
                if (piece.tris.empty()) continue;
                MapShape s;
                buildShape(piece.verts, piece.tris, {}, {}, "bsp", piece.materialRef, &s);
                int32_t shape = (int32_t)out->shapes.size();
                out->shapes.push_back(std::move(s));
                out->stats.bspTris += (int32_t)(piece.tris.size() / 3);

                MapPlacement pl;
                pl.shape = shape;
                pl.xform = M4::identity();  // the level BSP is world-space
                pl.coll = CollChannel::All;
                pl.visible = true;
                pl.collide = true;
                out->placements.push_back(pl);
                out->stats.placements++;
                out->stats.chan[(int)pl.coll]++;
                out->stats.visiblePlacements++;
                out->stats.collidePlacements++;
                expandBounds(out, pl);
            }
            if (verbose)
                std::fprintf(stderr, "[maploader] %s: level BSP %d pieces, %d triangles\n",
                             out->fileName.c_str(), (int)bsp.size(), out->stats.bspTris);
        } else if (verbose) {
            std::fprintf(stderr, "[maploader] %s: no level BSP (%s)\n",
                         out->fileName.c_str(), berr.c_str());
        }
    }

    // ---- sessions -------------------------------------------------------
    struct Pending {
        int32_t num;
        V3 pos;
        int32_t yaw;
    };
    std::map<int32_t, Pending> carSpawns;
    std::map<int32_t, Pending> ballSpawns;
    std::vector<Pending> playerStarts;
    std::vector<Pending> pathNodes;

    for (int32_t i = 0; i < n; i++) {
        const ue3::ActorRec& a = actors[(size_t)i];
        if (!a.valid) continue;
        // Spawn points are identified by AActor.Tag ("PlayerSpawn07"/"BallSpawn07");
        // some maps put them in the object name instead.
        int kind = 0, num = -1;
        auto classify = [&](const std::string& key) -> bool {
            if (key.empty()) return false;
            std::string l = toLower(key);
            if (startsWith(l, "playerspawn")) {
                kind = 1;
                num = trailingNumber(key);
                return true;
            }
            if (startsWith(l, "ballspawn")) {
                kind = 2;
                num = trailingNumber(key);
                return true;
            }
            return false;
        };
        if (!classify(a.tag) && !classify(a.name)) {
            if (a.className == "PlayerStart_TA" || a.className == "PlayerStart")
                playerStarts.push_back({-1, a.loc, a.yaw});
            // Any PathNode flavour (PathNode, PathNode_TA, RoadPathNode, ...):
            // they are the mapper's own navigation waypoints.
            else if (toLower(a.className).find("pathnode") != std::string::npos)
                pathNodes.push_back({-1, a.loc, a.yaw});
            continue;
        }
        if (num < 0) continue;
        Pending p{num, a.loc, a.yaw};
        if (kind == 1) {
            carSpawns[num] = p;
            out->stats.spawnActors++;
        } else {
            ballSpawns[num] = p;
            out->stats.ballSpawns++;
        }
    }

    if (!carSpawns.empty()) {
        for (const auto& kv : carSpawns) {
            MapSession s;
            char buf[16];
            std::snprintf(buf, sizeof buf, "%02d", kv.first);
            s.name = buf;
            s.carPos = kv.second.pos;
            s.carYawRad = (float)ue3::unitsToRad(kv.second.yaw);
            s.hasCar = true;
            auto bi = ballSpawns.find(kv.first);
            if (bi != ballSpawns.end()) {
                s.ballPos = bi->second.pos;
                s.hasBall = true;
            }
            out->sessions.push_back(s);
        }
    } else if (!playerStarts.empty()) {
        MapSession s;
        s.name = "PlayerStart";
        s.carPos = playerStarts.front().pos;
        s.carYawRad = (float)ue3::unitsToRad(playerStarts.front().yaw);
        s.hasCar = true;
        auto bi = ballSpawns.begin();
        if (bi != ballSpawns.end()) {
            s.ballPos = bi->second.pos;
            s.hasBall = true;
        }
        out->sessions.push_back(s);
    }

    // ---- derived checkpoints -------------------------------------------
    // Only Dribble-style maps ship a numbered spawn series (PlayerSpawn01..);
    // a map with a lone PlayerStart would otherwise have nothing to step
    // through. UE3 PathNodes are the mapper's own navigation points along the
    // intended route, so on those maps they become checkpoints. They are raw
    // points here - placement on solid ground, ordering and reach radii are
    // sorted out by sanitizeSessions() once every session is known.
    if (out->sessions.size() < 2 && !pathNodes.empty()) {
        int n2 = 0;
        for (const Pending& pn : pathNodes) {
            MapSession s;
            char buf[16];
            std::snprintf(buf, sizeof buf, "Nav%02d", ++n2);
            s.name = buf;
            s.carPos = pn.pos;
            s.carYawRad = (float)ue3::unitsToRad(pn.yaw);
            s.hasCar = true;
            s.derived = true;
            out->sessions.push_back(s);
        }
    }

    {
        std::string slog;
        sanitizeSessions(out, &slog);
        if (verbose && !slog.empty())
            std::fprintf(stderr, "[maploader] %s: sessions: %s\n", out->fileName.c_str(),
                         slog.c_str());
    }

    // ---- stats ----------------------------------------------------------
    out->stats.sessions = (int32_t)out->sessions.size();
    out->stats.shapes = (int32_t)out->shapes.size();
    out->stats.hasBounds = out->hasBounds;
    out->stats.boundsMin = out->boundsMin;
    out->stats.boundsMax = out->boundsMax;
    int64_t uniqueTris = 0, renderTris = 0, collideTris = 0;
    for (const MapShape& s : out->shapes) uniqueTris += (int64_t)s.idx.size() / 3;
    for (const MapPlacement& p : out->placements) {
        int64_t t = (int64_t)out->shapes[(size_t)p.shape].idx.size() / 3;
        if (p.visible) renderTris += t;
        if (p.collide) collideTris += t;
    }
    out->stats.uniqueTris = (int32_t)std::min<int64_t>(uniqueTris, INT32_MAX);
    out->stats.renderTris = (int32_t)std::min<int64_t>(renderTris, INT32_MAX);
    out->stats.collisionTris = (int32_t)std::min<int64_t>(collideTris, INT32_MAX);

    if (out->placements.empty()) {
        if (verbose)
            std::fprintf(stderr,
                         "[maploader] %s: exports=%d components=%d parsed=%d failed=%d "
                         "shapes=%d firstFail=%s\n",
                         out->fileName.c_str(), n, componentsSeen,
                         out->stats.objectsParsed, out->stats.objectsFailed,
                         out->stats.shapes, firstFail.c_str());
        *err = "no geometry found in " + out->fileName;
        if (!firstFail.empty()) *err += " (" + firstFail + ")";
        if (verbose && err->size() < 400) *err += " [verbose]";
        if (!verbose && componentsSeen == 0) *err += " (no components found)";
        return false;
    }
    if (out->sessions.empty()) {
        // Still playable: fall back to the default reset position.
        MapSession s;
        s.name = "Default";
        out->sessions.push_back(s);
    }
    return true;
}

std::string describeMap(const MapData& m) {
    std::string s;
    s += fmtStr("file            : %s\n", m.fileName.c_str());
    s += fmtStr("path            : %s\n", m.path.c_str());
    s += fmtStr("exports         : %d\n", m.stats.exports);
    s += fmtStr("components      : %d parsed, %d failed\n", m.stats.objectsParsed,
                m.stats.objectsFailed);
    s += fmtStr("static meshes   : %d parsed, %d failed\n", m.stats.staticMeshes,
                m.stats.staticMeshesFailed);
    s += fmtStr("shapes          : %d (%d convex)\n", m.stats.shapes, m.stats.convexElems);
    if (m.stats.bspTris)
        s += fmtStr("level BSP       : %d triangles\n", m.stats.bspTris);
    s += fmtStr("placements      : %d (visible %d, colliding %d)\n", m.stats.placements,
                m.stats.visiblePlacements, m.stats.collidePlacements);
    s += fmtStr("channels        : all=%d vehicle-only=%d ball-only=%d none=%d\n",
                m.stats.chan[0], m.stats.chan[1], m.stats.chan[2], m.stats.chan[3]);
    s += fmtStr("triangles       : %d unique, %d rendered, %d colliding (instanced)\n",
                m.stats.uniqueTris, m.stats.renderTris, m.stats.collisionTris);
    if (m.hasBounds)
        s += fmtStr("bounds          : (%.0f %.0f %.0f) .. (%.0f %.0f %.0f)\n",
                    m.boundsMin.x, m.boundsMin.y, m.boundsMin.z, m.boundsMax.x,
                    m.boundsMax.y, m.boundsMax.z);
    else
        s += "bounds          : (none)\n";
    s += fmtStr("spawn actors    : %d car, %d ball\n", m.stats.spawnActors,
                m.stats.ballSpawns);
    s += fmtStr("sessions        : %d%s\n", m.stats.sessions,
                m.stats.derivedSessions
                    ? fmtStr(" (%d derived from the map's navigation points)",
                             m.stats.derivedSessions)
                        .c_str()
                    : "");
    for (const MapSession& ses : m.sessions) {
        s += fmtStr("  [%s] car=(%.0f, %.0f, %.0f) yaw=%.1fdeg reach=%.0f", ses.name.c_str(),
                    ses.carPos.x, ses.carPos.y, ses.carPos.z,
                    ses.carYawRad * 57.2957795f, ses.reach);
        if (ses.hasBall)
            s += fmtStr(" ball=(%.0f, %.0f, %.0f)\n", ses.ballPos.x, ses.ballPos.y,
                        ses.ballPos.z);
        else
            s += " ball=<unchanged>\n";
    }
    return s;
}

}  // namespace maploader
