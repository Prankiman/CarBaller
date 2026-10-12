#include "map_sessions.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "map_data.h"

namespace maploader {
namespace {

// Where a car comes to rest when it is set down on a floor: its origin sits
// this far above the surface (measured on the default arena and on the maps'
// own spawn points, which all settle 33-45 uu below their authored z).
constexpr float kCarRestZ = 18.f;
// A ceiling this close over a floor means the point is inside the solid, and
// the car has to be lifted out onto the top of it.
constexpr float kBuriedGap = 600.f;
// How far below a spawn to look for ground before calling it a hole.
constexpr float kDownWin = 2500.f;
// |n.z| below this and the surface is a wall, not ground.
constexpr float kWalkable = 0.6f;
// What counts as standing room: a level, walkable patch the size of the car
// with nothing closing in overhead. The Octane is 118 x 84 uu, so the probes
// sit on the corners of its footprint.
constexpr float kFlat = 0.85f;      // preferred slope (cos 32 deg)
constexpr float kCarHalfLong = 59.f, kCarHalfWide = 42.f;
constexpr float kFootSlack = 40.f;   // allowed height difference across it
constexpr float kSlopeSlack = 150.f;  // ... and across the relaxed, sloped pass
// Clear space over the floor. 250 is what the car needs; the camera hangs
// back and above it, so a spawn under a low overhang plays badly even when
// the car itself fits - open space is preferred, tight space is the fallback.
constexpr float kMinGap = 250.f, kRoomyGap = 700.f;
// Two derived checkpoints this close together are the same checkpoint.
constexpr float kDedupe = 600.f;
// A map that shipped its own numbered spawn series keeps its precision.
constexpr float kAuthoredReach = 600.f;
// Derived checkpoints get a reach that scales with how far apart the route
// runs, so a course whose points sit 15 km apart stays completable.
constexpr float kMinReach = 1000.f, kMaxReach = 5000.f;

struct Inst {
    const MapShape* shape = nullptr;
    M4 xform;
    V3 mn, mx;  // world AABB
};

// Vertical queries over the map's car-solid triangles: what is directly under
// (or over) a point, and how much room there is to stand.
class GroundQuery {
public:
    explicit GroundQuery(const MapData& m) {
        for (const MapPlacement& pl : m.placements) {
            if (!pl.collide) continue;                       // not a rigid body
            if (pl.coll == CollChannel::BallOnly) continue;  // the car passes through it
            if (pl.shape < 0 || (size_t)pl.shape >= m.shapes.size()) continue;
            const MapShape& sh = m.shapes[(size_t)pl.shape];
            if (sh.idx.size() < 3 || sh.verts.empty()) continue;
            Inst in;
            in.shape = &sh;
            in.xform = pl.xform;
            V3 mn(3.4e38f, 3.4e38f, 3.4e38f), mx(-3.4e38f, -3.4e38f, -3.4e38f);
            for (int c = 0; c < 8; c++) {
                const V3 corner((c & 1) ? sh.boundsMax.x : sh.boundsMin.x,
                                (c & 2) ? sh.boundsMax.y : sh.boundsMin.y,
                                (c & 4) ? sh.boundsMax.z : sh.boundsMin.z);
                const V3 w = pl.xform.mulPoint(corner);
                mn.x = std::min(mn.x, w.x); mx.x = std::max(mx.x, w.x);
                mn.y = std::min(mn.y, w.y); mx.y = std::max(mx.y, w.y);
                mn.z = std::min(mn.z, w.z); mx.z = std::max(mx.z, w.z);
            }
            in.mn = mn;
            in.mx = mx;
            insts_.push_back(in);
        }
    }

    // Topmost (wantMax) or lowest surface whose XY column contains p and that
    // lies in [zLo, zHi]. Optionally reports |n.z| (how level it is).
    bool columnHit(const V3& p, float zLo, float zHi, bool wantMax, float* outZ,
                   float* outNz = nullptr) const {
        bool found = false;
        float best = wantMax ? -3.4e38f : 3.4e38f, bestNz = 0.f;
        for (const Inst& in : insts_) {
            if (p.x < in.mn.x || p.x > in.mx.x || p.y < in.mn.y || p.y > in.mx.y) continue;
            if (in.mx.z < zLo || in.mn.z > zHi) continue;
            forEachTriangle(in, [&](const V3& a, const V3& b, const V3& c) {
                // XY barycentric test; vertical faces have no XY area and are
                // skipped, so this only ever reports floors and ceilings.
                const float area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
                if (std::fabs(area) < 1e-3f) return;
                const float u = ((p.x - a.x) * (c.y - a.y) - (p.y - a.y) * (c.x - a.x)) / area;
                const float v = ((b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x)) / area;
                if (u < 0.f || v < 0.f || u + v > 1.f) return;
                const float z = a.z + u * (c.z - a.z) + v * (b.z - a.z);
                if (z < zLo || z > zHi) return;
                if (!found || (wantMax ? z > best : z < best)) {
                    found = true;
                    best = z;
                    if (outNz) bestNz = std::fabs((b - a).cross(c - a).norm().z);
                }
            });
        }
        if (found) {
            *outZ = best;
            if (outNz) *outNz = bestNz;
        }
        return found;
    }

    // Can a car stand at (x, y) somewhere in [zLo, zHi]? The floor under the
    // centre has to be level with the four corners of the car's footprint -
    // these maps are thin panels, so a checkpoint on an edge tips the car
    // over - and nothing may close in overhead. Returns the surface z.
    bool stands(const V3& c, float zLo, float zHi, float yaw, float minNz, float slack,
                float minGap, float* outZ) const {
        float z0 = 0.f, nz = 0.f;
        if (!columnHit(c, zLo, zHi, true, &z0, &nz)) return false;
        if (nz < minNz) return false;
        const float cf = std::cos(yaw), sf = std::sin(yaw);
        const float off[4][2] = {
            {(kCarHalfLong * cf - kCarHalfWide * sf), (kCarHalfLong * sf + kCarHalfWide * cf)},
            {(kCarHalfLong * cf + kCarHalfWide * sf), (kCarHalfLong * sf - kCarHalfWide * cf)},
            {(-kCarHalfLong * cf - kCarHalfWide * sf), (-kCarHalfLong * sf + kCarHalfWide * cf)},
            {(-kCarHalfLong * cf + kCarHalfWide * sf), (-kCarHalfLong * sf - kCarHalfWide * cf)},
        };
        for (const auto& o : off) {
            const V3 s(c.x + o[0], c.y + o[1], 0.f);
            float z = 0.f, sn = 0.f;
            if (!columnHit(s, z0 - slack, z0 + slack, true, &z, &sn)) return false;
            if (sn < minNz) return false;
        }
        float ceil = 0.f;
        if (columnHit(c, z0 + 1.f, z0 + minGap, false, &ceil)) return false;  // low ceiling
        if (outZ) *outZ = z0;
        return true;
    }

private:
    // Transforms one shape's triangles to world space as it walks them.
    template <typename F>
    static void forEachTriangle(const Inst& in, F&& fn) {
        const MapShape& sh = *in.shape;
        for (size_t t = 0; t + 2 < sh.idx.size(); t += 3) {
            const uint32_t i0 = sh.idx[t], i1 = sh.idx[t + 1], i2 = sh.idx[t + 2];
            if (i0 >= sh.verts.size() || i1 >= sh.verts.size() || i2 >= sh.verts.size()) continue;
            const MapVertex& v0 = sh.verts[i0];
            const MapVertex& v1 = sh.verts[i1];
            const MapVertex& v2 = sh.verts[i2];
            fn(in.xform.mulPoint(V3(v0.px, v0.py, v0.pz)),
               in.xform.mulPoint(V3(v1.px, v1.py, v1.pz)),
               in.xform.mulPoint(V3(v2.px, v2.py, v2.pz)));
        }
    }

    std::vector<Inst> insts_;
};

float dist2(const V3& a, const V3& b) {
    const V3 d = a - b;
    return d.dot(d);
}

// Walk outwards from a point looking for somewhere a car can actually sit,
// starting with the point itself. Returns the spawn position (surface + rest
// height) in `out`.
bool findStand(const GroundQuery& q, const V3& from, float yaw, float minNz, float slack,
               float minGap, V3* out) {
    static const float kRadii[] = {150.f, 300.f, 500.f, 750.f, 1050.f, 1400.f, 1800.f, 2300.f};
    // The ground to stand on is the one under the point (a little above counts
    // - the point may be embedded by a few uu), never a ceiling: reading a
    // slab's underside as the floor would place the car inside it.
    const float zLo = from.z - kDownWin, zHi = from.z + kFootSlack;
    float z0 = 0.f;
    if (q.stands(from, zLo, zHi, yaw, minNz, slack, minGap, &z0)) {
        *out = V3(from.x, from.y, z0 + kCarRestZ);
        return true;
    }
    for (const float r : kRadii) {
        for (int k = 0; k < 8; k++) {
            const float a = (float)k * (float)M_PI / 4.f;
            const V3 c(from.x + std::cos(a) * r, from.y + std::sin(a) * r, 0.f);
            if (q.stands(c, zLo, zHi, yaw, minNz, slack, minGap, &z0)) {
                *out = V3(c.x, c.y, z0 + kCarRestZ);
                return true;
            }
        }
    }
    return false;
}

}  // namespace

int sanitizeSessions(MapData* m, std::string* log) {
    if (!m || m->sessions.empty()) return 0;

    // A map that shipped a numbered spawn series (PlayerSpawn01..) keeps its
    // own checkpoints untouched: their numbering, order and precision are the
    // mapper's. Only derived ones get re-ordered, collapsed and resized.
    const bool hasAuthored = m->stats.spawnActors > 0;

    GroundQuery q(*m);
    int lifted = 0, moved = 0, relocated = 0, dropped = 0, collapsed = 0;

    // ---- 1. put every spawn on ground a car can sit on ----------------
    std::vector<MapSession> keep;
    keep.reserve(m->sessions.size());
    for (const MapSession& orig : m->sessions) {
        MapSession s = orig;
        if (!s.hasCar) {
            keep.push_back(s);
            continue;
        }
        const V3 p = s.carPos;

        // Inside a slab (a point is only ever this close to a ceiling when
        // something solid is around it): come out through the top first.
        V3 from = p;
        float zBelow = 0.f, zAbove = 0.f;
        const bool below = q.columnHit(p, p.z - kDownWin, p.z + 1.f, true, &zBelow);
        const bool above = below && q.columnHit(p, p.z, p.z + kDownWin, false, &zAbove);
        const bool buried = below && above && (zAbove - zBelow) < kBuriedGap;
        if (buried) {
            from.z = zAbove + kCarRestZ;
            lifted++;
        }

        V3 pos;
        // Level and open first, then level and tight, then sloped either way.
        if (findStand(q, from, s.carYawRad, kFlat, kFootSlack, kRoomyGap, &pos) ||
            findStand(q, from, s.carYawRad, kFlat, kFootSlack, kMinGap, &pos) ||
            findStand(q, from, s.carYawRad, kWalkable, kSlopeSlack, kRoomyGap, &pos) ||
            findStand(q, from, s.carYawRad, kWalkable, kSlopeSlack, kMinGap, &pos)) {
            if (dist2(pos, p) > 1.f) {
                moved++;
                if ((pos.x - p.x) * (pos.x - p.x) + (pos.y - p.y) * (pos.y - p.y) > 1.f)
                    relocated++;
            }
            s.carPos = pos;
            if (s.hasBall) s.ballPos += pos - p;  // keep the pair together
            keep.push_back(s);
            continue;
        }

        // Nowhere at all nearby. A start point still has to exist, so fall
        // back to whatever ground sits under the middle of the map; anything
        // else (a derived checkpoint over the void) is not a checkpoint.
        if (!s.derived) {
            const V3 mid = (m->boundsMin + m->boundsMax) * 0.5f;
            float z = 0.f;
            if (q.columnHit(V3(mid.x, mid.y, m->boundsMax.z), m->boundsMin.z - kDownWin,
                            m->boundsMax.z, true, &z)) {
                s.carPos = V3(mid.x, mid.y, z + kCarRestZ);
                moved++;
                relocated++;
                keep.push_back(s);
                continue;
            }
            keep.push_back(s);  // truly nowhere: keep the authored point
            continue;
        }
        dropped++;
    }
    m->sessions.swap(keep);

    // ---- 2. collapse near-duplicates ----------------------------------
    // A mapper's PathNodes cluster several per platform; two points inside
    // each other's reach would fight over the session every frame.
    if (!hasAuthored && m->sessions.size() > 1) {
        std::vector<MapSession> uniq;
        uniq.push_back(m->sessions[0]);  // the map's own start always stays
        for (size_t i = 1; i < m->sessions.size(); i++) {
            bool dup = false;
            for (const MapSession& k : uniq) {
                if (dist2(k.carPos, m->sessions[i].carPos) <= kDedupe * kDedupe) {
                    dup = true;
                    break;
                }
            }
            if (!dup) uniq.push_back(m->sessions[i]);
        }
        collapsed += (int)(m->sessions.size() - uniq.size());
        m->sessions.swap(uniq);
    }

    // ---- 3. order the derived ones into a route -----------------------
    // Nearest-neighbour from the map's own start, so prev/next steps along a
    // sensible path instead of package order. Authored series keep the
    // mapper's numbering.
    if (!hasAuthored && m->sessions.size() > 1) {
        std::vector<MapSession> route;
        std::vector<MapSession> rest(m->sessions.begin() + 1, m->sessions.end());
        V3 cur = m->sessions.front().carPos;
        route.push_back(m->sessions.front());
        while (!rest.empty()) {
            size_t best = 0;
            float bestD = 3.4e38f;
            for (size_t i = 0; i < rest.size(); i++) {
                const float d = dist2(cur, rest[i].carPos);
                if (d < bestD) {
                    bestD = d;
                    best = i;
                }
            }
            route.push_back(rest[best]);
            cur = rest[best].carPos;
            rest.erase(rest.begin() + (long)best);
        }
        m->sessions.swap(route);
    }

    // ---- 4. facing and reach ------------------------------------------
    for (size_t i = 0; i < m->sessions.size(); i++) {
        MapSession& s = m->sessions[i];
        if (s.derived) {
            // Face the next one along the route; the last looks back at the
            // one it came from rather than into whatever is ahead of it.
            const V3 to = (i + 1 < m->sessions.size())
                              ? m->sessions[i + 1].carPos - s.carPos
                              : (i > 0 ? m->sessions[i - 1].carPos - s.carPos : V3(0.f, 1.f, 0.f));
            if (to.len2d() > 1.f) s.carYawRad = std::atan2(to.y, to.x);
        }
        s.reach = kAuthoredReach;
        if (!hasAuthored) {
            float nn = 3.4e38f;
            for (size_t j = 0; j < m->sessions.size(); j++) {
                if (j == i) continue;
                nn = std::min(nn, dist2(s.carPos, m->sessions[j].carPos));
            }
            if (nn < 3.4e38f) s.reach = clampf(0.5f * std::sqrt(nn), kMinReach, kMaxReach);
        }
    }

    m->stats.sessions = (int32_t)m->sessions.size();
    int derived = 0;
    for (const MapSession& s : m->sessions)
        if (s.derived) derived++;
    m->stats.derivedSessions = derived;

    if (log) {
        char buf[256];
        std::snprintf(buf, sizeof buf,
                      "%d placed on ground (%d lifted out of solid, %d moved sideways), "
                      "%d collapsed as duplicates, %d dropped over nothing, %d kept",
                      moved, lifted, relocated, collapsed, dropped, (int)m->sessions.size());
        *log = buf;
    }
    return dropped;
}

}  // namespace maploader
