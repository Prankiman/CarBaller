#pragma once
// CPU mesh construction helpers (low-poly procedural geometry).

#include "../core/math.h"
#include "gl_utils.h"

#include <vector>
#include <utility>
#include <cmath>

struct Col {
    float r = 1, g = 1, b = 1, a = 1;
    Col() = default;
    Col(float r_, float g_, float b_, float a_ = 1) : r(r_), g(g_), b(b_), a(a_) {}
};

struct MeshBuilder {
    std::vector<Vertex> verts;
    std::vector<uint32_t> idx;

    void clear() { verts.clear(); idx.clear(); }

    uint32_t addVert(const V3& p, const V3& n, float u, float v, const Col& c) {
        verts.push_back({p.x, p.y, p.z, n.x, n.y, n.z, u, v, c.r, c.g, c.b, c.a});
        return uint32_t(verts.size() - 1);
    }

    // Triangle with orientation fix: if its normal disagrees with `want`, flip.
    void triOriented(uint32_t a, uint32_t b, uint32_t c, const V3& want) {
        const Vertex& va = verts[a];
        const Vertex& vb = verts[b];
        const Vertex& vc = verts[c];
        V3 e1(vb.px - va.px, vb.py - va.py, vb.pz - va.pz);
        V3 e2(vc.px - va.px, vc.py - va.py, vc.pz - va.pz);
        V3 n = e1.cross(e2);
        if (n.dot(want) < 0) std::swap(b, c);
        idx.push_back(a);
        idx.push_back(b);
        idx.push_back(c);
    }

    void quadOriented(const V3& p0, const V3& p1, const V3& p2, const V3& p3,
                      const V3& wantN, const Col& c,
                      const float uv[8] = nullptr) {
        V3 n = wantN.norm();
        float z0[2] = {0, 0}, z1[2] = {1, 0}, z2[2] = {1, 1}, z3[2] = {0, 1};
        if (uv) {
            z0[0] = uv[0]; z0[1] = uv[1];
            z1[0] = uv[2]; z1[1] = uv[3];
            z2[0] = uv[4]; z2[1] = uv[5];
            z3[0] = uv[6]; z3[1] = uv[7];
        }
        uint32_t a = addVert(p0, n, z0[0], z0[1], c);
        uint32_t b = addVert(p1, n, z1[0], z1[1], c);
        uint32_t d = addVert(p2, n, z2[0], z2[1], c);
        uint32_t e = addVert(p3, n, z3[0], z3[1], c);
        triOriented(a, b, d, wantN);
        triOriented(a, d, e, wantN);
    }

    // Axis-aligned box
    void aabb(const V3& mn, const V3& mx, const Col& c) {
        auto P = [&](float x, float y, float z) { return V3(x, y, z); };
        // +X
        quadOriented(P(mx.x, mn.y, mn.z), P(mx.x, mx.y, mn.z), P(mx.x, mx.y, mx.z), P(mx.x, mn.y, mx.z), {1, 0, 0}, c);
        // -X
        quadOriented(P(mn.x, mn.y, mn.z), P(mn.x, mn.y, mx.z), P(mn.x, mx.y, mx.z), P(mn.x, mx.y, mn.z), {-1, 0, 0}, c);
        // +Y
        quadOriented(P(mn.x, mx.y, mn.z), P(mn.x, mx.y, mx.z), P(mx.x, mx.y, mx.z), P(mx.x, mx.y, mn.z), {0, 1, 0}, c);
        // -Y
        quadOriented(P(mn.x, mn.y, mn.z), P(mx.x, mn.y, mn.z), P(mx.x, mn.y, mx.z), P(mn.x, mn.y, mx.z), {0, -1, 0}, c);
        // +Z
        quadOriented(P(mn.x, mn.y, mx.z), P(mx.x, mn.y, mx.z), P(mx.x, mx.y, mx.z), P(mn.x, mx.y, mx.z), {0, 0, 1}, c);
        // -Z
        quadOriented(P(mn.x, mn.y, mn.z), P(mn.x, mx.y, mn.z), P(mx.x, mx.y, mn.z), P(mx.x, mn.y, mn.z), {0, 0, -1}, c);
    }

    // Flat quad, normal computed from winding p0->p1->p2
    void quad(const V3& p0, const V3& p1, const V3& p2, const V3& p3, const Col& c,
              const float uv[8] = nullptr) {
        V3 n = (p1 - p0).cross(p2 - p0);
        if (n.len() < 1e-9f) n = (p2 - p1).cross(p3 - p1);
        quadOriented(p0, p1, p2, p3, n.norm(), c, uv);
    }

    // ---- Extrude an x-monotone XZ profile along Y.
    // bottom: rear->front chain of (x,z); top: rear->front chain of (x,z).
    // Outline is CCW in (x,z) when x is right and z is up.
    // wallColFn: optional per-point color override for side walls.
    void prismXZ(const std::vector<std::pair<float, float>>& bottom,
                 const std::vector<std::pair<float, float>>& top,
                 float y0, float y1,
                 const Col& wallColFn, const Col& capCol,
                 const std::vector<Col>* bottomCols = nullptr,
                 const std::vector<Col>* topCols = nullptr,
                 bool skipBottomWalls = false) {
        auto P = [&](float x, float z, float y) { return V3(x, y, z); };

        auto edgeQuad = [&](const std::pair<float, float>& a, const std::pair<float, float>& b,
                            const Col& col) {
            // outward normal for CCW outline
            float dx = b.first - a.first, dz = b.second - a.second;
            V3 out(dz, 0, -dx);
            if (out.len() < 1e-9f) return;
            // verts: (a,y1) (b,y1) (b,y0) (a,y0) -> normal (dz,0,-dx) for y1>y0
            quadOriented(P(a.first, a.second, y1), P(b.first, b.second, y1),
                         P(b.first, b.second, y0), P(a.first, a.second, y0),
                         out, col);
        };

        // bottom chain walls: rear->front
        for (size_t i = 0; i + 1 < bottom.size(); i++) {
            bool isArch = std::fabs(bottom[i].second - bottom[i + 1].second) > 0.5f ||
                          bottom[i].second > 2.6f;
            if (skipBottomWalls && !isArch) continue;
            Col c = wallColFn;
            if (bottomCols && i < bottomCols->size()) c = (*bottomCols)[i];
            edgeQuad(bottom[i], bottom[i + 1], c);
        }
        // front connector: bottom.back (x max) -> top.back
        {
            Col c = wallColFn;
            if (bottomCols && !bottomCols->empty()) c = bottomCols->back();
            edgeQuad(bottom.back(), top.back(), c);
        }
        // top chain walls: front->rear (reverse)
        for (size_t i = top.size(); i-- > 1;) {
            Col c = wallColFn;
            if (topCols && i < topCols->size()) c = (*topCols)[i];
            edgeQuad(top[i], top[i - 1], c);
        }
        // rear connector: top rear -> bottom rear
        {
            Col c = wallColFn;
            if (topCols && !topCols->empty()) c = topCols->front();
            edgeQuad(top.front(), bottom.front(), c);
        }

        // caps: strip between bottom (rear->front, x asc) and top (rear->front, x asc)
        auto cap = [&](float y, const V3& wantN) {
            size_t i = 0, j = 0;
            size_t nb = bottom.size(), nt = top.size();
            auto emit = [&](const std::pair<float, float>& a,
                            const std::pair<float, float>& b,
                            const std::pair<float, float>& c) {
                uint32_t ia = addVert(V3(a.first, y, a.second), wantN, 0, 0, capCol);
                uint32_t ib = addVert(V3(b.first, y, b.second), wantN, 0, 0, capCol);
                uint32_t ic = addVert(V3(c.first, y, c.second), wantN, 0, 0, capCol);
                triOriented(ia, ib, ic, wantN);
            };
            while (i < nb - 1 || j < nt - 1) {
                bool advanceBottom;
                if (i >= nb - 1) advanceBottom = false;
                else if (j >= nt - 1) advanceBottom = true;
                else advanceBottom = (bottom[i + 1].first <= top[j + 1].first);
                if (advanceBottom) {
                    emit(bottom[i], bottom[i + 1], top[j]);
                    i++;
                } else {
                    emit(bottom[i], top[j + 1], top[j]);
                    j++;
                }
            }
        };
        cap(y1, {0, 1, 0});   // right side (y1) faces +Y
        cap(y0, {0, -1, 0});
    }

    // Cylinder/cone along segment a->b (wheel axle etc.). aUV: wrap texture around.
    void cylinder(const V3& a, const V3& b, float r0, float r1, int segs,
                  const Col& c, bool capA, bool capB, bool wrapUV = false) {
        V3 axis = (b - a).norm();
        V3 up = std::fabs(axis.z) > 0.9f ? V3(1, 0, 0) : V3(0, 0, 1);
        V3 u = axis.cross(up).norm();
        V3 v = axis.cross(u).norm();
        std::vector<uint32_t> ringA, ringB;
        for (int i = 0; i <= segs; i++) {
            float t = float(i) / segs;
            float ang = t * 2 * (float)M_PI;
            V3 radial = u * std::cos(ang) + v * std::sin(ang);
            float uu = wrapUV ? t : 0;
            ringA.push_back(addVert(a + radial * r0, radial, uu, 0, c));
            ringB.push_back(addVert(b + radial * r1, radial, uu, 1, c));
        }
        for (int i = 0; i < segs; i++) {
            V3 radial(verts[ringA[i]].nx, verts[ringA[i]].ny, verts[ringA[i]].nz);
            triOriented(ringA[i], ringA[i + 1], ringB[i + 1], radial);
            triOriented(ringA[i], ringB[i + 1], ringB[i], radial);
        }
        auto cap = [&](const V3& center, const std::vector<uint32_t>& ring, const V3& n) {
            uint32_t cIdx = addVert(center, n, 0.5f, 0.5f, c);
            for (int i = 0; i < segs; i++)
                triOriented(cIdx, ring[i], ring[i + 1], n);
        };
        if (capA) cap(a, ringA, axis * -1.0f);
        if (capB) cap(b, ringB, axis);
    }

    // UV sphere centered at c. uvForTexture maps equirectangular u,v.
    void sphere(const V3& c, float r, int lonSegs, int latSegs, const Col& col,
                bool uvForTexture = false) {
        std::vector<std::vector<uint32_t>> rows(latSegs + 1);
        for (int lat = 0; lat <= latSegs; lat++) {
            float v = float(lat) / latSegs;
            float phi = v * (float)M_PI;  // 0..pi (north..south)
            for (int lon = 0; lon <= lonSegs; lon++) {
                float u = float(lon) / lonSegs;
                float th = u * 2 * (float)M_PI;
                V3 n(std::sin(phi) * std::cos(th), std::sin(phi) * std::sin(th), std::cos(phi));
                V3 p = c + n * r;
                float uu = uvForTexture ? u : 0;
                float vv = uvForTexture ? v : 0;
                rows[lat].push_back(addVert(p, n, uu, vv, col));
            }
        }
        for (int lat = 0; lat < latSegs; lat++) {
            for (int lon = 0; lon < lonSegs; lon++) {
                uint32_t a = rows[lat][lon], b = rows[lat + 1][lon];
                uint32_t cc = rows[lat + 1][lon + 1], d = rows[lat][lon + 1];
                const Vertex& va = verts[a];
                V3 radial(va.px - c.x, va.py - c.y, va.pz - c.z);
                triOriented(a, b, cc, radial);
                triOriented(a, cc, d, radial);
            }
        }
    }

    // Flat annulus on the XY plane at height z (normal +Z), alpha ramps over width.
    void ringZ(const V3& center, float r0, float r1, int segs,
               const Col& inner, const Col& outer) {
        std::vector<uint32_t> in, out;
        for (int i = 0; i <= segs; i++) {
            float t = float(i) / segs;
            float ang = t * 2 * (float)M_PI;
            V3 dir(std::cos(ang), std::sin(ang), 0);
            in.push_back(addVert(center + dir * r0, {0, 0, 1}, 0, 0, inner));
            out.push_back(addVert(center + dir * r1, {0, 0, 1}, 1, 0, outer));
        }
        for (int i = 0; i < segs; i++) {
            triOriented(in[i], out[i], out[i + 1], {0, 0, 1});
            triOriented(in[i], out[i + 1], in[i + 1], {0, 0, 1});
        }
    }

    // Vertical quad strip helper for field markings on the floor (thin boxes)
    void flatRect(const V3& center, float halfA, float halfB, float z, const Col& c) {
        // rect in XY centered at center.xy
        quad(V3(center.x - halfA, center.y - halfB, z),
             V3(center.x + halfA, center.y - halfB, z),
             V3(center.x + halfA, center.y + halfB, z),
             V3(center.x - halfA, center.y + halfB, z), c);
    }
};
