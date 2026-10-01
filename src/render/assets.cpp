#include "assets.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <vector>

namespace {

constexpr float BT_TO_UU = 50.0f;

bool readCmf(const std::string& path, std::vector<int32_t>& tris, std::vector<float>& pts) {
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    int32_t nt = 0, nv = 0;
    bool ok = std::fread(&nt, 4, 1, f) == 1 && std::fread(&nv, 4, 1, f) == 1;
    if (!ok || nt <= 0 || nv <= 0 || nt > 1000000 || nv > 1000000) {
        std::fclose(f);
        return false;
    }
    tris.resize(size_t(nt) * 3);
    pts.resize(size_t(nv) * 3);
    ok = std::fread(tris.data(), 4, tris.size(), f) == tris.size() &&
         std::fread(pts.data(), 4, pts.size(), f) == pts.size();
    std::fclose(f);
    return ok;
}

Col arenaColForPos(const V3& p) {
    if (p.y > 5150) return {0.33f, 0.20f, 0.15f, 1};   // orange net
    if (p.y < -5150) return {0.15f, 0.20f, 0.33f, 1};  // blue net
    return {0.200f, 0.205f, 0.235f, 1};                // ramps / extra geometry
}

void appendCmf(MeshBuilder& mb, const std::string& path) {
    std::vector<int32_t> tris;
    std::vector<float> pts;
    if (!readCmf(path, tris, pts)) {
        std::fprintf(stderr, "[assets] failed to read %s\n", path.c_str());
        return;
    }
    const size_t nv = pts.size() / 3;
    uint32_t base = uint32_t(mb.verts.size());
    std::vector<V3> nrm(nv, V3(0, 0, 0));

    for (size_t i = 0; i < nv; i++) {
        V3 p(pts[i * 3] * BT_TO_UU, pts[i * 3 + 1] * BT_TO_UU, pts[i * 3 + 2] * BT_TO_UU);
        mb.addVert(p, V3(0, 0, 1), 0, 0, arenaColForPos(p));
    }
    for (size_t t = 0; t + 2 < tris.size(); t += 3) {
        uint32_t i0 = uint32_t(tris[t]), i1 = uint32_t(tris[t + 1]), i2 = uint32_t(tris[t + 2]);
        if (i0 >= nv || i1 >= nv || i2 >= nv) continue;
        const Vertex& a = mb.verts[base + i0];
        const Vertex& b = mb.verts[base + i1];
        const Vertex& c = mb.verts[base + i2];
        V3 e1(b.px - a.px, b.py - a.py, b.pz - a.pz);
        V3 e2(c.px - a.px, c.py - a.py, c.pz - a.pz);
        V3 fn = e1.cross(e2);
        if (fn.len() < 1e-9f) continue;
        nrm[i0] += fn;
        nrm[i1] += fn;
        nrm[i2] += fn;
        mb.idx.push_back(base + i0);
        mb.idx.push_back(base + i1);
        mb.idx.push_back(base + i2);
    }
    // angle-limited smoothing: only merge face normals within ~50 degrees
    std::vector<std::vector<V3>> adj(nv);
    std::vector<int> vertTris(nv, 0);
    for (size_t t = 0; t + 2 < tris.size(); t += 3) {
        uint32_t i0 = uint32_t(tris[t]), i1 = uint32_t(tris[t + 1]), i2 = uint32_t(tris[t + 2]);
        if (i0 >= nv || i1 >= nv || i2 >= nv) continue;
        const Vertex& a = mb.verts[base + i0];
        const Vertex& b = mb.verts[base + i1];
        const Vertex& c = mb.verts[base + i2];
        V3 e1(b.px - a.px, b.py - a.py, b.pz - a.pz);
        V3 e2(c.px - a.px, c.py - a.py, c.pz - a.pz);
        V3 fn = e1.cross(e2).norm();
        adj[i0].push_back(fn);
        adj[i1].push_back(fn);
        adj[i2].push_back(fn);
    }
    for (size_t i = 0; i < nv; i++) {
        if (adj[i].empty()) continue;
        V3 ref = adj[i][0];
        V3 acc(0, 0, 0);
        for (const V3& n : adj[i])
            if (n.dot(ref) > 0.64f) acc += n;   // ~50 deg
        V3 out = acc.norm();
        if (out.len() < 0.5f) out = ref;
        mb.verts[base + i].nx = out.x;
        mb.verts[base + i].ny = out.y;
        mb.verts[base + i].nz = out.z;
    }
}

void buildArenaFloor(MeshBuilder& mb) {
    const Col floorC(0.105f, 0.140f, 0.215f, 1);
    const float X = 4096, Y = 5120;

    // floor (normal +Z)
    mb.quad({-X, -Y, 0}, {X, -Y, 0}, {X, Y, 0}, {-X, Y, 0}, floorC);
}

void buildArenaShell(MeshBuilder& mb, const std::string& meshDir) {
    const Col ceilC(0.085f, 0.095f, 0.125f, 1);
    const Col wallC(0.165f, 0.170f, 0.205f, 1);

    const float X = 4096, Y = 5120, Z = 2044;

    // ceiling (normal -Z)
    mb.quad({-X, -Y, Z}, {-X, Y, Z}, {X, Y, Z}, {X, -Y, Z}, ceilC);
    // side walls (inward normals)
    mb.quad({X, -Y, 0}, {X, -Y, Z}, {X, Y, Z}, {X, Y, 0}, wallC);
    mb.quad({-X, -Y, 0}, {-X, Y, 0}, {-X, Y, Z}, {-X, -Y, Z}, wallC);

    for (int i = 0; i < 16; i++) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "/soccar/mesh_%d.cmf", i);
        appendCmf(mb, meshDir + buf);
    }
}

void buildMarkings(MeshBuilder& mb) {
    const Col line(0.80f, 0.86f, 0.95f, 0.42f);
    mb.flatRect(V3(0, 0, 0), 4096, 22, 2, line);                 // halfway line
    mb.ringZ({0, 0, 2}, 886, 930, 72, line, line);          // center circle
    for (int s = -1; s <= 1; s += 2) {
        mb.flatRect(V3(0, float(s) * 4096, 0), 1024, 22, 2, line);      // goal box front
        mb.flatRect(V3(-1024, float(s) * 4608, 0), 22, 512, 2, line);   // sides
        mb.flatRect(V3(1024, float(s) * 4608, 0), 22, 512, 2, line);
    }
}

void discY(MeshBuilder& mb, float y, float radius, int segs, const Col& c, int sign) {
    V3 n(0, float(sign), 0);
    uint32_t center = mb.addVert(V3(0, y, 0), n, 0.5f, 0.5f, c);
    std::vector<uint32_t> ring;
    for (int i = 0; i <= segs; i++) {
        float a = float(i) / segs * 2 * (float)M_PI;
        ring.push_back(mb.addVert(V3(std::cos(a) * radius, y, std::sin(a) * radius), n, 0, 0, c));
    }
    for (int i = 0; i < segs; i++) mb.triOriented(center, ring[i], ring[i + 1], n);
}

void annulusY(MeshBuilder& mb, float y, float r0, float r1, int segs, const Col& c, int sign) {
    V3 n(0, float(sign), 0);
    std::vector<uint32_t> a, b;
    for (int i = 0; i <= segs; i++) {
        float ang = float(i) / segs * 2 * (float)M_PI;
        V3 dir(std::cos(ang), 0, std::sin(ang));
        a.push_back(mb.addVert(V3(0, y, 0) + dir * r0, n, 0, 0, c));
        b.push_back(mb.addVert(V3(0, y, 0) + dir * r1, n, 1, 0, c));
    }
    for (int i = 0; i < segs; i++) {
        mb.triOriented(a[i], b[i], b[i + 1], n);
        mb.triOriented(a[i], b[i + 1], a[i + 1], n);
    }
}

void buildWheel(MeshBuilder& mb, float r, float hw) {
    const Col tire(0.065f, 0.065f, 0.075f, 1);
    const Col rim(0.60f, 0.62f, 0.65f, 1);
    const Col hub(0.17f, 0.18f, 0.20f, 1);
    mb.cylinder(V3(0, -hw, 0), V3(0, hw, 0), r, r, 22, tire, false, false);
    for (int s : {-1, 1}) {
        float k = float(s);
        annulusY(mb, k * hw, r * 0.63f, r, 22, tire, s);               // sidewall
        annulusY(mb, k * (hw - 0.4f), r * 0.63f, r * 0.24f, 22, rim, s); // rim face
        discY(mb, k * (hw - 1.1f), r * 0.24f, 16, hub, s);              // recessed hub
    }
}

// ---------------------------------------------------------------- car body
void buildCarBody(MeshBuilder& mb) {
    const Col body(0.78f, 0.79f, 0.81f, 1);
    const Col under(0.16f, 0.16f, 0.17f, 1);
    const Col well(0.085f, 0.085f, 0.095f, 1);
    const Col dark(0.13f, 0.13f, 0.14f, 1);
    const Col glass(0.045f, 0.055f, 0.075f, 1);
    const Col lightC(0.95f, 0.93f, 0.78f, 1);
    const Col tailC(0.72f, 0.07f, 0.07f, 1);

    auto rearArch = [](float x) {
        float t = clampf((x + 33.75f) / 17.0f, -1, 1);
        return 2.0f + 14.0f * std::sqrt(std::max(0.0f, 1 - t * t));
    };
    auto frontArch = [](float x) {
        float t = clampf((x - 51.25f) / 15.0f, -1, 1);
        return 2.0f + 12.0f * std::sqrt(std::max(0.0f, 1 - t * t));
    };

    // ---- main body profile (chains stored rear -> front, x ascending)
    std::vector<std::pair<float, float>> bottom, top;
    std::vector<Col> bc, tc;

    bottom.push_back({-46, rearArch(-46)});  bc.push_back(well);
    for (float x = -44; x <= -18.5f; x += 2) {
        bottom.push_back({x, rearArch(x)});
        bc.push_back(well);
    }
    bottom.push_back({-16.75f, 2}); bc.push_back(well);
    bottom.push_back({0, 2});       bc.push_back(under);
    bottom.push_back({36.25f, 2});  bc.push_back(well);
    for (float x = 38; x <= 66.5f; x += 2) {
        bottom.push_back({x, frontArch(x)});
        bc.push_back(well);
    }
    bottom.push_back({74, 2});      bc.push_back(dark);   // front bumper (also front face)

    top.push_back({-46, 32});  tc.push_back(body);  // rear lip spoiler
    top.push_back({-44, 32});  tc.push_back(body);
    top.push_back({-42, 30.6f}); tc.push_back(body);
    top.push_back({0, 30});    tc.push_back(body);
    top.push_back({48, 30});   tc.push_back(body);
    top.push_back({60, 29.5f}); tc.push_back(body);
    top.push_back({68, 27.5f}); tc.push_back(body);
    top.push_back({74, 20});   tc.push_back(body);

    mb.prismXZ(bottom, top, -43.35f, 43.35f, body, body, &bc, &tc);

    // ---- cabin / greenhouse
    std::vector<std::pair<float, float>> cb = {{-42, 30.5f}, {0, 30.5f}, {44, 30.5f}};
    std::vector<std::pair<float, float>> ct = {{-42, 34}, {-33, 41}, {34, 41}};
    std::vector<Col> cbc(3, body), ctc(3, body);
    mb.prismXZ(cb, ct, -37, 37, body, body, &cbc, &ctc);

    // ---- side windows (on the cabin caps, slightly proud)
    for (int s : {-1, 1}) {
        float y = s * 37.35f;
        V3 n(0, s, 0);
        mb.quadOriented(V3(-31, y, 32.6f), V3(33, y, 32.6f),
                        V3(33, y, 39.6f), V3(-31, y, 39.6f), n, glass);
    }

    // ---- windshield (on the raked front face)
    {
        // profile edge: (44,30.5) -> (34,41); outward = normalize(dz, -dx) in (x,z)
        auto pAt = [](float t) { return V3(44 - 10 * t, 0, 30.5f + 10.5f * t); };
        V3 n = V3(10.5f, 0, 10.0f).norm();
        V3 p0 = pAt(0.13f) + n * 0.45f;
        V3 p1 = pAt(0.87f) + n * 0.45f;
        mb.quadOriented(V3(p0.x, -33, p0.z), V3(p0.x, 33, p0.z),
                        V3(p1.x, 33, p1.z), V3(p1.x, -33, p1.z), n, glass);
    }
    // ---- rear hatch glass
    {
        auto pAt = [](float t) { return V3(-42 + 9 * t, 0, 34 + 7 * t); };
        V3 n = V3(-7.0f, 0, 9.0f).norm();
        V3 p0 = pAt(0.16f) + n * 0.45f;
        V3 p1 = pAt(0.86f) + n * 0.45f;
        mb.quadOriented(V3(p0.x, -32, p0.z), V3(p0.x, 32, p0.z),
                        V3(p1.x, 32, p1.z), V3(p1.x, -32, p1.z), n, glass);
    }

    // ---- headlights / grille / taillights
    for (int s : {-1, 1}) {
        float y0 = s * 24.0f, y1 = s * 40.0f;
        mb.quadOriented(V3(74.4f, y0, 11), V3(74.4f, y1, 11),
                        V3(74.4f, y1, 17.5f), V3(74.4f, y0, 17.5f), {1, 0, 0}, lightC);
        mb.quadOriented(V3(-46.4f, s * 26.0f, 23), V3(-46.4f, s * 41.0f, 23),
                        V3(-46.4f, s * 41.0f, 30), V3(-46.4f, s * 26.0f, 30), {-1, 0, 0}, tailC);
    }
    mb.quadOriented(V3(74.3f, -19, 4), V3(74.3f, 19, 4),
                    V3(74.3f, 19, 9.5f), V3(74.3f, -19, 9.5f), {1, 0, 0}, Col(0.09f, 0.09f, 0.10f, 1));

    // ---- belt-line trim (thin dark strip along the body sides)
    for (int s : {-1, 1}) {
        float y = s * 43.5f;
        mb.quadOriented(V3(-46, y, 29.2f), V3(74, y, 29.2f),
                        V3(74, y, 30.4f), V3(-46, y, 30.4f), V3(0, s, 0), dark);
    }
}

// ---------------------------------------------------------------- ball tex
void buildBallTexture(Texture2D& tex) {
    constexpr int W = 1024, H = 512;
    Image img;
    img.init(W, H);

    const float t = (1.0f + std::sqrt(5.0f)) / 2.0f;
    std::vector<V3> centers = {
        {0, 1, t}, {0, -1, t}, {0, 1, -t}, {0, -1, -t},
        {1, t, 0}, {-1, t, 0}, {1, -t, 0}, {-1, -t, 0},
        {t, 0, 1}, {-t, 0, 1}, {t, 0, -1}, {-t, 0, -1},
    };
    for (auto& c : centers) c = c.norm();

    auto smoothstep01 = [](float e0, float e1, float x) {
        float u = clampf((x - e0) / (e1 - e0), 0, 1);
        return u * u * (3 - 2 * u);
    };

    for (int y = 0; y < H; y++) {
        float v = (y + 0.5f) / H;
        float phi = v * (float)M_PI;
        for (int x = 0; x < W; x++) {
            float u = (x + 0.5f) / W;
            float th = u * 2 * (float)M_PI;
            V3 d(std::sin(phi) * std::cos(th), std::sin(phi) * std::sin(th), std::cos(phi));

            float d1 = -2, d2 = -2;
            for (const V3& c : centers) {
                float dv = d.dot(c);
                if (dv > d1) { d2 = d1; d1 = dv; }
                else if (dv > d2) { d2 = dv; }
            }
            // base panel color (white), pentagons dark, seams mid-gray
            float pent = smoothstep01(0.795f, 0.815f, d1);
            float seam = smoothstep01(0.016f, 0.004f, d1 - d2);

            float r = 0.93f, g = 0.94f, b = 0.95f;
            r = r * (1 - pent) + 0.26f * pent;
            g = g * (1 - pent) + 0.28f * pent;
            b = b * (1 - pent) + 0.31f * pent;
            r = r * (1 - seam) + 0.42f * seam;
            g = g * (1 - seam) + 0.44f * seam;
            b = b * (1 - seam) + 0.47f * seam;

            img.set(x, y, uint8_t(r * 255), uint8_t(g * 255), uint8_t(b * 255), 255);
        }
    }
    tex.create(W, H, img.px.data(), /*repeat=*/true, /*mips=*/true);
}

// ---------------------------------------------------------------- Fennec STL
// Model from Thingiverse #4195502 (CC BY-NC-SA, see assets/models/LICENSE.txt):
// binary STL, +Y forward, +X width (body centered on FGX), Z up, ground z=0.
bool readStl(const std::string& path, std::vector<float>& out) {
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    std::fseek(f, 0, SEEK_END);
    long sz = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    if (sz < 84) { std::fclose(f); return false; }
    uint8_t hdr[84];
    if (std::fread(hdr, 1, 84, f) != 84) { std::fclose(f); return false; }
    uint32_t n = 0;
    std::memcpy(&n, hdr + 80, 4);
    if (n == 0 || n > 2000000 || long(84) + long(n) * 50 != sz) {
        std::fclose(f);
        return false;   // not a binary STL (ascii unsupported: falls back)
    }
    out.resize(size_t(n) * 9);
    for (uint32_t i = 0; i < n; i++) {
        uint8_t rec[50];
        if (std::fread(rec, 1, 50, f) != 50) { std::fclose(f); return false; }
        float v[12];
        std::memcpy(v, rec, 48);   // skip the stored normal, recompute below
        for (int k = 0; k < 3; k++)
            for (int a = 0; a < 3; a++)
                out[size_t(i) * 9 + size_t(k) * 3 + a] = v[3 + k * 3 + a];
    }
    std::fclose(f);
    return true;
}

// Transform + paint raw STL triangles into the mesh (flat normals per face).
template <class Xf, class ColFn>
void appendStl(const std::vector<float>& tris, MeshBuilder& mb,
               const Xf& xf, const ColFn& colOf) {
    for (size_t i = 0; i + 8 < tris.size(); i += 9) {
        V3 a = xf(V3(tris[i], tris[i + 1], tris[i + 2]));
        V3 b = xf(V3(tris[i + 3], tris[i + 4], tris[i + 5]));
        V3 c = xf(V3(tris[i + 6], tris[i + 7], tris[i + 8]));
        V3 n = (b - a).cross(c - a);
        if (n.len() < 1e-12f) continue;
        n = n.norm();
        Col col = colOf((a + b + c) * (1.0f / 3.0f), n);
        uint32_t i0 = mb.addVert(a, n, 0, 0, col);
        uint32_t i1 = mb.addVert(b, n, 0, 0, col);
        uint32_t i2 = mb.addVert(c, n, 0, 0, col);
        mb.triOriented(i0, i1, i2, n);
    }
}

constexpr float FK = 1.082f;      // model units -> car units (110.9 -> 120)
constexpr float FGX = -12.01f;    // model body center (x, width axis)
constexpr float FGY = -36.4f;     // model rear-most point (y, length axis)
constexpr float CAR_GROUND = -17.0f;  // wheel contact plane in car space

// Model -> car: forward +Y -> +X (yaw -90 deg), width X -> -Y, ground to -17.
V3 fennecBodyXf(const V3& p) {
    return V3((p.y - FGY) * FK - 46.0f, -(p.x - FGX) * FK, p.z * FK + CAR_GROUND);
}

// Paint: body gray; glass on raked screens and the upper greenhouse.
Col fennecBodyCol(const V3& p, const V3& n) {
    const Col body(0.78f, 0.79f, 0.81f, 1);
    const Col glass(0.05f, 0.065f, 0.09f, 1);
    const bool raked = std::fabs(n.z) > 0.25f && std::fabs(n.z) < 0.94f &&
                       std::fabs(n.y) > 0.25f && p.z > 15.0f;
    const bool greenhouse = std::fabs(n.y) > 0.85f && n.z < 0.45f &&
                            p.z > 17.5f && p.x > -36.0f && p.x < 18.0f;
    return (raked || greenhouse) ? glass : body;
}

bool buildFennecBody(MeshBuilder& mb, const std::string& modelDir) {
    std::vector<float> tris;
    if (!readStl(modelDir + "/fennec.stl", tris)) return false;
    appendStl(tris, mb, fennecBodyXf, fennecBodyCol);
    return !mb.verts.empty();
}

// Wheels: separate STLs lying flat (axle = Z, layout center (cx,cy)); stood up
// so the axle runs along Y, painted in radial bands (tire/rim/hub).
bool buildFennecWheels(MeshBuilder& front, MeshBuilder& back,
                       WheelMount mounts[4], const std::string& modelDir) {
    std::vector<float> wf, wr;
    if (!readStl(modelDir + "/front_wheel.stl", wf)) return false;
    if (!readStl(modelDir + "/rear_wheel.stl", wr)) return false;

    const float rF = 19.5f / 2 * FK, rR = 22.0f / 2 * FK;
    const float lat = 21.5f;
    // Arch centers measured from the body STL's wheel openings.
    const V3 mF(49.6f, 0, CAR_GROUND + rF);
    const V3 mR(-23.9f, 0, CAR_GROUND + rR);
    const float cFx = -92.99f, cFy = 42.35f;   // print-layout centers
    const float cRx = -84.9f, cRy = -3.99f;

    mounts[0] = {V3(mF.x,  lat, mF.z), rF, true};
    mounts[1] = {V3(mF.x, -lat, mF.z), rF, true};
    mounts[2] = {V3(mR.x,  lat, mR.z), rR, false};
    mounts[3] = {V3(mR.x, -lat, mR.z), rR, false};

    // The mesh must stay CENTERED at the origin: the renderer translates it to
    // the mount and spins it about its own axle. Baking the mount in here would
    // double the offset and make the wheels orbit the car instead of spinning.
    auto wheelXf = [](const V3& p, float cx, float cy, int side) {
        float dx = p.x - cx, dy = p.y - cy, dz = p.z - 5.095f;
        float x = dx, y = -dz, z = dy;      // stand up: axle Z -> Y
        if (side < 0) { x = -x; y = -y; }   // far side: 180 deg about Z
        return V3(x * FK, y * FK, z * FK);
    };
    auto wheelCol = [](float r) {
        return [r](const V3& p, const V3&) {
            // centered mesh: radial distance from the axle (Y axis)
            float rad = std::sqrt(p.x * p.x + p.z * p.z);
            if (rad > 0.85f * r) return Col(0.065f, 0.065f, 0.075f, 1);  // tire
            if (rad > 0.30f * r) return Col(0.58f, 0.60f, 0.63f, 1);     // rim
            return Col(0.17f, 0.18f, 0.20f, 1);                          // hub
        };
    };

    for (int s = 1; s >= -1; s -= 2) {
        appendStl(wf, front,
                  [&](const V3& p) { return wheelXf(p, cFx, cFy, s); },
                  wheelCol(rF));
        appendStl(wr, back,
                  [&](const V3& p) { return wheelXf(p, cRx, cRy, s); },
                  wheelCol(rR));
    }
    return !front.verts.empty() && !back.verts.empty();
}

}  // namespace

bool GameAssets::build(const std::string& meshDir, const std::string& modelDir) {
    {
        MeshBuilder mb;
        buildArenaFloor(mb);
        arenaFloor.upload(mb.verts, mb.idx);
    }
    {
        MeshBuilder mb;
        buildArenaShell(mb, meshDir);
        arenaShell.upload(mb.verts, mb.idx);
    }
    {
        MeshBuilder mb;
        buildMarkings(mb);
        markings.upload(mb.verts, mb.idx);
    }
    {
        MeshBuilder mb;
        // Thingiverse Fennec model; falls back to the procedural body if the
        // files are missing (keeps fresh/partial checkouts playable).
        if (!buildFennecBody(mb, modelDir)) buildCarBody(mb);
        carBody.upload(mb.verts, mb.idx);
    }
    {
        MeshBuilder mbF, mbB;
        if (!buildFennecWheels(mbF, mbB, wheelMounts, modelDir)) {
            buildWheel(mbF, 12.5f, 6.0f);
            buildWheel(mbB, 15.0f, 6.5f);
            wheelMounts[0] = {V3(51.25f, 25.90f, -4.5f), 12.5f, true};
            wheelMounts[1] = {V3(51.25f, -25.90f, -4.5f), 12.5f, true};
            wheelMounts[2] = {V3(-33.75f, 29.50f, -2.0f), 15.0f, false};
            wheelMounts[3] = {V3(-33.75f, -29.50f, -2.0f), 15.0f, false};
        }
        wheelFront.upload(mbF.verts, mbF.idx);
        wheelBack.upload(mbB.verts, mbB.idx);
    }
    {
        MeshBuilder mb;
        mb.sphere(V3(0, 0, 0), 91.25f, 56, 28, Col(1, 1, 1, 1), /*uvForTexture=*/true);
        ball.upload(mb.verts, mb.idx);
        buildBallTexture(ballTex);
    }
    {
        MeshBuilder mb;
        Col in(1, 1, 1, 0.75f), out(1, 1, 1, 0.75f);
        Col in2(1, 1, 1, 0.0f), out2(1, 1, 1, 0.0f);
        // soft band centered on radius 91 (uses uv-less ring: alpha via vertex color)
        mb.ringZ(V3(0, 0, 0), 74, 91, 64, in2, in);
        mb.ringZ(V3(0, 0, 0), 91, 108, 64, out, out2);
        indicator.upload(mb.verts, mb.idx);
    }
    {
        MeshBuilder mb;
        V3 n(0, 0, 1);
        uint32_t center = mb.addVert(V3(0, 0, 0), n, 0.5f, 0.5f, Col(0, 0, 0, 0.58f));
        std::vector<uint32_t> ring;
        for (int i = 0; i <= 40; i++) {
            float a = float(i) / 40 * 2 * (float)M_PI;
            ring.push_back(mb.addVert(V3(std::cos(a), std::sin(a), 0), n, 0, 0, Col(0, 0, 0, 0)));
        }
        for (int i = 0; i < 40; i++) {
            mb.triOriented(center, ring[i], ring[i + 1], n);
        }
        shadowDisc.upload(mb.verts, mb.idx);
    }
    return arenaFloor.valid() && arenaShell.valid() && carBody.valid() && ball.valid();
}

void GameAssets::destroy() {
    arenaFloor.destroy();
    arenaShell.destroy();
    markings.destroy();
    carBody.destroy();
    wheelFront.destroy();
    wheelBack.destroy();
    ball.destroy();
    indicator.destroy();
    shadowDisc.destroy();
    ballTex.destroy();
}
