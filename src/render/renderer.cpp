#include "renderer.h"

#include "mesh_builder.h"

#include <cmath>
#include <cstdio>

namespace {

const char* LIT_VS = R"(#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNrm;
layout(location=2) in vec2 aUV;
layout(location=3) in vec4 aCol;
uniform mat4 uModel, uView, uProj;
out vec3 vWorld;
out vec3 vNrm;
out vec2 vUV;
out vec4 vCol;
void main() {
    vec4 w = uModel * vec4(aPos, 1.0);
    vWorld = w.xyz;
    vNrm = mat3(uModel) * aNrm;
    vUV = aUV;
    vCol = aCol;
    gl_Position = uProj * uView * w;
}
)";

const char* LIT_FS = R"(#version 330 core
in vec3 vWorld;
in vec3 vNrm;
in vec2 vUV;
in vec4 vCol;
uniform int uMode;      // 0 = arena (world grid), 1 = textured, 2 = plain
uniform float uAlpha;   // global alpha (see-through walls)
uniform sampler2D uTex;
uniform vec3 uLightDir; // direction the light travels
out vec4 frag;
void main() {
    vec3 N = normalize(vNrm);
    if (!gl_FrontFacing) N = -N;
    vec3 albedo = vCol.rgb;
    if (uMode == 1) albedo *= texture(uTex, vUV).rgb;
    if (uMode == 0) {
        vec2 g = abs(N.z) > 0.7 ? vWorld.xy : (abs(N.x) > 0.7 ? vWorld.zy : vWorld.xz);
        vec2 f1 = abs(fract(g / 1024.0) - 0.5) * 1024.0;
        float line1 = 1.0 - smoothstep(2.0, 7.0, min(f1.x, f1.y));
        albedo = mix(albedo, albedo * 1.85 + 0.03, line1 * 0.62);
        vec2 f2 = abs(fract(g / 256.0) - 0.5) * 256.0;
        float line2 = 1.0 - smoothstep(1.0, 3.0, min(f2.x, f2.y));
        albedo = mix(albedo, albedo * 1.35, line2 * 0.22);
    }
    float ndl = max(dot(N, -uLightDir), 0.0);
    vec3 hemi = mix(vec3(0.70, 0.74, 0.84), vec3(1.05), N.z * 0.5 + 0.5);
    vec3 light = vec3(0.36) * hemi + vec3(1.0, 0.97, 0.90) * ndl * 0.95;
    frag = vec4(pow(albedo * light, vec3(1.0/2.2)), vCol.a * uAlpha);
}
)";

const char* UNLIT_VS = R"(#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNrm;
layout(location=2) in vec2 aUV;
layout(location=3) in vec4 aCol;
uniform mat4 uModel, uView, uProj;
out vec2 vUV;
out vec4 vCol;
void main() {
    vec4 w = uModel * vec4(aPos, 1.0);
    vUV = aUV;
    vCol = aCol;
    gl_Position = uProj * uView * w;
}
)";

const char* UNLIT_FS = R"(#version 330 core
in vec2 vUV;
in vec4 vCol;
uniform int uTextured;   // 0 = vertex color, 1 = multiply texture, 2 = radial disc
uniform sampler2D uTex;
uniform float uAlphaMul;
out vec4 frag;
void main() {
    vec4 c = vCol;
    if (uTextured == 1) c *= texture(uTex, vUV);
    if (uTextured == 2) {
        float r = length(vUV - 0.5) * 2.0;
        float a = 1.0 - smoothstep(0.30, 1.0, r);
        c.a *= a * a;
    }
    c.rgb = pow(c.rgb, vec3(1.0/2.2));
    c.a *= uAlphaMul;
    frag = c;
}
)";

const V3 SKY(0.31f, 0.35f, 0.45f);
const V3 LIGHT_DIR(0.42f, -0.36f, -0.83f);  // light travels downward

// One outline segment a->b as two crossed quads of width t. From any view
// angle at least one of the two faces you, so the outline never vanishes
// edge-on - no line-width/geometry-shader support needed.
void addEdge(MeshBuilder& mb, const V3& a, const V3& b, float t, const Col& c) {
    const V3 d = b - a;
    const float len = d.len();
    if (len < 1e-4f) return;
    const V3 axis = d / len;
    const V3 ref = std::fabs(axis.z) < 0.9f ? V3(0, 0, 1) : V3(1, 0, 0);
    const V3 u = axis.cross(ref).norm();
    const V3 v = axis.cross(u).norm();
    const float h = t * 0.5f;
    mb.quad(a + u * h, b + u * h, b - u * h, a - u * h, c);
    mb.quad(a + v * h, b + v * h, b - v * h, a - v * h, c);
}

}  // namespace

bool Renderer::init(const std::string& meshDir, const std::string& modelDir) {
    bool ok = true;
    ok &= lit_.compile(LIT_VS, LIT_FS, "lit");
    ok &= unlit_.compile(UNLIT_VS, UNLIT_FS, "unlit");
    ok &= assets_.build(meshDir, modelDir);
    if (!ok) std::fprintf(stderr, "[renderer] init failed\n");
    return ok;
}

void Renderer::shutdown() {
    assets_.destroy();
    particleMesh_.destroy();
    hitboxCar_.destroy();
    hitboxBall_.destroy();
    lit_.destroy();
    unlit_.destroy();
}

void Renderer::setHitboxDims(const V3& size, const V3& offset, float ballRadius) {
    hitboxCar_.destroy();
    hitboxBall_.destroy();
    if (size.x <= 0 || size.y <= 0 || size.z <= 0 || ballRadius <= 0) return;

    const Col carCol(1.0f, 0.58f, 0.10f, 0.95f);   // your car, RL orange
    const Col ballCol(0.35f, 1.0f, 0.45f, 0.95f);  // ball green (RL overlays)
    const float t = 2.2f;                          // line thickness (uu)

    // ---- car: the 12 edges of the oriented physics box (config offset
    // baked into car-local space, so the draw transform is just the frame).
    MeshBuilder mb;
    const V3 mn = offset - size * 0.5f;
    const V3 mx = offset + size * 0.5f;
    V3 v[8];
    for (int i = 0; i < 8; i++)
        v[i] = V3((i & 1) ? mx.x : mn.x, (i & 2) ? mx.y : mn.y, (i & 4) ? mx.z : mn.z);
    static const int EDGES[12][2] = {
        {0, 1}, {2, 3}, {4, 5}, {6, 7},   // along X
        {0, 2}, {1, 3}, {4, 6}, {5, 7},   // along Y
        {0, 4}, {1, 5}, {2, 6}, {3, 7},   // along Z
    };
    for (const auto& e : EDGES) addEdge(mb, v[e[0]], v[e[1]], t, carCol);
    hitboxCar_.upload(mb.verts, mb.idx);

    // ---- ball: a wireframe globe at the collision radius, outset a touch
    // so the rings never z-fight the ball's skin.
    mb.clear();
    const float R = ballRadius + 2.0f;
    auto circle = [&](const V3& c0, const V3& u, const V3& w, float r, int segs) {
        for (int i = 0; i < segs; i++) {
            const float a0 = float(i) / segs * 2.0f * (float)M_PI;
            const float a1 = float(i + 1) / segs * 2.0f * (float)M_PI;
            addEdge(mb, c0 + (u * std::cos(a0) + w * std::sin(a0)) * r,
                        c0 + (u * std::cos(a1) + w * std::sin(a1)) * r, t, ballCol);
        }
    };
    circle(V3(0, 0, 0), V3(1, 0, 0), V3(0, 1, 0), R, 28);          // equator
    const float q = (float)M_PI * 0.25f;
    const float rl = R * std::cos(q), rz = R * std::sin(q);
    circle(V3(0, 0, rz), V3(1, 0, 0), V3(0, 1, 0), rl, 28);         // latitudes
    circle(V3(0, 0, -rz), V3(1, 0, 0), V3(0, 1, 0), rl, 28);
    for (int m = 0; m < 4; m++) {                                   // meridians
        const float a = m * (float)M_PI * 0.25f;
        circle(V3(0, 0, 0), V3(std::cos(a), std::sin(a), 0), V3(0, 0, 1), R, 28);
    }
    hitboxBall_.upload(mb.verts, mb.idx);
}

void Renderer::drawLit(const GpuMesh& mesh, const M4& model, int mode, float alpha) {
    if (!mesh.valid()) return;
    lit_.setMat4("uModel", model.m);
    lit_.setInt("uMode", mode);
    lit_.setFloat("uAlpha", alpha);   // GL defaults to 0 - must set every draw
    mesh.draw();
}

void Renderer::drawUnlit(const GpuMesh& mesh, const M4& model, float alphaMul,
                         bool additive, bool textured) {
    if (!mesh.valid()) return;
    glBlendFunc(GL_SRC_ALPHA, additive ? GL_ONE : GL_ONE_MINUS_SRC_ALPHA);
    unlit_.setMat4("uModel", model.m);
    unlit_.setFloat("uAlphaMul", alphaMul);
    unlit_.setInt("uTextured", textured ? 1 : 0);
    mesh.draw();
}

void Renderer::render(const RLCamera& cam, const RenderParams& p, const ParticleSystem& ps) {
    w_ = p.width;
    h_ = p.height;
    const SimSnapshot& s = *p.snap;

    glViewport(0, 0, w_, h_);
    glDisable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
    glClearColor(SKY.x, SKY.y, SKY.z, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    float aspect = h_ > 0 ? float(w_) / float(h_) : 1.0f;
    lastFovX = p.fovX;
    view_ = M4::viewRL(cam.eye, cam.target);
    proj_ = M4::perspectiveH(p.fovX, aspect, 10.0f, 30000.0f);

    lit_.use();
    lit_.setMat4("uView", view_.m);
    lit_.setMat4("uProj", proj_.m);
    lit_.setVec3("uLightDir", LIGHT_DIR.x, LIGHT_DIR.y, LIGHT_DIR.z);
    lit_.setInt("uTex", 0);

    // ---------- walls/ceiling are always see-through (like Rocket League);
    // only the floor stays fully solid. At opacity 1 the shell is just part
    // of the opaque pass instead.
    const bool seeThrough = p.wallOpacity < 0.999f;

    // ---------- opaque
    drawLit(assets_.arenaFloor, M4::identity(), 0);   // floor always solid
    if (!seeThrough) drawLit(assets_.arenaShell, M4::identity(), 0);

    // ball
    {
        M4 model = M4::fromFrame(s.ballF, s.ballR, s.ballU, s.ballPos);
        assets_.ballTex.bind(0);
        drawLit(assets_.ball, model, 1);
    }

    // car body
    {
        M4 model = M4::fromFrame(s.carF, s.carR, s.carU, s.carPos);
        drawLit(assets_.carBody, model, 2);

        // wheels: front pair steers, all spin with ground speed
        float signedSpeed = s.carVel.dot(s.carF);
        float avgR = 0;
        for (const WheelMount& w : assets_.wheelMounts) avgR += w.r;
        avgR /= 4.0f;
        if (s.onGround) wheelSpin_ += (signedSpeed * p.dt) / avgR;
        else wheelSpin_ *= std::exp(-1.2f * p.dt);
        float steerAng = p.steerInput * 0.5f;

        for (const WheelMount& w : assets_.wheelMounts) {
            M4 local = M4::translate(w.pos);
            if (w.front) local = local * M4::rotateAxis(V3(0, 0, 1), steerAng);
            local = local * M4::rotateAxis(V3(0, 1, 0), wheelSpin_ / (w.r / avgR));
            M4 model2 = model * local;
            drawLit(w.front ? assets_.wheelFront : assets_.wheelBack, model2, 2);
        }
    }

    // ---------- transparent
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);

    unlit_.use();
    unlit_.setMat4("uView", view_.m);
    unlit_.setMat4("uProj", proj_.m);
    unlit_.setInt("uTex", 0);

    drawUnlit(assets_.markings, M4::identity(), 1.0f, false);

    if (p.showShadows) {
        float carA = clampf(1.0f - s.carPos.z / 500.0f, 0, 1);
        if (carA > 0.01f) {
            M4 m = M4::translate(V3(s.carPos.x, s.carPos.y, 3.0f)) *
                   M4::scale(V3(72, 66, 1));
            drawUnlit(assets_.shadowDisc, m, carA, false);
        }
        float ballA = clampf(1.0f - s.ballPos.z / 700.0f, 0, 1);
        if (ballA > 0.01f) {
            M4 m = M4::translate(V3(s.ballPos.x, s.ballPos.y, 3.5f)) *
                   M4::scale(V3(95, 95, 1));
            drawUnlit(assets_.shadowDisc, m, ballA, false);
        }
    }

    if (p.showBallRing) {
        M4 m = M4::translate(V3(s.ballPos.x, s.ballPos.y, 4.0f));
        drawUnlit(assets_.indicator, m, 0.85f, false);
    }

    // ---------- flip reset indicator (Settings > Camera): a glowing hollow
    // disc pinned under the car's underside while a *ball* reset is held. It
    // rides in the car's own frame (so it stays under the wheels when the car
    // is tilted or inverted), additively blended so it glows, pops in on the
    // tick the reset lands and settles into a steady glow until the flip is
    // used or we land. Wall/ceiling resets never get here - see Sim::stepOnce.
    if (p.showFlipReset && s.flipReset) {
        const float t = clampf(s.flipResetAge / 0.35f, 0.0f, 1.0f);
        const float scale = lerpf(1.5f, 1.0f, t);
        const float alpha = lerpf(1.9f, 1.0f, t);
        // Local -Z points at the wheels, which reach 17uu below the car
        // origin: -26 parks the 12uu slab's top face 3uu clear of them, so it
        // reads as the car's glowing underside rather than something the car
        // sits on.
        M4 model = M4::fromFrame(s.carF, s.carR, s.carU, s.carPos) *
                   M4::translate(V3(0, 0, -26.0f)) *
                   M4::scale(V3(scale, scale, 1));
        drawUnlit(assets_.flipDisc, model, alpha, /*additive=*/true);
    }

    // ---------- physics hitbox outlines (Settings > Graphics).
    // Drawn in the transparent pass, depth-tested but offset slightly
    // toward the camera so box edges grazing the car body don't z-fight.
    if (p.showHitboxes) {
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(-1.0f, -1.0f);
        if (hitboxCar_.valid()) {
            M4 model = M4::fromFrame(s.carF, s.carR, s.carU, s.carPos);
            drawUnlit(hitboxCar_, model, 1.0f, false);
        }
        if (hitboxBall_.valid())
            drawUnlit(hitboxBall_, M4::translate(s.ballPos), 1.0f, false);
        glDisable(GL_POLYGON_OFFSET_FILL);
    }

    // ---------- particles (additive billboards)
    if (ps.count() > 0) {
        V3 d = (cam.target - cam.eye).norm();
        V3 right = headingRight(d);
        float rl = right.len();
        if (rl > 1e-5f) right = right / rl; else right = V3(1, 0, 0);
        V3 back = d * -1.0f;
        V3 up = right.cross(back);

        static std::vector<Vertex> pv;
        static std::vector<uint32_t> pi;
        pv.clear();
        pi.clear();
        ps.buildQuads(right, up, pv, pi);
        if (!pv.empty()) {
            particleMesh_.upload(pv, pi, /*dynamic=*/true);
            unlit_.setMat4("uModel", M4::identity().m);
            unlit_.setFloat("uAlphaMul", 1.0f);
            unlit_.setInt("uTextured", 2);  // analytic radial disc
            glBlendFunc(GL_SRC_ALPHA, GL_ONE);  // additive glow
            particleMesh_.draw();
        }
    }

    // ---------- see-through walls, drawn last: blended over whatever is
    // behind them (car, ball, floor, sky), depth-tested but no depth writes
    if (seeThrough) {
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        lit_.use();   // drawLit's uniforms must land on the lit program
        drawLit(assets_.arenaShell, M4::identity(), 0, p.wallOpacity);
    }

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

bool Renderer::project(const V3& world, float& sx, float& sy) const {
    const float* v = view_.m;
    const float* m = proj_.m;
    // view * point (w = 1)
    float vx = v[0] * world.x + v[4] * world.y + v[8] * world.z + v[12];
    float vy = v[1] * world.x + v[5] * world.y + v[9] * world.z + v[13];
    float vz = v[2] * world.x + v[6] * world.y + v[10] * world.z + v[14];
    // proj * point
    float cx = m[0] * vx + m[4] * vy + m[8] * vz + m[12];
    float cy = m[1] * vx + m[5] * vy + m[9] * vz + m[13];
    float cw = m[3] * vx + m[7] * vy + m[11] * vz + m[15];
    if (cw <= 1e-4f) return false;
    float ndcx = cx / cw, ndcy = cy / cw;
    sx = (ndcx * 0.5f + 0.5f) * w_;
    sy = (1.0f - (ndcy * 0.5f + 0.5f)) * h_;
    return true;
}
