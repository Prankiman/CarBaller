#include "particles.h"

#include <cmath>

void ParticleSystem::spawnBoost(const V3& pos, const V3& dirBack, int count) {
    V3 up(0, 0, 1);
    V3 side = dirBack.cross(up);
    if (side.len() < 1e-3f) side = V3(1, 0, 0);
    side = side.norm();

    for (int i = 0; i < count && ps_.size() < MAX_P; i++) {
        Particle p;
        p.pos = pos + side * frand(-4, 4) + up * frand(-4, 4) + dirBack * frand(0, 5);
        p.vel = dirBack * frand(320, 540) + side * frand(-35, 35) + up * frand(-35, 35);
        p.life = p.maxLife = frand(0.14f, 0.30f);
        p.size0 = frand(4, 6.5f);
        p.size1 = frand(10, 17);
        p.drag = 5.5f;
        p.grav = 60;
        ps_.push_back(p);
    }
}

void ParticleSystem::spawnImpact(const V3& pos, float strength, int quality) {
    int n = 6 + quality * 7;
    float speedScale = clampf(strength / 2500.0f, 0.25f, 1.6f);
    for (int i = 0; i < n && ps_.size() < MAX_P; i++) {
        Particle p;
        // random direction, biased upward and outward
        float th = frand(0, 2 * (float)M_PI);
        float z = frand(-0.35f, 1.0f);
        V3 dir(std::cos(th), std::sin(th), z);
        dir = dir.norm();
        p.pos = pos + dir * frand(2, 12);
        p.vel = dir * frand(180, 620) * speedScale;
        p.life = p.maxLife = frand(0.25f, 0.55f);
        p.size0 = frand(3.5f, 6.5f);
        p.size1 = frand(1.5f, 3.5f);
        p.drag = 2.5f;
        p.grav = -2200;
        ps_.push_back(p);
    }
}

void ParticleSystem::update(float dt) {
    for (size_t i = 0; i < ps_.size();) {
        Particle& p = ps_[i];
        p.life -= dt;
        if (p.life <= 0) {
            ps_[i] = ps_.back();
            ps_.pop_back();
            continue;
        }
        p.vel += V3(0, 0, p.grav) * dt;
        if (p.drag > 0) p.vel *= std::exp(-p.drag * dt);
        p.pos += p.vel * dt;
        i++;
    }
}

void ParticleSystem::buildQuads(const V3& right, const V3& up,
                                std::vector<Vertex>& outVerts,
                                std::vector<uint32_t>& outIdx) const {
    outVerts.reserve(outVerts.size() + ps_.size() * 4);
    outIdx.reserve(outIdx.size() + ps_.size() * 6);

    for (const Particle& p : ps_) {
        float t = clampf(p.life / p.maxLife, 0, 1);   // 1 = fresh
        float size = lerpf(p.size1, p.size0, t);
        // young: bright yellow-white, old: deep orange
        float r = 1.0f;
        float g = lerpf(0.38f, 0.88f, t);
        float b = lerpf(0.06f, 0.45f, t);
        float a = t * t * 0.9f;

        V3 halfR = right * (size * 0.5f);
        V3 halfU = up * (size * 0.5f);
        uint32_t base = uint32_t(outVerts.size());
        auto push = [&](const V3& off, float u, float v) {
            Vertex vert{};
            V3 w = p.pos + off;
            vert.px = w.x; vert.py = w.y; vert.pz = w.z;
            vert.nx = 0; vert.ny = 0; vert.nz = 1;
            vert.u = u; vert.v = v;
            vert.r = r; vert.g = g; vert.b = b; vert.a = a;
            outVerts.push_back(vert);
        };
        push(-halfR - halfU, 0, 0);
        push(halfR - halfU, 1, 0);
        push(halfR + halfU, 1, 1);
        push(-halfR + halfU, 0, 1);
        outIdx.push_back(base);
        outIdx.push_back(base + 1);
        outIdx.push_back(base + 2);
        outIdx.push_back(base);
        outIdx.push_back(base + 2);
        outIdx.push_back(base + 3);
    }
}
