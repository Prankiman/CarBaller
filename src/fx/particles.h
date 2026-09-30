#pragma once
// Cheap billboard particles: boost flame, ball-hit sparks.

#include "../core/math.h"
#include "../render/gl_utils.h"

#include <vector>

struct Particle {
    V3 pos, vel;
    float life = 0, maxLife = 1;
    float size0 = 10, size1 = 20;
    float r = 1, g = 1, b = 1;
    float drag = 0, grav = 0;
};

class ParticleSystem {
public:
    void clear() { ps_.clear(); }

    // Boost exhaust: cone behind the car. dirBack points away from the car's nose.
    void spawnBoost(const V3& pos, const V3& dirBack, int count);

    // Impact burst at contact point.
    void spawnImpact(const V3& pos, float strength, int quality);

    void update(float dt);

    int count() const { return int(ps_.size()); }

    // Build camera-facing quads (dynamic geometry).
    void buildQuads(const V3& right, const V3& up,
                    std::vector<Vertex>& outVerts, std::vector<uint32_t>& outIdx) const;

private:
    std::vector<Particle> ps_;
    static constexpr size_t MAX_P = 800;
    unsigned rngState_ = 12345;
    float frand() {  // xorshift
        rngState_ ^= rngState_ << 13;
        rngState_ ^= rngState_ >> 17;
        rngState_ ^= rngState_ << 5;
        return (rngState_ & 0xFFFFFF) / float(0xFFFFFF);
    }
    float frand(float a, float b) { return a + (b - a) * frand(); }
};
