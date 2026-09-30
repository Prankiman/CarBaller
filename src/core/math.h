#pragma once
// Minimal 3D math for rendering (RL units, Z-up, left-handed world as in Rocket League).
// World convention (matches RocketSim RotMat): forward = +X at yaw 0,
// car "right" = cross(up, forward) => +Y at yaw 0, up = +Z.

#include <cmath>

struct V3 {
    float x = 0, y = 0, z = 0;

    V3() = default;
    V3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    V3 operator+(const V3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    V3 operator-(const V3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    V3 operator-() const { return {-x, -y, -z}; }
    V3 operator*(float s) const { return {x * s, y * s, z * s}; }
    V3 operator/(float s) const { return {x / s, y / s, z / s}; }
    V3& operator+=(const V3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    V3& operator-=(const V3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    V3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }

    float dot(const V3& o) const { return x * o.x + y * o.y + z * o.z; }
    V3 cross(const V3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
    float len() const { return std::sqrt(x * x + y * y + z * z); }
    float len2d() const { return std::sqrt(x * x + y * y); }
    V3 norm() const {
        float l = len();
        return l > 1e-8f ? V3(x / l, y / l, z / l) : V3(0, 0, 0);
    }
    static V3 lerp(const V3& a, const V3& b, float t) {
        return a + (b - a) * t;
    }
};
inline V3 operator*(float s, const V3& v) { return v * s; }

// "Right" side of a horizontal heading d in RL's world (== RocketSim's convention:
// right = cross(up, forward)).
inline V3 headingRight(const V3& d) { return V3(0, 0, 1).cross(d); }

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }
// shortest-path angle interpolation target
inline float wrapAngle(float a) {
    while (a > 3.14159265f) a -= 6.2831853f;
    while (a < -3.14159265f) a += 6.2831853f;
    return a;
}

// Column-major 4x4 matrix (OpenGL layout).
struct M4 {
    float m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

    static M4 identity() { return M4{}; }

    M4 operator*(const M4& o) const {
        M4 r;
        for (int c = 0; c < 4; c++)
            for (int rr = 0; rr < 4; rr++) {
                float s = 0;
                for (int k = 0; k < 4; k++) s += m[k * 4 + rr] * o.m[c * 4 + k];
                r.m[c * 4 + rr] = s;
            }
        return r;
    }

    V3 mulPoint(const V3& p) const {
        return {
            m[0] * p.x + m[4] * p.y + m[8] * p.z + m[12],
            m[1] * p.x + m[5] * p.y + m[9] * p.z + m[13],
            m[2] * p.x + m[6] * p.y + m[10] * p.z + m[14],
        };
    }
    V3 mulDir(const V3& p) const {
        return {
            m[0] * p.x + m[4] * p.y + m[8] * p.z,
            m[1] * p.x + m[5] * p.y + m[9] * p.z,
            m[2] * p.x + m[6] * p.y + m[10] * p.z,
        };
    }

    static M4 translate(const V3& t) {
        M4 r;
        r.m[12] = t.x; r.m[13] = t.y; r.m[14] = t.z;
        return r;
    }
    static M4 scale(const V3& s) {
        M4 r;
        r.m[0] = s.x; r.m[5] = s.y; r.m[10] = s.z;
        return r;
    }
    static M4 rotateAxis(const V3& axis, float ang) {
        V3 a = axis.norm();
        float c = std::cos(ang), s = std::sin(ang), t = 1 - c;
        M4 r;
        r.m[0] = t * a.x * a.x + c;
        r.m[1] = t * a.x * a.y + s * a.z;
        r.m[2] = t * a.x * a.z - s * a.y;
        r.m[4] = t * a.x * a.y - s * a.z;
        r.m[5] = t * a.y * a.y + c;
        r.m[6] = t * a.y * a.z + s * a.x;
        r.m[8] = t * a.x * a.z + s * a.y;
        r.m[9] = t * a.y * a.z - s * a.x;
        r.m[10] = t * a.z * a.z + c;
        return r;
    }

    // Model matrix from a RocketSim-style orthonormal frame (forward, right, up)
    // as columns: local (1,0,0)->forward, (0,1,0)->right, (0,0,1)->up.
    static M4 fromFrame(const V3& f, const V3& r, const V3& u, const V3& pos) {
        M4 out;
        out.m[0] = f.x; out.m[1] = f.y; out.m[2] = f.z; out.m[3] = 0;
        out.m[4] = r.x; out.m[5] = r.y; out.m[6] = r.z; out.m[7] = 0;
        out.m[8] = u.x; out.m[9] = u.y; out.m[10] = u.z; out.m[11] = 0;
        out.m[12] = pos.x; out.m[13] = pos.y; out.m[14] = pos.z; out.m[15] = 1;
        return out;
    }

    // OpenGL perspective from *horizontal* FOV (RL's FOV setting is horizontal).
    static M4 perspectiveH(float fovX, float aspect, float znear, float zfar) {
        float f = 1.0f / std::tan(fovX * 0.5f);
        float fovY = 2.0f * std::atan(std::tan(fovX * 0.5f) / aspect);
        float fy = 1.0f / std::tan(fovY * 0.5f);
        M4 r{};
        for (auto& v : r.m) v = 0;
        r.m[0] = f;
        r.m[5] = fy;
        r.m[10] = (zfar + znear) / (znear - zfar);
        r.m[11] = -1;
        r.m[14] = (2 * zfar * znear) / (znear - zfar);
        return r;
    }

    // View matrix for RL's world: screen-right = cross(worldUp, viewDir),
    // screen-up ~= worldUp, view direction maps to -Z (OpenGL camera space).
    static M4 viewRL(const V3& eye, const V3& target) {
        V3 d = (target - eye).norm();
        V3 r = headingRight(d);          // horizontal, world right of the view dir
        float rl = r.len();
        if (rl < 1e-5f) {                // looking straight up/down: bail to any frame
            r = V3(1, 0, 0);
            rl = 1;
        }
        r = r / rl;
        V3 back = d * -1.0f;
        V3 u = r.cross(back);            // completes the (r, u, back) frame

        M4 v;
        v.m[0] = r.x; v.m[4] = r.y; v.m[8] = r.z;
        v.m[1] = u.x; v.m[5] = u.y; v.m[9] = u.z;
        v.m[2] = back.x; v.m[6] = back.y; v.m[10] = back.z;
        v.m[12] = -r.dot(eye);
        v.m[13] = -u.dot(eye);
        v.m[14] = -back.dot(eye);
        return v;
    }
};

// Interpolate between two orthonormal frames (bilinear-ish on each basis vector,
// re-orthonormalized). t in [0,1].
inline void lerpFrame(const V3& f0, const V3& r0, const V3& u0,
                      const V3& f1, const V3& r1, const V3& u1,
                      float t, V3& f, V3& r, V3& u) {
    f = V3::lerp(f0, f1, t).norm();
    u = V3::lerp(u0, u1, t).norm();
    // rebuild right = cross(up, forward) (RL convention)
    r = u.cross(f).norm();
    // re-orthogonalize up (identity frame: forward x right == +Z)
    u = f.cross(r).norm();
}
