#pragma once
// Minimal OpenGL helpers: shader program, textured/untextured meshes.

#include <GL/glew.h>
#include <string>
#include <vector>
#include <cstdint>

struct Vertex {
    float px, py, pz;
    float nx, ny, nz;
    float u, v;
    float r, g, b, a;
};

class Shader {
public:
    bool compile(const char* vsSrc, const char* fsSrc, const char* name = "");
    void use() const { glUseProgram(prog_); }
    GLuint id() const { return prog_; }
    GLint loc(const char* n) const { return glGetUniformLocation(prog_, n); }
    void setMat4(const char* n, const float* m16) const;
    void setVec3(const char* n, float x, float y, float z) const;
    void setVec4(const char* n, float x, float y, float z, float w) const;
    void setFloat(const char* n, float v) const;
    void setInt(const char* n, int v) const;
    void destroy() { if (prog_) glDeleteProgram(prog_); prog_ = 0; }

private:
    GLuint prog_ = 0;
};

class GpuMesh {
public:
    void upload(const std::vector<Vertex>& verts, const std::vector<uint32_t>& idx,
                bool dynamic = false);
    void draw() const;
    void destroy();
    bool valid() const { return vao_ != 0; }
    int indexCount() const { return count_; }

private:
    GLuint vao_ = 0, vbo_ = 0, ibo_ = 0;
    int count_ = 0;
};

class Texture2D {
public:
    // RGBA8 data, bottom-up or top-down handled by caller (we set UNPACK_FLIP off)
    bool create(int w, int h, const uint8_t* rgba, bool repeat = true, bool mips = true);
    void bind(int unit) const;
    void destroy();
    bool valid() const { return tex_ != 0; }

private:
    GLuint tex_ = 0;
};

// Simple CPU-side image builder for procedural textures.
struct Image {
    int w = 0, h = 0;
    std::vector<uint8_t> px;  // RGBA
    void init(int w_, int h_) { w = w_; h = h_; px.assign(size_t(w) * h * 4, 0); }
    void set(int x, int y, uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255) {
        if (x < 0 || y < 0 || x >= w || y >= h) return;
        size_t i = (size_t(y) * w + x) * 4;
        px[i] = r; px[i + 1] = g; px[i + 2] = b; px[i + 3] = a;
    }
    void blend(int x, int y, uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
        if (x < 0 || y < 0 || x >= w || y >= h) return;
        size_t i = (size_t(y) * w + x) * 4;
        float t = a / 255.0f;
        px[i + 0] = uint8_t(px[i + 0] * (1 - t) + r * t);
        px[i + 1] = uint8_t(px[i + 1] * (1 - t) + g * t);
        px[i + 2] = uint8_t(px[i + 2] * (1 - t) + b * t);
        px[i + 3] = 255;
    }
};
