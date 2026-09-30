#include "gl_utils.h"

#include <cstdio>

static GLuint compileStage(GLenum type, const char* src, const char* name) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[4096];
        glGetShaderInfoLog(s, sizeof(log), nullptr, log);
        std::fprintf(stderr, "[gl] %s shader compile error (%s):\n%s\n",
                     type == GL_VERTEX_SHADER ? "vertex" : "fragment", name, log);
        glDeleteShader(s);
        return 0;
    }
    return s;
}

bool Shader::compile(const char* vsSrc, const char* fsSrc, const char* name) {
    GLuint vs = compileStage(GL_VERTEX_SHADER, vsSrc, name);
    GLuint fs = compileStage(GL_FRAGMENT_SHADER, fsSrc, name);
    if (!vs || !fs) return false;
    GLuint p = glCreateProgram();
    glAttachShader(p, vs);
    glAttachShader(p, fs);
    glLinkProgram(p);
    glDeleteShader(vs);
    glDeleteShader(fs);
    GLint ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[4096];
        glGetProgramInfoLog(p, sizeof(log), nullptr, log);
        std::fprintf(stderr, "[gl] program link error (%s):\n%s\n", name, log);
        glDeleteProgram(p);
        return false;
    }
    if (prog_) glDeleteProgram(prog_);
    prog_ = p;
    return true;
}

void Shader::setMat4(const char* n, const float* m) const { glUniformMatrix4fv(loc(n), 1, GL_FALSE, m); }
void Shader::setVec3(const char* n, float x, float y, float z) const { glUniform3f(loc(n), x, y, z); }
void Shader::setVec4(const char* n, float x, float y, float z, float w) const { glUniform4f(loc(n), x, y, z, w); }
void Shader::setFloat(const char* n, float v) const { glUniform1f(loc(n), v); }
void Shader::setInt(const char* n, int v) const { glUniform1i(loc(n), v); }

void GpuMesh::upload(const std::vector<Vertex>& verts, const std::vector<uint32_t>& idx,
                     bool dynamic) {
    if (!vao_) {
        glGenVertexArrays(1, &vao_);
        glGenBuffers(1, &vbo_);
        glGenBuffers(1, &ibo_);
    }
    GLenum usage = dynamic ? GL_STREAM_DRAW : GL_STATIC_DRAW;
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(verts.size() * sizeof(Vertex)), verts.data(), usage);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(idx.size() * 4), idx.data(), usage);

    const GLsizei stride = sizeof(Vertex);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(Vertex, px));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(Vertex, nx));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(Vertex, u));
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(Vertex, r));
    glBindVertexArray(0);
    count_ = int(idx.size());
}

void GpuMesh::draw() const {
    if (!vao_) return;
    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, count_, GL_UNSIGNED_INT, nullptr);
}

void GpuMesh::destroy() {
    if (vao_) glDeleteVertexArrays(1, &vao_);
    if (vbo_) glDeleteBuffers(1, &vbo_);
    if (ibo_) glDeleteBuffers(1, &ibo_);
    vao_ = vbo_ = ibo_ = 0;
    count_ = 0;
}

bool Texture2D::create(int w, int h, const uint8_t* rgba, bool repeat, bool mips) {
    if (!tex_) glGenTextures(1, &tex_);
    glBindTexture(GL_TEXTURE_2D, tex_);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, repeat ? GL_REPEAT : GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, repeat ? GL_REPEAT : GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    if (mips) {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glGenerateMipmap(GL_TEXTURE_2D);
    } else {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    }
    return true;
}

void Texture2D::bind(int unit) const {
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_2D, tex_);
}

void Texture2D::destroy() {
    if (tex_) glDeleteTextures(1, &tex_);
    tex_ = 0;
}
