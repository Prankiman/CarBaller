#pragma once
// UE3 (.udk/.upk) package reader: summary, name/import/export tables and the
// tagged property chain that precedes every object's native payload.
//
// Scope: vanilla UDK-era packages (FileVersion 867/868, LicenseeVersion 0,
// uncompressed). Layout was validated empirically against the three maps in
// downloaded_maps/ (see README section "Map loader").

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "map_data.h"

namespace ue3 {

using CollChannel = ::maploader::CollChannel;

// One tagged property of an object.
struct Prop {
    std::string name;    // e.g. "Location"
    std::string type;    // e.g. "StructProperty"
    std::string structName;  // only for StructProperty
    std::string enumName;    // only for ByteProperty
    int32_t size = 0;        // DataSize from the tag
    int32_t arrayIndex = 0;
    uint8_t boolVal = 0;     // BoolProperty value (tag carries it, DataSize == 0)

    bool isArray = false;
    int32_t arrayCount = 0;
    int fixedElemSize = 0;   // >0 when the payload is count * fixed bytes
    std::vector<uint8_t> data;            // scalar payload / raw array payload
    std::vector<int32_t> intArray;        // ArrayProperty of 4-byte ints/refs
    std::vector<std::vector<uint8_t>> fixedArray;  // ArrayProperty of fixed-size items
    std::vector<std::vector<Prop>> structArray;    // ArrayProperty of structs

    bool isStruct = false;
    bool structIsNative = false;  // payload could not be read as a nested chain
    std::vector<Prop> nested;

    // convenience accessors (return defaults when the shape does not match)
    float f32(int off = 0) const;
    int32_t i32(int off = 0) const;
    // Vector / Rotator struct payloads
    bool vec3(float* out) const;   // 12-byte float payload
    bool rot3(int32_t* out) const;  // 12-byte int payload (Pitch, Yaw, Roll)
};

// Find by name inside a chain (last occurrence wins, like UE property bags).
const Prop* findProp(const std::vector<Prop>& chain, const char* name);

struct ExportInfo {
    int32_t classRef = 0;   // >0: export, <0: import, 0: None
    int32_t superRef = 0;
    int32_t outerRef = 0;
    int32_t nameIndex = 0;
    int32_t nameInstance = 0;
    int32_t serialOffset = 0;  // absolute file offset
    int32_t serialSize = 0;
};

struct ImportInfo {
    int32_t classPackageIndex = 0;
    int32_t classNameIndex = 0;
    int32_t outerIndex = 0;   // <0 into imports
    int32_t objectNameIndex = 0;
};

// Classifies an object reference into an export/import index (UE-style):
// ref > 0 -> exports[ref - 1], ref < 0 -> imports[-ref - 1], 0 -> None.
inline int refToIndex(int32_t ref) { return ref > 0 ? ref - 1 : (ref < 0 ? -ref - 1 : -1); }

class Package {
public:
    bool load(const std::string& path, std::string* err);

    int version() const { return fileVersion_; }
    int licenseeVersion() const { return licenseeVersion_; }
    int32_t nameCount() const { return (int32_t)names_.size(); }
    const std::string& nameAt(int32_t idx) const;
    std::string resolveFName(int32_t index, int32_t instance) const;

    int32_t numExports() const { return (int32_t)exports_.size(); }
    const ExportInfo& ex(int32_t i) const { return exports_[i]; }
    const ImportInfo& im(int32_t i) const { return imports_[i]; }

    std::string exportName(int32_t i) const;
    std::string exportClassName(int32_t i) const;   // "" when unknown
    // Class name reachable through an object reference (>0 export / <0 import).
    std::string classNameOfRef(int32_t ref) const;
    std::string objectNameOfRef(int32_t ref) const;
    // True when `ref` resolves to an export whose class chain contains `cls`.
    bool refIsA(int32_t ref, const char* cls) const;
    // Walks Super links of an export class; returns true when any link names `cls`.
    bool exportIsA(int32_t exportIdx, const char* cls) const;

    // Parse an object's pre-header + property chain.
    // On success returns the header length (4/8/12/26) and fills `chain`;
    // `chainEnd` receives the absolute offset where the native payload begins.
    bool parseObject(int32_t exportIdx, int* headerLen, std::vector<Prop>* chain,
                     int32_t* chainEnd, std::string* err = nullptr) const;

    // Same, but with an explicit pre-header size (4 plain UObject, 8
    // UActorComponent, 26 AActor subclass). Used when the class is known.
    bool parseObjectAt(int32_t exportIdx, int headerLen, std::vector<Prop>* chain,
                       int32_t* chainEnd, std::string* err = nullptr) const;

    const uint8_t* bytes() const { return data_.data(); }
    size_t size() const { return data_.size(); }

private:
    bool parseSummary(std::string* err);
    bool parseTables(std::string* err);
    bool tryChain(int32_t off, int32_t end, int headerLen, std::vector<Prop>* chain,
                  int32_t* chainEnd) const;

    std::vector<uint8_t> data_;
    int fileVersion_ = 0, licenseeVersion_ = 0;
    int32_t headerSize_ = 0;
    std::vector<std::string> names_;
    std::vector<ImportInfo> imports_;
    std::vector<ExportInfo> exports_;
};

// Bounds-checked little-endian cursor over a byte range.
struct Cursor {
    const uint8_t* p = nullptr;
    int32_t o = 0;
    int32_t end = 0;
    const uint8_t* base = nullptr;

    Cursor() = default;
    Cursor(const uint8_t* d, int32_t start, int32_t stop)
        : p(d), o(start), end(stop), base(d) {}

    bool need(int64_t n) const { return n >= 0 && o >= 0 && (int64_t)o + n <= (int64_t)end; }
    int32_t left() const { return end - o; }

    int32_t i32() {
        if (!need(4)) return 0;
        int32_t v;
        std::memcpy(&v, p + o, 4);
        o += 4;
        return v;
    }
    uint32_t u32() { return (uint32_t)i32(); }
    int16_t i16() {
        if (!need(2)) return 0;
        int16_t v;
        std::memcpy(&v, p + o, 2);
        o += 2;
        return v;
    }
    uint16_t u16() { return (uint16_t)i16(); }
    uint8_t u8() {
        if (!need(1)) return 0;
        return p[o++];
    }
    float f32() {
        if (!need(4)) return 0.f;
        float v;
        std::memcpy(&v, p + o, 4);
        o += 4;
        return v;
    }
    V3 f3() {
        V3 v(0, 0, 0);
        if (!need(12)) return v;
        std::memcpy(&v, p + o, 12);
        o += 12;
        return v;
    }
    void skip(int n) { o += n; }
    const uint8_t* take(int n) {
        if (!need(n)) return nullptr;
        const uint8_t* r = p + o;
        o += n;
        return r;
    }
};

}  // namespace ue3
