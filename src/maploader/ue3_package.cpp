#include "ue3_package.h"

#include <cstdio>
#include <cstring>
#include <fstream>

namespace ue3 {

namespace {
void setErr(std::string* err, const std::string& msg) {
    if (err) *err = msg;
}
}  // namespace

// ---------------------------------------------------------------- accessors

float Prop::f32(int off) const {
    if (off < 0 || (size_t)off + 4 > data.size()) return 0.f;
    float v;
    std::memcpy(&v, data.data() + off, 4);
    return v;
}

int32_t Prop::i32(int off) const {
    if (off < 0 || (size_t)off + 4 > data.size()) return 0;
    int32_t v;
    std::memcpy(&v, data.data() + off, 4);
    return v;
}

bool Prop::vec3(float* out) const {
    if (data.size() != 12) return false;
    std::memcpy(out, data.data(), 12);
    return true;
}

bool Prop::rot3(int32_t* out) const {
    if (data.size() < 12) return false;
    std::memcpy(out, data.data(), 12);
    return true;
}

const Prop* findProp(const std::vector<Prop>& chain, const char* name) {
    const Prop* found = nullptr;
    for (const Prop& p : chain)
        if (p.name == name) found = &p;
    return found;
}

// ---------------------------------------------------------------- package

bool Package::load(const std::string& path, std::string* err) {
    data_.clear();
    names_.clear();
    imports_.clear();
    exports_.clear();

    std::ifstream in(path, std::ios::binary);
    if (!in) {
        setErr(err, "cannot open " + path);
        return false;
    }
    in.seekg(0, std::ios::end);
    std::streamoff sz = in.tellg();
    if (sz <= 0 || sz > (std::streamoff)1 << 31) {
        setErr(err, "bad file size");
        return false;
    }
    in.seekg(0, std::ios::beg);
    data_.resize((size_t)sz);
    in.read((char*)data_.data(), sz);
    if (!in) {
        setErr(err, "read failed");
        return false;
    }
    if (!parseSummary(err)) return false;
    if (!parseTables(err)) return false;
    return true;
}

bool Package::parseSummary(std::string* err) {
    const uint8_t* d = data_.data();
    if (data_.size() < 129) {
        setErr(err, "file too small to be a UE3 package");
        return false;
    }
    uint32_t tag;
    std::memcpy(&tag, d, 4);
    if (tag != 0x9E2A83C1u) {
        char buf[64];
        std::snprintf(buf, sizeof buf, "bad package tag 0x%08X", tag);
        setErr(err, buf);
        return false;
    }
    uint32_t ver;
    std::memcpy(&ver, d + 4, 4);
    fileVersion_ = (int)(ver & 0xFFFF);
    licenseeVersion_ = (int)(ver >> 16);
    std::memcpy(&headerSize_, d + 8, 4);

    size_t p = 12;
    int32_t folderLen = 0;
    std::memcpy(&folderLen, d + p, 4);
    p += 4;
    if (folderLen < 0 || p + (size_t)folderLen > data_.size()) {
        setErr(err, "bad folder name");
        return false;
    }
    p += (size_t)folderLen;

    auto rd = [&](int off) {
        int32_t v;
        std::memcpy(&v, d + p + off, 4);
        return v;
    };
    int32_t nameCount = rd(4), nameOffset = rd(8);
    int32_t exportCount = rd(12), exportOffset = rd(16);
    int32_t importCount = rd(20), importOffset = rd(24);
    p += 48;  // 12 ints: flags, nameCount..unk38
    p += 16;  // package guid
    int32_t genCount = 0;
    std::memcpy(&genCount, d + p, 4);
    p += 4;
    if (genCount < 0 || genCount > 4096) {
        setErr(err, "bad generation count");
        return false;
    }
    p += (size_t)genCount * 12;  // {ExportCount, NameCount, NetObjectCount} per generation

    // EngineVersion, CookerVersion, CompressionFlags, CompressedChunkCount
    int32_t compressionFlags = 0, chunkCount = 0;
    if (p + 16 <= data_.size()) {
        std::memcpy(&compressionFlags, d + p + 8, 4);
        std::memcpy(&chunkCount, d + p + 12, 4);
    }
    if (compressionFlags != 0 || chunkCount != 0) {
        setErr(err, "compressed packages are not supported");
        return false;
    }

    if (nameCount < 0 || exportCount < 0 || importCount < 0 ||
        nameOffset < 0 || exportOffset < 0 || importOffset < 0) {
        setErr(err, "negative table size");
        return false;
    }

    // name table
    size_t q = (size_t)nameOffset;
    names_.reserve((size_t)nameCount);
    for (int32_t i = 0; i < nameCount; i++) {
        if (q + 4 > data_.size()) {
            setErr(err, "name table overrun");
            return false;
        }
        int32_t len;
        std::memcpy(&len, d + q, 4);
        q += 4;
        if (len <= 0 || len > 1024 || q + (size_t)len + 8 > data_.size()) {
            setErr(err, "bad name entry");
            return false;
        }
        std::string s((const char*)d + q, (size_t)len);
        while (!s.empty() && s.back() == '\0') s.pop_back();
        names_.push_back(std::move(s));
        q += (size_t)len + 8;  // + 8-byte FTaggedName flags
    }

    // import table: 7 int32 each
    imports_.reserve((size_t)importCount);
    q = (size_t)importOffset;
    for (int32_t i = 0; i < importCount; i++) {
        if (q + 28 > data_.size()) {
            setErr(err, "import table overrun");
            return false;
        }
        ImportInfo im;
        // FObjectImport: ClassPackage(FName 8B), ClassName(FName 8B),
        // PackageIndex(4B), ObjectName(FName 8B) = 28 bytes
        std::memcpy(&im.classPackageIndex, d + q + 0, 4);
        std::memcpy(&im.classNameIndex, d + q + 8, 4);
        std::memcpy(&im.outerIndex, d + q + 16, 4);
        std::memcpy(&im.objectNameIndex, d + q + 20, 4);
        imports_.push_back(im);
        q += 28;
    }

    // export table: 13 int32 + 16-byte guid = 68 bytes
    exports_.reserve((size_t)exportCount);
    q = (size_t)exportOffset;
    for (int32_t i = 0; i < exportCount; i++) {
        if (q + 68 > data_.size()) {
            setErr(err, "export table overrun");
            return false;
        }
        ExportInfo e;
        std::memcpy(&e.classRef, d + q + 0, 4);
        std::memcpy(&e.superRef, d + q + 4, 4);
        std::memcpy(&e.outerRef, d + q + 8, 4);
        std::memcpy(&e.nameIndex, d + q + 12, 4);
        std::memcpy(&e.nameInstance, d + q + 16, 4);
        std::memcpy(&e.serialSize, d + q + 32, 4);
        std::memcpy(&e.serialOffset, d + q + 36, 4);
        exports_.push_back(e);
        q += 68;
    }

    for (const ExportInfo& e : exports_) {
        if (e.serialOffset < 0 || e.serialSize < 0 ||
            (size_t)e.serialOffset + (size_t)e.serialSize > data_.size()) {
            setErr(err, "export payload out of range");
            return false;
        }
        if (e.nameIndex < 0 || e.nameIndex >= (int32_t)names_.size()) {
            setErr(err, "export name index out of range");
            return false;
        }
    }
    return true;
}

bool Package::parseTables(std::string*) { return true; }

const std::string& Package::nameAt(int32_t idx) const {
    static const std::string empty;
    if (idx < 0 || idx >= (int32_t)names_.size()) return empty;
    return names_[idx];
}

std::string Package::resolveFName(int32_t index, int32_t) const {
    return nameAt(index);
}

std::string Package::exportName(int32_t i) const {
    if (i < 0 || i >= (int32_t)exports_.size()) return std::string();
    return nameAt(exports_[i].nameIndex);
}

std::string Package::classNameOfRef(int32_t ref) const {
    int idx = refToIndex(ref);
    if (ref > 0) {
        if (idx < 0 || idx >= (int32_t)exports_.size()) return std::string();
        int32_t cr = exports_[idx].classRef;
        int ci = refToIndex(cr);
        if (cr > 0) return exportName(ci);
        if (cr < 0 && ci >= 0 && ci < (int32_t)imports_.size())
            return nameAt(imports_[ci].objectNameIndex);
        return std::string();
    }
    if (ref < 0 && idx >= 0 && idx < (int32_t)imports_.size())
        return nameAt(imports_[idx].objectNameIndex);
    return std::string();
}

std::string Package::objectNameOfRef(int32_t ref) const {
    int idx = refToIndex(ref);
    if (ref > 0) return exportName(idx);
    if (ref < 0 && idx >= 0 && idx < (int32_t)imports_.size())
        return nameAt(imports_[idx].objectNameIndex);
    return std::string();
}

std::string Package::exportClassName(int32_t i) const {
    if (i < 0 || i >= (int32_t)exports_.size()) return std::string();
    return classNameOfRef(exports_[i].classRef);
}

bool Package::refIsA(int32_t ref, const char* cls) const {
    if (ref <= 0) return false;
    return exportIsA(ref - 1, cls);
}

bool Package::exportIsA(int32_t exportIdx, const char* cls) const {
    if (exportIdx < 0 || exportIdx >= (int32_t)exports_.size()) return false;
    int32_t ref = exports_[exportIdx].classRef;
    for (int guard = 0; guard < 64 && ref != 0; guard++) {
        int idx = refToIndex(ref);
        if (ref < 0) {  // import: object name of the class object
            if (idx < 0 || idx >= (int32_t)imports_.size()) return false;
            if (nameAt(imports_[idx].objectNameIndex) == cls) return true;
            if (nameAt(imports_[idx].classNameIndex) == cls) return true;
            return false;
        }
        if (idx < 0 || idx >= (int32_t)exports_.size()) return false;
        if (nameAt(exports_[idx].nameIndex) == cls) return true;
        ref = exports_[idx].superRef;
    }
    return false;
}

// ---------------------------------------------------------------- properties

namespace {

struct ChainErr {
    bool bad = false;
};

bool parseChain(Cursor& c, const std::vector<std::string>& names, std::vector<Prop>& out,
                int depth) {
    if (depth > 6) return false;
    for (int guard = 0; guard < 4096; guard++) {
        if (!c.need(8)) return false;
        int32_t nameIdx = c.i32();
        c.u32();
        if (nameIdx < 0 || nameIdx >= (int32_t)names.size()) return false;
        const std::string& pname = names[nameIdx];
        if (pname == "None") return true;
        if (!c.need(8)) return false;
        int32_t typeIdx = c.i32();
        c.u32();
        if (typeIdx < 0 || typeIdx >= (int32_t)names.size()) return false;
        const std::string& ptype = names[typeIdx];
        int32_t size = c.i32();
        int32_t arr = c.i32();
        if (size < 0 || !c.need(size)) return false;

        Prop p;
        p.name = pname;
        p.type = ptype;
        p.size = size;
        p.arrayIndex = arr;

        if (ptype == "StructProperty") {
            if (!c.need(8)) return false;
            int32_t sn = c.i32();
            c.u32();
            if (sn < 0 || sn >= (int32_t)names.size()) return false;
            p.structName = names[sn];
        } else if (ptype == "BoolProperty") {
            p.boolVal = c.u8();
            p.data.clear();
            out.push_back(std::move(p));
            continue;
        } else if (ptype == "ByteProperty") {
            if (!c.need(8)) return false;
            int32_t en = c.i32();
            c.u32();
            if (en < 0 || en >= (int32_t)names.size()) return false;
            p.enumName = names[en];
        }

        int32_t payloadStart = c.o;
        int32_t payloadEnd = payloadStart + size;
        if (payloadEnd > c.end) return false;

        if (ptype == "ArrayProperty") {
            int32_t count = c.i32();
            p.isArray = true;
            p.arrayCount = count;
            int32_t rem = payloadEnd - c.o;
            int32_t sub = c.o;
            bool done = false;
            if (count > 0 && rem == count * 4) {
                p.intArray.resize((size_t)count);
                if (count) std::memcpy(p.intArray.data(), c.p + sub, (size_t)count * 4);
                c.o = payloadEnd;
                done = true;
            }
            if (!done) {
                std::vector<std::vector<Prop>> elems;
                bool ok = count > 0;
                for (int32_t i = 0; ok && i < count; i++) {
                    std::vector<Prop> sub2;
                    if (!parseChain(c, names, sub2, depth + 1)) {
                        ok = false;
                        break;
                    }
                    elems.push_back(std::move(sub2));
                }
                if (ok && c.o == payloadEnd) {
                    p.structArray = std::move(elems);
                    done = true;
                } else {
                    c.o = sub;
                }
            }
            if (!done) {
                if (count > 0 && rem % count == 0) {
                    p.fixedElemSize = rem / count;
                    p.fixedArray.resize((size_t)count);
                    for (int32_t i = 0; i < count; i++)
                        p.fixedArray[i].assign(c.p + sub + (size_t)i * p.fixedElemSize,
                                              c.p + sub + (size_t)(i + 1) * p.fixedElemSize);
                } else {
                    p.data.assign(c.p + sub, c.p + payloadEnd);
                }
                c.o = payloadEnd;
            }
        } else if (ptype == "StructProperty") {
            Cursor sub = c;
            sub.o = payloadStart;
            sub.end = payloadEnd;
            std::vector<Prop> nested;
            if (parseChain(sub, names, nested, depth + 1) && sub.o == payloadEnd) {
                p.nested = std::move(nested);
            } else {
                p.structIsNative = true;
                p.data.assign(c.p + payloadStart, c.p + payloadEnd);
            }
            c.o = payloadEnd;
        } else {
            p.data.assign(c.p + payloadStart, c.p + payloadEnd);
            c.o = payloadEnd;
        }
        out.push_back(std::move(p));
    }
    return false;
}

}  // namespace

bool Package::tryChain(int32_t off, int32_t end, int headerLen, std::vector<Prop>* chain,
                       int32_t* chainEnd) const {
    if (headerLen > end - off) return false;
    Cursor c(data_.data(), off + headerLen, end);
    chain->clear();
    if (!parseChain(c, names_, *chain, 0)) return false;
    *chainEnd = c.o;
    return true;
}

bool Package::parseObject(int32_t exportIdx, int* headerLen, std::vector<Prop>* chain,
                          int32_t* chainEnd, std::string* err) const {
    if (exportIdx < 0 || exportIdx >= (int32_t)exports_.size()) {
        setErr(err, "export index out of range");
        return false;
    }
    const ExportInfo& e = exports_[exportIdx];
    int32_t off = e.serialOffset;
    int32_t end = e.serialOffset + e.serialSize;
    if (e.serialSize <= 0) {
        *headerLen = 0;
        chain->clear();
        *chainEnd = off;
        return true;
    }
    static const int kHeaders[4] = {4, 8, 12, 26};
    // exact match wins
    for (int h : kHeaders) {
        int32_t ce = 0;
        if (tryChain(off, end, h, chain, &ce) && ce == end) {
            *headerLen = h;
            *chainEnd = ce;
            return true;
        }
    }
    // otherwise take the parse that leaves the least unconsumed payload
    int best = -1;
    int32_t bestErr = 0, bestEnd = 0;
    std::vector<Prop> bestChain;
    for (int h : kHeaders) {
        int32_t ce = 0;
        std::vector<Prop> tmp;
        if (!tryChain(off, end, h, &tmp, &ce)) continue;
        int32_t errv = ce - end;
        if (errv < 0) errv = -errv;
        if (best < 0 || errv < bestErr) {
            best = h;
            bestErr = errv;
            bestEnd = ce;
            bestChain = std::move(tmp);
        }
    }
    if (best < 0) {
        setErr(err, "no object header fits export '" + exportName(exportIdx) + "'");
        return false;
    }
    *headerLen = best;
    *chainEnd = bestEnd;
    *chain = std::move(bestChain);
    return true;
}

bool Package::parseObjectAt(int32_t exportIdx, int headerLen, std::vector<Prop>* chain,
                            int32_t* chainEnd, std::string* err) const {
    if (exportIdx < 0 || exportIdx >= (int32_t)exports_.size()) {
        setErr(err, "export index out of range");
        return false;
    }
    const ExportInfo& e = exports_[exportIdx];
    if (e.serialSize <= 0) {
        chain->clear();
        *chainEnd = e.serialOffset;
        return true;
    }
    if (!tryChain(e.serialOffset, e.serialOffset + e.serialSize, headerLen, chain, chainEnd)) {
        setErr(err, "property chain does not fit export '" + exportName(exportIdx) + "'");
        return false;
    }
    return true;
}

}  // namespace ue3
