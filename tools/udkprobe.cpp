// udkprobe <mode> ... - read a cooked UE3 package (.udk/.upk) the way the
// map loader does, without loading geometry or running the game. For
// figuring out *where the file says things are*: checkpoint courses, ball
// spawns and teleports, Kismet dispatch ladders, raw property chains.
//
// Modes (each takes the map path first):
//   kismet <map>            ball/course Kismet ops (AddGameBall, Teleport,
//                           SetLocation, SetVelocity, CompareInt, SetInt,
//                           ModifyObjectList, ...), every variable link
//                           resolved to a location where the package says
//                           one, plus the ops each output feeds.
//   ladder <map>            every SeqCond_CompareInt: its section constant
//                           (ValueB) and the ops its outputs fire - the
//                           dispatcher ladder that numbers a course.
//   vars <map>              every SeqVar_* carrying a VarName: its binding
//                           (ObjValue actor + location, int/float value).
//                           "PlayerSpawnN"/"BallSpawnN" series live here.
//   find <map> <substr>...  exports whose object name or Tag matches.
//   chain <map> <idx>...    the raw property chain (and native tail bytes)
//                           of one or more exports, by index.
//
// Build (from repo root):
//   c++ -std=c++20 -O2 -I. -Isrc tools/udkprobe.cpp src/maploader/ue3_package.cpp \
//       -o udkprobe
// Run:
//   ./udkprobe ladder downloaded_maps/r3.udk
//   ./udkprobe kismet downloaded_maps/TJBrotherAirDribble.udk | grep -i ball
#include <cctype>
#include <cstdio>
#include <cstring>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "maploader/ue3_package.h"

using ue3::Package;
using ue3::Prop;

// ---------------------------------------------------------------- helpers --

static std::string lower(std::string s) {
    for (char& c : s) c = (char)std::tolower((unsigned char)c);
    return s;
}

// StringProperty payload: [i32 byte length][bytes].
static std::string propStr(const Prop& p) {
    if (p.data.size() < 4) return {};
    int32_t len;
    std::memcpy(&len, p.data.data(), 4);
    if (len <= 0 || (size_t)len + 4 > p.data.size()) return {};
    std::string s(reinterpret_cast<const char*>(p.data.data()) + 4, (size_t)len);
    while (!s.empty() && s.back() == '\0') s.pop_back();
    return s;
}

// NameProperty payload: [i32 name index][i32 instance] into the name table.
static std::string propName(const Package& pk, const Prop& p) {
    if (p.type != "NameProperty" || p.data.size() < 4) return {};
    const int32_t idx = p.i32(0);
    const int32_t inst = p.data.size() >= 8 ? p.i32(4) : 0;
    if (idx < 0 || idx >= pk.nameCount()) return {};
    return pk.resolveFName(idx, inst);
}

// Object refs out of an array property: a flat int array (LinkedVariables)
// or per-element structs carrying "LinkedOp" (an output link's Links).
static void arrayRefs(const Prop* p, std::vector<int32_t>* out) {
    if (!p) return;
    for (int32_t v : p->intArray) out->push_back(v);
    for (const std::vector<uint8_t>& e : p->fixedArray) {
        if (e.size() >= 4) {
            int32_t v;
            std::memcpy(&v, e.data(), 4);
            out->push_back(v);
        }
    }
    for (const std::vector<Prop>& e : p->structArray)
        if (const Prop* lo = ue3::findProp(e, "LinkedOp")) out->push_back(lo->i32());
}

// The actor (or anything with a Location) behind an object ref.
static bool locOfRef(const Package& pk, int32_t ref, V3* loc) {
    if (ref <= 0 || ref > pk.numExports()) return false;
    std::vector<Prop> chain;
    int hdr = 0;
    int32_t end = 0;
    if (!pk.parseObject(ref - 1, &hdr, &chain, &end)) return false;
    const Prop* l = ue3::findProp(chain, "Location");
    if (!l || l->data.size() < 12) return false;
    std::memcpy(loc, l->data.data(), 12);
    return true;
}

// A variable ref -> a location, the same ways the loader resolves one:
// SeqVar_Object through its ObjValue actor, SeqVar_Named through the
// tag/object tables, SeqVar_Vector from its value, an actor directly.
struct World {
    const Package* pk = nullptr;
    std::map<std::string, V3> tagLoc;   // actor Tag -> Location
    std::map<std::string, V3> nameLoc;  // actor object name -> Location

    void build() {
        const int n = pk->numExports();
        std::vector<Prop> chain;
        for (int k = 0; k < n; k++) {
            // the loader's own actor test (ue3::scanActors): a non-component
            // object carrying a Location struct. Class defaults and imports
            // with default-valued props are skipped by name.
            const std::string cls = pk->exportClassName(k);
            if (cls.empty() || (cls.size() >= 9 && cls.compare(cls.size() - 9, 9,
                                                              "Component") == 0))
                continue;
            const std::string nm = pk->exportName(k);
            if (nm.rfind("Default__", 0) == 0) continue;
            int hdr = 0;
            int32_t end = 0;
            if (!pk->parseObject(k, &hdr, &chain, &end)) continue;
            const Prop* loc = ue3::findProp(chain, "Location");
            if (!loc || loc->type != "StructProperty" || loc->data.size() < 12) continue;
            V3 f;
            std::memcpy(&f, loc->data.data(), 12);
            nameLoc[nm] = f;
            if (const Prop* tag = ue3::findProp(chain, "Tag"))
                if (tag->type == "NameProperty") {
                    const std::string t = propName(*pk, *tag);
                    if (!t.empty()) tagLoc[t] = f;
                }
        }
    }

    bool varLoc(int32_t ref, V3* loc, std::string* how) const {
        if (ref <= 0 || ref > pk->numExports()) return false;
        const std::string cls = pk->classNameOfRef(ref);
        std::vector<Prop> c;
        int hdr = 0;
        int32_t end = 0;
        if (!pk->parseObject(ref - 1, &hdr, &c, &end)) return false;
        if (cls == "SeqVar_Object") {
            const Prop* ov = ue3::findProp(c, "ObjValue");
            if (!ov || !locOfRef(*pk, ov->i32(), loc)) return false;
            if (how)
                *how = "obj->" + pk->objectNameOfRef(ov->i32()) + "(" +
                       pk->classNameOfRef(ov->i32()) + ")";
            return true;
        }
        if (cls == "SeqVar_Named") {
            const Prop* fn = ue3::findProp(c, "FindVarName");
            const std::string nm = fn ? propName(*pk, *fn) : "";
            if (nm.empty()) return false;
            auto it = tagLoc.find(nm);
            if (it != tagLoc.end()) {
                *loc = it->second;
                if (how) *how = "tag\"" + nm + "\"";
                return true;
            }
            it = nameLoc.find(nm);
            if (it != nameLoc.end()) {
                *loc = it->second;
                if (how) *how = "named\"" + nm + "\"";
                return true;
            }
            return false;
        }
        if (cls == "SeqVar_Vector") {
            const Prop* v = ue3::findProp(c, "Value");
            if (v && v->data.size() >= 12) {
                std::memcpy(loc, v->data.data(), 12);
                if (how) *how = "vector";
                return true;
            }
            return false;
        }
        return locOfRef(*pk, ref, loc);
    }
};

// ------------------------------------------------------------------ modes --

// kismet: the ops that place cars and balls, with everything resolvable
// resolved. The four course schemes the workshop maps use all show up here.
static int modeKismet(const char* path) {
    Package pk;
    std::string err;
    if (!pk.load(path, &err)) {
        std::fprintf(stderr, "LOADFAIL %s\n", err.c_str());
        return 1;
    }
    const int n = pk.numExports();
    World w;
    w.pk = &pk;
    w.build();
    std::printf("== tagged actors (%zu):\n", w.tagLoc.size());
    int shown = 0;
    for (const auto& [t, l] : w.tagLoc) {
        std::printf("  %-32s (%.0f,%.0f,%.0f)\n", t.c_str(), l.x, l.y, l.z);
        if (++shown >= 30) {
            std::printf("  ...\n");
            break;
        }
    }

    std::vector<Prop> chain;
    for (int k = 0; k < n; k++) {
        const std::string cls = pk.exportClassName(k);
        if (cls.empty()) continue;
        int hdr = 0;
        int32_t end = 0;
        if (!pk.parseObject(k, &hdr, &chain, &end)) continue;

        if (cls == "SeqVar_Named") {
            const Prop* fn = ue3::findProp(chain, "FindVarName");
            const std::string nm = fn ? propName(pk, *fn) : "";
            if (nm.empty()) continue;
            V3 l;
            std::string how;
            if (w.varLoc(k, &l, &how))
                std::printf("VarNamed#%d \"%s\" -> (%.0f,%.0f,%.0f)\n", k, nm.c_str(), l.x,
                            l.y, l.z);
            else
                std::printf("VarNamed#%d \"%s\" -> unresolved (runtime)\n", k, nm.c_str());
            continue;
        }
        const bool isBall = cls.find("AddGameBall") != std::string::npos;
        const bool isTele = cls == "SeqAct_Teleport";
        const bool isVec = cls == "SeqVar_Vector";
        const bool isMore =
            cls == "SeqAct_SetLocation" || cls == "SeqAct_SetVelocity" ||
            cls == "SeqAct_GetLocationAndRotation" || cls == "SeqAct_SetVectorComponents" ||
            cls == "SeqAct_GetVectorComponents" || cls == "SeqAct_Destroy" ||
            cls == "SeqAct_SetInt" || cls == "SeqCond_CompareInt" ||
            cls == "SeqAct_FinishSequence" || cls == "SeqAct_ModifyObjectList" ||
            cls == "SeqAct_AddInt" || cls == "SeqAct_Toggle" ||
            cls == "SeqEvent_Touch" || cls == "SeqEvent_PlayerSpawned";
        if (!isBall && !isTele && !isMore && !isVec) continue;
        if (isVec) {
            V3 f;
            std::string how;
            if (w.varLoc(k, &f, &how))
                std::printf("VarVector#%d=(%.0f,%.0f,%.0f) [%s]\n", k, f.x, f.y, f.z,
                            how.c_str());
            continue;
        }

        std::printf("%s#%d:", cls.c_str(), k);
        if (const Prop* vl = ue3::findProp(chain, "VariableLinks")) {
            for (const std::vector<Prop>& link : vl->structArray) {
                const Prop* d = ue3::findProp(link, "LinkDesc");
                std::string desc;
                if (d) {
                    if (d->type == "NameProperty")
                        desc = propName(pk, *d);
                    else
                        desc = propStr(*d);
                }
                std::vector<int32_t> refs;
                arrayRefs(ue3::findProp(link, "LinkedVariables"), &refs);
                if (refs.empty()) {
                    std::printf(" \"%s\"=-", desc.c_str());
                    continue;
                }
                for (int32_t r : refs) {
                    V3 l;
                    std::string how;
                    std::printf(" \"%s\"=[%s %s", desc.c_str(),
                                r > 0 && r <= n ? pk.classNameOfRef(r).c_str() : "imp",
                                r > 0 && r <= n ? pk.objectNameOfRef(r).c_str() : "?");
                    if (w.varLoc(r, &l, &how))
                        std::printf(" %s loc=(%.0f,%.0f,%.0f)", how.c_str(), l.x, l.y, l.z);
                    std::printf("]");
                }
            }
        }
        for (const Prop& p : chain) {
            if (p.name == "IntValue" && p.data.size() == 4)
                std::printf(" IntValue=%d", p.i32());
            if (p.name == "ValueB" && p.data.size() == 4)
                std::printf(" ValueB=%d", p.i32());
            if (p.data.size() != 12) continue;
            if (p.name == "Location" || p.name.find("ector") != std::string::npos) {
                float f[3];
                std::memcpy(f, p.data.data(), 12);
                std::printf(" <%s>=(%.0f,%.0f,%.0f)", p.name.c_str(), f[0], f[1], f[2]);
            }
        }
        if (const Prop* ol = ue3::findProp(chain, "OutputLinks")) {
            std::vector<int32_t> refs;
            for (const std::vector<Prop>& link : ol->structArray)
                arrayRefs(ue3::findProp(link, "Links"), &refs);
            if (!refs.empty()) {
                std::printf(" OUT:");
                for (int32_t r : refs)
                    if (r > 0 && r <= n)
                        std::printf(" %s#%d", pk.exportClassName(r - 1).c_str(), r - 1);
            }
        }
        std::printf("\n");
    }
    return 0;
}

// The destination an op names: the first ref of its "Destination" (or
// "Target") variable link, unwrapped through SeqVar_Object to the actor.
static bool opDestLoc(const Package& pk, int32_t opIdx, const char* linkName, V3* loc) {
    std::vector<Prop> chain;
    int hdr = 0;
    int32_t end = 0;
    if (!pk.parseObject(opIdx, &hdr, &chain, &end)) return false;
    if (const Prop* vl = ue3::findProp(chain, "VariableLinks"))
        for (const std::vector<Prop>& link : vl->structArray) {
            const Prop* d = ue3::findProp(link, "LinkDesc");
            if (!d || propStr(*d) != linkName) continue;
            std::vector<int32_t> refs;
            arrayRefs(ue3::findProp(link, "LinkedVariables"), &refs);
            for (int32_t r : refs) {
                if (r > 0 && r <= pk.numExports() &&
                    pk.classNameOfRef(r) == "SeqVar_Object") {
                    std::vector<Prop> vc;
                    int h2 = 0;
                    int32_t e2 = 0;
                    if (pk.parseObject(r - 1, &h2, &vc, &e2))
                        if (const Prop* ov = ue3::findProp(vc, "ObjValue"))
                            r = ov->i32();
                }
                if (locOfRef(pk, r, loc)) return true;
            }
        }
    return false;
}

// ladder: the section constants of the CompareInt dispatcher and what each
// condition fires - the course order of a map driven by teleports (r3).
static int modeLadder(const char* path) {
    Package pk;
    std::string err;
    if (!pk.load(path, &err)) {
        std::fprintf(stderr, "LOADFAIL %s\n", err.c_str());
        return 1;
    }
    const int n = pk.numExports();
    std::vector<Prop> chain;
    for (int k = 0; k < n; k++) {
        if (pk.exportClassName(k) != "SeqCond_CompareInt") continue;
        int hdr = 0;
        int32_t end = 0;
        if (!pk.parseObject(k, &hdr, &chain, &end)) continue;
        const Prop* vb = ue3::findProp(chain, "ValueB");
        const Prop* va = ue3::findProp(chain, "ValueA");
        std::printf("Cond#%d ValueB=%d", k, vb ? vb->i32() : -9999);
        if (va) std::printf(" ValueA=%d", va->i32());
        if (const Prop* ol = ue3::findProp(chain, "OutputLinks")) {
            std::set<int> seen;
            for (const std::vector<Prop>& link : ol->structArray) {
                std::vector<int32_t> refs;
                arrayRefs(ue3::findProp(link, "Links"), &refs);
                for (int32_t r : refs) {
                    if (r <= 0 || r > n || seen.count(r - 1)) continue;
                    seen.insert(r - 1);
                    std::printf(" -> %s#%d", pk.exportClassName(r - 1).c_str(), r - 1);
                    if (pk.exportClassName(r - 1) == "SeqAct_Teleport") {
                        V3 l;
                        if (opDestLoc(pk, r - 1, "Destination", &l))
                            std::printf(" [dest (%.0f,%.0f,%.0f)]", l.x, l.y, l.z);
                    }
                }
            }
        }
        std::printf("\n");
    }
    return 0;
}

// vars: the Kismet variables carrying a name and a binding - numbered
// PlayerSpawnN/BallSpawnN checkpoint series show up here.
static int modeVars(const char* path) {
    Package pk;
    std::string err;
    if (!pk.load(path, &err)) {
        std::fprintf(stderr, "LOADFAIL %s\n", err.c_str());
        return 1;
    }
    std::vector<Prop> chain;
    for (int k = 0; k < pk.numExports(); k++) {
        const std::string cls = pk.exportClassName(k);
        if (cls.rfind("SeqVar_", 0) != 0) continue;
        int hdr = 0;
        int32_t end = 0;
        if (!pk.parseObject(k, &hdr, &chain, &end)) continue;
        const Prop* vn = ue3::findProp(chain, "VarName");
        const std::string varName = vn ? propName(pk, *vn) : "";
        if (varName.empty()) continue;
        std::printf("%s#%d VarName=\"%s\"", cls.c_str(), k, varName.c_str());
        if (const Prop* ov = ue3::findProp(chain, "ObjValue")) {
            const int32_t r = ov->i32();
            std::printf(" ObjValue=%d", r);
            V3 l;
            if (locOfRef(pk, r, &l))
                std::printf(" (%s %s loc=(%.0f,%.0f,%.0f))", pk.classNameOfRef(r).c_str(),
                            pk.objectNameOfRef(r).c_str(), l.x, l.y, l.z);
        }
        if (const Prop* iv = ue3::findProp(chain, "IntValue")) std::printf(" Int=%d", iv->i32());
        if (const Prop* fv = ue3::findProp(chain, "FloatValue"))
            std::printf(" Float=%g", fv->f32());
        for (const Prop& p : chain)
            if (p.isArray && p.arrayCount > 0) std::printf(" arr:%s(%d)", p.name.c_str(), p.arrayCount);
        std::printf("\n");
    }
    return 0;
}

// find: exports whose object name or Tag matches one of the substrings.
static int modeFind(const char* path, int argc, char** argv) {
    Package pk;
    std::string err;
    if (!pk.load(path, &err)) {
        std::fprintf(stderr, "LOADFAIL %s\n", err.c_str());
        return 1;
    }
    std::vector<Prop> chain;
    for (int k = 0; k < pk.numExports(); k++) {
        const std::string nm = pk.exportName(k);
        int hdr = 0;
        int32_t end = 0;
        std::string tagName;
        V3 loc;
        bool hasLoc = false;
        if (pk.parseObject(k, &hdr, &chain, &end)) {
            if (const Prop* t = ue3::findProp(chain, "Tag"))
                if (t->type == "NameProperty") tagName = propName(pk, *t);
            if (const Prop* l = ue3::findProp(chain, "Location"))
                if (l->data.size() >= 12) {
                    std::memcpy(&loc, l->data.data(), 12);
                    hasLoc = true;
                }
        }
        bool hit = false;
        for (int a = 3; a < argc; a++) {
            if (nm.find(argv[a]) != std::string::npos ||
                tagName.find(argv[a]) != std::string::npos) {
                hit = true;
                break;
            }
        }
        if (!hit) continue;
        std::printf("#%d %s(%s) tag=\"%s\"", k, nm.c_str(), pk.exportClassName(k).c_str(),
                    tagName.c_str());
        if (hasLoc) std::printf(" loc=(%.0f,%.0f,%.0f)", loc.x, loc.y, loc.z);
        std::printf("\n");
    }
    return 0;
}

// chain: the raw property chain of an export, with names decoded and the
// native tail (typed constants such as a CompareInt's ValueB live there or
// as plain props) shown as bytes.
static int modeChain(const char* path, int argc, char** argv) {
    Package pk;
    std::string err;
    if (!pk.load(path, &err)) {
        std::fprintf(stderr, "LOADFAIL %s\n", err.c_str());
        return 1;
    }
    for (int a = 3; a < argc; a++) {
        const int k = atoi(argv[a]);
        if (k < 0 || k >= pk.numExports()) continue;
        std::printf("export#%d %s cls=%s size=%d\n", k, pk.exportName(k).c_str(),
                    pk.exportClassName(k).c_str(), pk.ex(k).serialSize);
        int hdr = 0;
        int32_t end = 0;
        std::vector<Prop> chain;
        if (!pk.parseObject(k, &hdr, &chain, &end)) {
            std::printf("  parse failed\n");
            continue;
        }
        std::printf("  headerLen=%d chainEnd=%d\n", hdr, end);
        for (const Prop& p : chain) {
            std::printf("  prop %s type=%s size=%d", p.name.c_str(), p.type.c_str(), p.size);
            if (p.data.size() == 12) {
                float f[3];
                std::memcpy(f, p.data.data(), 12);
                std::printf(" f3=(%g,%g,%g)", f[0], f[1], f[2]);
            }
            if (p.data.size() == 4) std::printf(" i=%d", p.i32());
            if (p.type == "NameProperty" && p.data.size() >= 4)
                std::printf(" name=\"%s\"", propName(pk, p).c_str());
            if (p.type == "StringProperty" && p.data.size() >= 4)
                std::printf(" str=\"%s\"", propStr(p).c_str());
            std::printf("\n");
        }
        // deep: link arrays with their scalar members
        for (const Prop& p : chain) {
            if (!p.isArray || p.structArray.empty()) continue;
            std::printf("  array %s (%d elems):\n", p.name.c_str(), p.arrayCount);
            for (size_t i = 0; i < p.structArray.size() && i < 4; i++) {
                for (const Prop& q : p.structArray[i]) {
                    std::printf("    [%zu] %s type=%s size=%d", i, q.name.c_str(),
                                q.type.c_str(), q.size);
                    if (q.data.size() == 4) std::printf(" i=%d", q.i32());
                    if (q.type == "NameProperty" && q.data.size() >= 4)
                        std::printf(" name=\"%s\"", propName(pk, q).c_str());
                    if (q.type == "StringProperty" && q.data.size() >= 4)
                        std::printf(" str=\"%s\"", propStr(q).c_str());
                    if (!q.intArray.empty()) {
                        std::printf(" ints=");
                        for (int32_t v : q.intArray) std::printf(" %d", v);
                    }
                    if (!q.structArray.empty()) {
                        std::printf(" sub=");
                        for (const Prop& r : q.structArray[0]) std::printf(" %s", r.name.c_str());
                    }
                    std::printf("\n");
                }
            }
        }
        const int32_t off = pk.ex(k).serialOffset, stop = off + pk.ex(k).serialSize;
        if (end > 0 && end < stop) {
            std::printf("  native(%d):", stop - end);
            for (int32_t b = end; b < stop && b < end + 48; b++)
                std::printf(" %02x", pk.bytes()[b]);
            std::printf("\n");
            if (stop - end >= 12) {
                float f[3];
                std::memcpy(f, pk.bytes() + end, 12);
                std::printf("  as f3=(%g,%g,%g)\n", f[0], f[1], f[2]);
            }
        }
    }
    return 0;
}

// ------------------------------------------------------------------- main --

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr,
                     "usage: %s <kismet|ladder|vars|find|chain> <map> [args]\n"
                     "  find:  %s find <map> <substr>...\n"
                     "  chain: %s chain <map> <exportIdx>...\n",
                     argv[0], argv[0], argv[0]);
        return 2;
    }
    const std::string mode = argv[1];
    if (mode == "kismet") return modeKismet(argv[2]);
    if (mode == "ladder") return modeLadder(argv[2]);
    if (mode == "vars") return modeVars(argv[2]);
    if (mode == "find") return modeFind(argv[2], argc, argv);
    if (mode == "chain") return modeChain(argv[2], argc, argv);
    std::fprintf(stderr, "unknown mode: %s\n", mode.c_str());
    return 2;
}
