#include "map_kismet.h"

#include <cctype>
#include <cstring>
#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "ue3_level.h"

namespace maploader {
namespace {

using ue3::Package;
using ue3::Prop;

// StringProperty payload: [i32 byte length][bytes].
std::string propStr(const Prop& p) {
    if (p.data.size() < 4) return {};
    int32_t len;
    std::memcpy(&len, p.data.data(), 4);
    if (len <= 0 || (size_t)len + 4 > p.data.size()) return {};
    std::string s(reinterpret_cast<const char*>(p.data.data()) + 4, (size_t)len);
    while (!s.empty() && s.back() == '\0') s.pop_back();
    return s;
}

// NameProperty payload: [i32 name index][i32 instance] into the name table.
std::string propName(const Package& pk, const Prop& p) {
    if (p.type != "NameProperty" || p.data.size() < 4) return {};
    const int32_t idx = p.i32(0);
    const int32_t inst = p.data.size() >= 8 ? p.i32(4) : 0;
    if (idx < 0 || idx >= pk.nameCount()) return {};
    return pk.resolveFName(idx, inst);
}

// Object refs out of an array property: a flat int array (LinkedVariables)
// or per-element structs carrying "LinkedOp" (an output link's Links).
void arrayRefs(const Prop* p, std::vector<int32_t>* out) {
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

// The VariableLinks of one op as (LinkDesc -> linked variable refs) pairs.
void varLinks(const std::vector<Prop>& chain,
              std::vector<std::pair<std::string, std::vector<int32_t>>>* out) {
    const Prop* vl = ue3::findProp(chain, "VariableLinks");
    if (!vl) return;
    for (const std::vector<Prop>& link : vl->structArray) {
        const Prop* d = ue3::findProp(link, "LinkDesc");
        if (!d) continue;
        std::vector<int32_t> refs;
        arrayRefs(ue3::findProp(link, "LinkedVariables"), &refs);
        out->emplace_back(propStr(*d), std::move(refs));
    }
}

// The ops an op's output links feed, as export indices.
void outEdges(const Package& pkg, const std::vector<Prop>& chain, std::vector<int32_t>* out) {
    const Prop* ol = ue3::findProp(chain, "OutputLinks");
    if (!ol) return;
    for (const std::vector<Prop>& link : ol->structArray) {
        std::vector<int32_t> refs;
        arrayRefs(ue3::findProp(link, "Links"), &refs);
        for (int32_t v : refs)
            if (v > 0 && v <= pkg.numExports()) out->push_back(v - 1);
    }
}

// A PlayerSpawn/BallSpawn-style ref is either the spawn actor itself or a
// SeqVar_Object variable carrying it in ObjValue. Returns false when no
// Location can be read.
bool resolveSpawn(const Package& pkg, int32_t ref, V3* pos, float* yawRad) {
    if (ref <= 0 || ref > pkg.numExports()) return false;
    if (pkg.classNameOfRef(ref) == "SeqVar_Object") {
        std::vector<Prop> chain;
        int hdr = 0;
        int32_t end = 0;
        if (!pkg.parseObject(ref - 1, &hdr, &chain, &end)) return false;
        const Prop* ov = ue3::findProp(chain, "ObjValue");
        if (!ov) return false;
        ref = ov->i32();
        if (ref <= 0 || ref > pkg.numExports()) return false;
    }
    std::vector<Prop> chain;
    int hdr = 0;
    int32_t end = 0;
    if (!pkg.parseObject(ref - 1, &hdr, &chain, &end)) return false;
    const Prop* loc = ue3::findProp(chain, "Location");
    if (!loc) return false;
    float f[3];
    if (!loc->vec3(f)) return false;
    *pos = V3(f[0], f[1], f[2]);
    if (yawRad) {
        *yawRad = 0.f;
        const Prop* rot = ue3::findProp(chain, "Rotation");
        int32_t r[3];
        if (rot && rot->rot3(r)) *yawRad = (float)ue3::unitsToRad(r[1]);
    }
    return true;
}

// The section a SeqCond_CompareInt dispatches on: the inline constant
// (ValueB, UE3 stores typed constants natively) or a "B" variable link
// bound to a SeqVar_Int. 0 when the condition compares against a runtime
// value we cannot know.
int32_t condLevel(const Package& pkg, const std::vector<Prop>& chain) {
    if (const Prop* vb = ue3::findProp(chain, "ValueB"))
        if (vb->data.size() >= 4 && vb->i32() >= 1) return vb->i32();
    std::vector<std::pair<std::string, std::vector<int32_t>>> links;
    varLinks(chain, &links);
    for (const auto& [desc, refs] : links) {
        if (desc != "B") continue;
        for (int32_t r : refs) {
            if (r <= 0 || pkg.classNameOfRef(r) != "SeqVar_Int") continue;
            std::vector<Prop> vchain;
            int hdr = 0;
            int32_t end = 0;
            if (!pkg.parseObject(r - 1, &hdr, &vchain, &end)) continue;
            const Prop* iv = ue3::findProp(vchain, "IntValue");
            if (iv && iv->i32() >= 1) return iv->i32();
        }
    }
    return 0;
}

// "PlayerSpawn07" / "BallSpawn3" -> 7 / 3, -1 when the name is anything else.
int numberedName(const std::string& s, const char* prefix) {
    const size_t pl = std::strlen(prefix);
    if (s.size() <= pl || s.compare(0, pl, prefix) != 0) return -1;
    int n = 0;
    for (size_t i = pl; i < s.size(); i++) {
        if (s[i] < '0' || s[i] > '9') return -1;
        n = n * 10 + (s[i] - '0');
    }
    return n >= 1 && n <= 999 ? n : -1;
}

// True when a Teleport's Target is the spawned ball rather than the car or
// a piece of geometry: named "...ball..." variables count, and so does a
// runtime-filled object slot (the ball is stored in one after AddGameBall);
// an actor bound in the editor is a moving-platform reset, not a ball.
bool targetIsBallish(const Package& pkg, int32_t ref) {
    if (ref <= 0 || ref > pkg.numExports()) return false;
    const std::string cls = pkg.classNameOfRef(ref);
    if (cls == "SeqVar_Object") {
        std::vector<Prop> chain;
        int hdr = 0;
        int32_t end = 0;
        if (!pkg.parseObject(ref - 1, &hdr, &chain, &end)) return false;
        const Prop* ov = ue3::findProp(chain, "ObjValue");
        return !ov || ov->i32() <= 0;  // unfilled slot: the runtime ball
    }
    if (cls == "SeqVar_Named") {
        std::vector<Prop> chain;
        int hdr = 0;
        int32_t end = 0;
        if (!pkg.parseObject(ref - 1, &hdr, &chain, &end)) return false;
        const Prop* fn = ue3::findProp(chain, "FindVarName");
        std::string nm = fn ? propName(pkg, *fn) : "";
        for (char& c : nm) c = (char)std::tolower((unsigned char)c);
        return nm.find("ball") != std::string::npos;
    }
    return false;
}

}  // namespace

int kismetCheckpoints(const Package& pkg, std::vector<KismetCheckpoint>* out, int* indexed) {
    if (!out) return 0;
    out->clear();
    if (indexed) *indexed = 0;

    std::vector<Prop> chain;
    int hdr = 0;
    int32_t end = 0;

    // ---- pass 1: "SpawnLevel" sequences binding PlayerSpawn / BallSpawn.
    // The course/rings scheme: one Sequence per section, dispatched by a
    // SeqCond_CompareInt on the map's `Level` variable.
    std::unordered_set<int32_t> cpSeqs;             // sequences with a
    std::unordered_map<int32_t, size_t> cpSeqToIdx; //   PlayerSpawn link (+ idx)
    for (int32_t k = 0; k < pkg.numExports(); k++) {
        if (!pkg.exportIsA(k, "Sequence")) continue;
        if (!pkg.parseObject(k, &hdr, &chain, &end)) continue;
        std::vector<std::pair<std::string, std::vector<int32_t>>> links;
        varLinks(chain, &links);
        KismetCheckpoint cp;
        for (auto& [desc, refs] : links) {
            if (desc == "PlayerSpawn") {
                for (int32_t raw : refs)
                    if (raw > 0) {  // a bound variable names this a dispatch
                        cpSeqs.insert(k);  // target for the level indexer
                        break;
                    }
            }
            if (desc != "PlayerSpawn" && desc != "BallSpawn") continue;
            for (int32_t raw : refs) {
                if (raw <= 0) continue;  // import refs carry no location
                V3 pos;
                float yaw = 0.f;
                if (!resolveSpawn(pkg, raw, &pos, &yaw)) continue;
                if (desc == "PlayerSpawn") {
                    cp.car = pos;
                    cp.carYawRad = yaw;
                    cp.hasCar = true;
                } else {
                    cp.ball = pos;
                    cp.hasBall = true;
                }
                break;
            }
        }
        if (cp.hasCar) {
            cpSeqToIdx[k] = out->size();
            out->push_back(cp);
        }
    }

    // ---- pass 2: numbered Kismet variables. Some maps bind their spawn
    // series as SeqVar_Object variables named "PlayerSpawnN"/"BallSpawnN"
    // (editor-picked PathNodes) instead of tagging actors or wrapping them
    // in sequences; the number is the section. Runs after the level pass so
    // sequence checkpoints can claim their numbers first.
    auto numberedVars = [&]() {
    // Collect every numbered variable first: a map may export a section's
    // BallSpawn before its PlayerSpawn, and either may come without the
    // other.
    struct NumVar {
        V3 car, ball;
        float carYaw = 0.f;
        bool hasCar = false, hasBall = false;
    };
    std::map<int, NumVar> vars;
    for (int32_t k = 0; k < pkg.numExports(); k++) {
        if (!pkg.exportIsA(k, "SeqVar_Object")) continue;
        if (!pkg.parseObject(k, &hdr, &chain, &end)) continue;
        const Prop* vn = ue3::findProp(chain, "VarName");
        const std::string varName = vn ? propName(pkg, *vn) : "";
        if (varName.empty()) continue;
        int num = numberedName(varName, "PlayerSpawn");
        bool isCar = num >= 1;
        if (!isCar) {
            num = numberedName(varName, "BallSpawn");
            if (num < 1) continue;
        }
        const Prop* ov = ue3::findProp(chain, "ObjValue");
        if (!ov || ov->i32() <= 0) continue;
        V3 pos;
        float yaw = 0.f;
        if (!resolveSpawn(pkg, k + 1, &pos, &yaw)) continue;  // via ObjValue
        NumVar& nv = vars[num];
        if (isCar) {
            nv.car = pos;
            nv.carYaw = yaw;
            nv.hasCar = true;
        } else {
            nv.ball = pos;
            nv.hasBall = true;
        }
    }
    std::map<int, size_t> byLevel;
    for (size_t i = 0; i < out->size(); i++)
        if ((*out)[i].level >= 1) byLevel.emplace((*out)[i].level, i);
    for (const auto& [num, nv] : vars) {
        auto it = byLevel.find(num);
        if (it != byLevel.end()) {
            KismetCheckpoint& c = (*out)[it->second];
            if (nv.hasCar && !c.hasCar) {
                c.car = nv.car;
                c.carYawRad = nv.carYaw;
                c.hasCar = true;
            }
            if (nv.hasBall && !c.hasBall) {
                c.ball = nv.ball;
                c.hasBall = true;
            }
            continue;
        }
        if (!nv.hasCar) continue;  // a ball without its car is not a checkpoint
        KismetCheckpoint cp;
        cp.car = nv.car;
        cp.carYawRad = nv.carYaw;
        cp.hasCar = true;
        if (nv.hasBall) {
            cp.ball = nv.ball;
            cp.hasBall = true;
        }
        cp.level = num;
        byLevel[num] = out->size();
        out->push_back(cp);
    }
    };

    // ---- level pass: the dispatcher's section constants. Its output must
    // land on one of the sequences above for the number to apply; a level
    // whose output fires a Teleport instead is recorded as a ladder step
    // for pass 3.
    std::unordered_map<int32_t, int32_t> ladderConds;  // cond -> section
    {
        std::vector<int32_t> edges;
        for (int32_t k = 0; k < pkg.numExports(); k++) {
            if (!pkg.exportIsA(k, "SeqCond_CompareInt")) continue;
            if (!pkg.parseObject(k, &hdr, &chain, &end)) continue;
            edges.clear();
            outEdges(pkg, chain, &edges);
            const int32_t level = condLevel(pkg, chain);
            if (level < 1) continue;
            bool hits = false;
            for (int32_t t : edges)
                if (cpSeqs.count(t)) { hits = true; break; }
            if (hits) {
                for (int32_t t : edges) {
                    auto it = cpSeqToIdx.find(t);
                    if (it != cpSeqToIdx.end()) (*out)[it->second].level = level;
                }
            }
            for (int32_t t : edges)
                if (pkg.exportIsA(t, "SeqAct_Teleport")) {
                    ladderConds[k] = level;
                    break;
                }
        }
    }
    numberedVars();  // pass 2 (after levels are known)

    // ---- pass 3: ladder teleports. Maps without any of the above drive
    // their course straight from the dispatcher: each condition's output
    // fires a Teleport whose Destination actor is that section's start.
    // Only used when nothing richer was found (sequences/numbers carry the
    // ball as well).
    std::unordered_set<int32_t> ladderTels;
    if (out->empty()) {
        std::vector<int32_t> edges;
        for (const auto& [k, level] : ladderConds) {
            if (!pkg.parseObject(k, &hdr, &chain, &end)) continue;
            edges.clear();
            outEdges(pkg, chain, &edges);
            for (int32_t t : edges) {
                if (!pkg.exportIsA(t, "SeqAct_Teleport")) continue;
                if (!ladderTels.insert(t).second) continue;  // one section each
                if (!pkg.parseObject(t, &hdr, &chain, &end)) continue;
                std::vector<std::pair<std::string, std::vector<int32_t>>> links;
                varLinks(chain, &links);
                for (const auto& [desc, refs] : links) {
                    if (desc != "Destination") continue;
                    for (int32_t raw : refs) {
                        V3 pos;
                        float yaw = 0.f;
                        if (!resolveSpawn(pkg, raw, &pos, &yaw)) continue;
                        KismetCheckpoint cp;
                        cp.car = pos;
                        cp.carYawRad = yaw;
                        cp.hasCar = true;
                        cp.level = level;
                        out->push_back(cp);
                        break;
                    }
                    break;
                }
            }
        }
    }

    // ---- pass 4: ball teleports. Ops that move the spawned ball (its
    // handle only exists at runtime) still name where the ball goes; that
    // position belongs to the section whose start it sits at.
    {
        std::vector<int32_t> edges;
        for (int32_t k = 0; k < pkg.numExports(); k++) {
            if (!pkg.exportIsA(k, "SeqAct_Teleport")) continue;
            if (ladderTels.count(k)) continue;
            if (!pkg.parseObject(k, &hdr, &chain, &end)) continue;
            std::vector<std::pair<std::string, std::vector<int32_t>>> links;
            varLinks(chain, &links);
            int32_t targetRef = 0, destRef = 0;
            for (const auto& [desc, refs] : links) {
                if (desc == "Target" && !refs.empty()) targetRef = refs.front();
                if (desc == "Destination" && !refs.empty()) destRef = refs.front();
            }
            if (!targetIsBallish(pkg, targetRef) || destRef <= 0) continue;
            V3 pos;
            float yaw = 0.f;
            if (!resolveSpawn(pkg, destRef, &pos, &yaw)) continue;
            size_t best = out->size();
            float bestD = 3.4e38f;
            for (size_t i = 0; i < out->size(); i++) {
                if ((*out)[i].hasBall) continue;
                const float d = ((*out)[i].car - pos).len2d();
                if (d < bestD) {
                    bestD = d;
                    best = i;
                }
            }
            if (best < out->size()) {
                (*out)[best].ball = pos;
                (*out)[best].hasBall = true;
            }
        }
    }

    if (indexed) {
        *indexed = 0;
        for (const KismetCheckpoint& c : *out)
            if (c.level > 0) (*indexed)++;
    }
    return (int)out->size();
}

}  // namespace maploader
