#pragma once
// UStaticMesh reader for vanilla UDK-era UE3 (ArVer 867/868).
// Field order follows umodel (gildor2/UEViewer) UnMesh3.cpp:
//   UStaticMesh3::Serialize -> FStaticMeshLODModel3::Serialize.
// The same layout was validated against r3.udk / Dribble2 / LethIceRings.

#include <cstdint>
#include <string>
#include <vector>

#include "map_data.h"
#include "ue3_package.h"

namespace ue3 {

struct MeshSection {
    int32_t materialRef = 0;  // object ref (>0 export, <0 import)
    int32_t firstIndex = 0;   // index into the triangle index buffer
    int32_t numFaces = 0;
};

struct StaticMeshParse {
    bool ok = false;
    std::string err;

    V3 origin, boxExtent;
    float sphereRadius = 0.f;
    int32_t bodySetupRef = 0;
    int32_t kdopNodeCount = 0;
    int32_t kdopTriCount = 0;
    int32_t numLods = 0;
    int32_t headerLen = 0;
    int32_t bytesLeft = 0;

    // Static collision designation: false when the real game renders this
    // mesh but never blocks (goal sheets, sky sphere, signs).
    bool rigidBody = true;

    // LOD0
    std::vector<V3> pos;    // one vertex position each
    std::vector<float> uv;  // 2 floats per vertex (texcoord set 0)
    std::vector<uint32_t> idx;
    std::vector<MeshSection> sections;

    V3 boundsMin, boundsMax;
    bool hasBounds = false;
};

// Parses the static mesh referenced by `exportIdx` (class must be StaticMesh).
bool parseStaticMesh(const Package& pk, int32_t exportIdx, StaticMeshParse* out,
                     std::string* err);

}  // namespace ue3
