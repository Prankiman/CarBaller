#pragma once
// Level BSP (UModel + UModelComponent) decoding.
//
// A cooked UE3 map renders its brush model through UModelComponent exports
// instead of StaticMeshComponents; maps that use BSP for part of their
// geometry (DribbleChallenge2 does for its course) would otherwise show holes.
// Brush *volumes* (TriggerVolume/BlockingVolume) have no render component and
// are handled elsewhere, so only the level's own Model is decoded here.

#include <cstdint>
#include <string>
#include <vector>

#include "map_data.h"
#include "ue3_package.h"

namespace ue3 {

// One group of BSP faces sharing a material (and therefore a colour slot).
// Positions are already in world space - the level BSP has no transform.
struct BspPiece {
    int32_t materialRef = 0;  // object ref of FBspSurf.Material (colour only)
    std::vector<V3> verts;
    std::vector<uint32_t> tris;
};

// Best-effort: returns false (empty `out`, reason in `err`) when the package
// has no BSP render components or when the native payload does not match the
// layout this decoder expects. Callers treat BSP as optional geometry.
bool readLevelBsp(const Package& pk, std::vector<BspPiece>* out, std::string* err);

}  // namespace ue3
