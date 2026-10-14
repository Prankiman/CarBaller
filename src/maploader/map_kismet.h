#pragma once
// Course-section checkpoints from the map's Kismet. Workshop maps author
// their courses four ways; all of them number the sections, and the loader
// reads whichever scheme the package carries (no per-map special cases):
//
//  1. "SpawnLevel" sequences (Leth's rings): each Sequence binds a
//     "PlayerSpawn" variable link (the Teleport's car destination) and a
//     "BallSpawn" link (where AddGameBall drops the ball); a
//     SeqCond_CompareInt on the map's `Level` variable names the section.
//  2. Numbered Kismet variables (TJBrother's air dribble): SeqVar_Object
//     variables named "PlayerSpawnN"/"BallSpawnN" bound to editor-picked
//     PathNodes; N is the section.
//  3. Ladder teleports (r3): the CompareInt dispatcher (inline ValueB
//     constant) fires one SeqAct_Teleport per section; the Destination
//     actor is that section's start.
//  4. Tagged "PlayerSpawnNN"/"BallSpawnNN" PointInSpace actors (Dribble
//     Challenge) - those are read straight off the actors by the loader
//     and merged into the same numbering (see map_loader.cpp).
//
// Ball teleports (ops that move the spawned ball, whose handle only exists
// at runtime) still name where the ball goes; that position is attached to
// the section whose start it sits at. A map with none of these falls back
// to its PathNodes.

#include <vector>

#include "map_data.h"
#include "ue3_package.h"

namespace maploader {

// One checkpoint pair from the Kismet. `level` is the section constant (>= 1)
// the pair serves, or 0 when no dispatcher named it.
struct KismetCheckpoint {
    V3 car;
    float carYawRad = 0.f;
    bool hasCar = false;
    V3 ball;
    bool hasBall = false;
    int level = 0;
};

// Reads every Sequence with a PlayerSpawn/BallSpawn binding into `out`
// (package order), and assigns section numbers from the SeqCond_CompareInt
// dispatchers that route to them. Returns the number of checkpoints found;
// `indexed` (optional) receives how many got a section constant. Broken
// links are skipped, never fatal.
int kismetCheckpoints(const ue3::Package& pkg, std::vector<KismetCheckpoint>* out,
                      int* indexed = nullptr);

}  // namespace maploader
