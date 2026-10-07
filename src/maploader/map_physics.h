#pragma once
// RocketSim/Bullet collision for a custom map: swaps the arena's built-in
// soccar collision for the map's collidable placements, and swaps it back.
//
// Deliberately independent of map_data.h's consumers: nothing here is needed
// by the parser or the renderer, and the handle type stays opaque so this
// header carries no Bullet includes.

#include <string>

#include "map_data.h"

namespace RocketSim {
class Arena;
}

namespace maploader {

// Owns every Bullet object created for one loaded map. Opaque on purpose:
// construct with applyMapCollision(), destroy with releaseMapCollision().
struct MapCollision;

// Replaces `arena`'s static collision with the collidable placements of
// `data`. Returns nullptr (and sets *err) when the map has no collidable
// geometry, leaving the arena untouched.
MapCollision* applyMapCollision(RocketSim::Arena* arena, const MapData& data, std::string* err);

// Strips the map collision, restores the arena's original collision and
// deletes `h`. Safe with either argument null; must run before the arena is
// destroyed while a map is loaded.
void releaseMapCollision(RocketSim::Arena* arena, MapCollision* h);

}  // namespace maploader
