#pragma once
// Post-processing for the session (checkpoint) list built by the loader.
//
// Map actors are not placed with a car in mind: UE3 PathNodes are navigation
// points for walking AI, so on a map of floating platforms many of them hang
// in mid-air, sit inside a slab, or come in near-duplicate clusters. Spawning
// a car there drops it a kilometre, depenetrates it sideways, or makes the
// checkpoint list oscillate between two points it can never tell apart.
//
// sanitizeSessions() repairs that against the map's own collision geometry:
// every spawn is put on solid ground (or the point is dropped when there is
// none), duplicates collapse, the derived ones are re-ordered into a route,
// and each session gets a reach radius sized to how far apart the route runs.

#include <string>

namespace maploader {

struct MapData;

// Returns the number of sessions removed. `log` (optional) receives a short
// human-readable summary of what had to be moved.
int sanitizeSessions(MapData* m, std::string* log);

}  // namespace maploader
