#pragma once
// Entry point of the map loader: turns a UE3 .udk/.upk package into
// MapData (render shapes + placements + collision channels + sessions).

#include <string>

#include "map_data.h"

namespace maploader {

// Parses `path` into `out`. On failure returns false and writes a short
// human-readable reason into `err` (never empty).
bool loadMap(const std::string& path, MapData* out, std::string* err);

// Multi-line text report (used by CARBALLER_DUMP_MAP for headless validation).
std::string describeMap(const MapData& m);

}  // namespace maploader
