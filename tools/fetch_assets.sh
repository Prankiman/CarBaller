#!/usr/bin/env bash
# Downloads Rocket League soccar arena collision meshes (.cmf) for RocketSim.
# These are publicly mirrored dumps of the RL arena collision geometry
# (floor/wall ramps, wall/ceiling fillets, corner ramps, goal interiors).
# RocketSim itself only supplies the flat floor/wall/ceiling planes.
set -euo pipefail

DEST="$(cd "$(dirname "$0")/.." && pwd)/assets/collision_meshes/soccar"
mkdir -p "$DEST"

# Public mirrors of the same 16-file soccar dump (tried in order).
SOURCES=(
  "https://raw.githubusercontent.com/pjsny/RocketSimPy/main/collision_meshes/soccar"
  "https://raw.githubusercontent.com/JeffA233/rlgym-sim-rs/main/collision_meshes/soccar"
  "https://raw.githubusercontent.com/lucas-emery/rocket-league-gym/main/rlgym/rocket_league/sim/collision_meshes/soccar"
)

for i in $(seq 0 15); do
  out="$DEST/mesh_$i.cmf"
  if [[ -s "$out" ]]; then
    continue
  fi
  ok=0
  for base in "${SOURCES[@]}"; do
    if curl -fsSL --retry 2 -o "$out.tmp" "$base/mesh_$i.cmf"; then
      # sanity: size must be 8 + 12*(tris+verts)
      sz=$(stat -c%s "$out.tmp")
      if (( (sz - 8) % 12 == 0 && sz > 8 )); then
        mv "$out.tmp" "$out"
        ok=1
        break
      fi
    fi
  done
  if (( ! ok )); then
    rm -f "$out.tmp"
    echo "FAILED: mesh_$i.cmf" >&2
    exit 1
  fi
done

echo "OK: 16 soccar collision meshes in $DEST"
