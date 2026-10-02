#!/usr/bin/env bash
# Assembles a distributable Linux tarball for carballer.
#
#   cmake -B build-release -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-release -j"$(nproc)"
#   ./tools/package_linux.sh                 # or: ./tools/package_linux.sh <build-dir>
#
# Produces dist/carballer-<version>-linux-<arch>.tar.gz and its .sha256.
# Set CARBALLER_VERSION to override the default version.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${1:-$ROOT/build-release}"
VERSION="${CARBALLER_VERSION:-0.1.0}"
ARCH="$(uname -m)"
NAME="carballer-${VERSION}-linux-${ARCH}"
DIST="$ROOT/dist"
STAGE="$DIST/$NAME"

die() { echo "error: $*" >&2; exit 1; }

[[ -x "$BUILD_DIR/carballer" ]] || die "no executable at $BUILD_DIR/carballer - build first:
  cmake -B build-release -DCMAKE_BUILD_TYPE=Release
  cmake --build build-release -j\"\$(nproc)\""

# Arena collision meshes are gitignored (fetched by tools/fetch_assets.sh) and
# the game never downloads them at runtime, so a tarball without them ships a
# game that refuses to start. Fail loudly instead.
missing=0
for i in $(seq 0 15); do
  [[ -s "$ROOT/assets/collision_meshes/soccar/mesh_${i}.cmf" ]] || missing=$((missing + 1))
done
(( missing == 0 )) || die "$missing/16 arena collision meshes missing - run ./tools/fetch_assets.sh"

echo "==> staging ${NAME}"
rm -rf "$STAGE"
mkdir -p "$STAGE/bin"

# Release builds still keep local symbols; strip them for distribution.
install -m 0755 "$BUILD_DIR/carballer" "$STAGE/bin/carballer"
strip --strip-unneeded "$STAGE/bin/carballer"

# Launcher: the game resolves assets - and writes settings.json / imgui.ini -
# relative to the working directory, so always start it from the package root.
# A bare ./bin/carballer from $HOME would find nothing and exit with
# "[sim] collision meshes not found".
cat > "$STAGE/carballer" <<'EOF'
#!/usr/bin/env bash
# carballer launcher: start the game from the package root so assets/ resolves
# no matter where this was invoked from (terminal, file manager, desktop entry).
set -euo pipefail
cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
exec ./bin/carballer "$@"
EOF
chmod 0755 "$STAGE/carballer"

echo "==> copying assets and docs"
cp -a "$ROOT/assets" "$STAGE/assets"
if [[ -d "$ROOT/docs/screenshots" ]]; then
  mkdir -p "$STAGE/docs"
  cp -a "$ROOT/docs/screenshots" "$STAGE/docs/screenshots"
fi
if [[ -f "$ROOT/docs/RELEASE_LINUX.md" ]]; then
  cp "$ROOT/docs/RELEASE_LINUX.md" "$STAGE/README-LINUX.md"
fi
cp "$ROOT/README.md" "$ROOT/LICENSE" "$STAGE/"

echo "==> verifying staged assets match the working copy"
diff -r "$ROOT/assets" "$STAGE/assets" > /dev/null ||
  die "staged assets differ from the working tree"

echo "==> packaging"
tar -C "$DIST" -czf "$DIST/${NAME}.tar.gz" "$NAME"
( cd "$DIST" && sha256sum "${NAME}.tar.gz" > "${NAME}.tar.gz.sha256" )

echo "OK: $DIST/${NAME}.tar.gz"
ls -lh "$DIST/${NAME}.tar.gz"
cat "$DIST/${NAME}.tar.gz.sha256"
