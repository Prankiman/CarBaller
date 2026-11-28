#!/usr/bin/env bash
# Configure and build carballer on Linux (and other Unix-like systems).
#
#   ./tools/build_linux.sh                 # RelWithDebInfo build in ./build
#   ./tools/build_linux.sh --release       # optimized build
#   ./tools/build_linux.sh --test          # build, then run the headless self-test
#   ./tools/build_linux.sh --run           # build, then launch the game
#
# Dependencies (Debian / Ubuntu):
#   sudo apt install build-essential cmake ninja-build pkg-config \
#                    libsdl2-dev libglew-dev curl
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${CARBALLER_BUILD_DIR:-$ROOT/build}"
BUILD_TYPE="${CARBALLER_BUILD_TYPE:-RelWithDebInfo}"
JOBS="$(nproc 2>/dev/null || echo 4)"
DO_RUN=0
DO_TEST=0
CLEAN=0

usage() {
    cat <<'EOF'
Usage: tools/build_linux.sh [options]

Options:
  --release            Build with CMAKE_BUILD_TYPE=Release (default: RelWithDebInfo)
  --debug              Build with CMAKE_BUILD_TYPE=Debug
  --build-type <type>  Use an explicit CMAKE_BUILD_TYPE
  -j <n>, --jobs <n>   Parallel build jobs (default: nproc)
  --clean              Delete the build directory before configuring
  --test               Run the CARBALLER_SELFTEST after building
  --run                Launch the game after building
  -h, --help           Show this help

Environment:
  CARBALLER_BUILD_DIR   Build directory (default: <repo>/build)
  CARBALLER_BUILD_TYPE  Default CMAKE_BUILD_TYPE
EOF
}

die() { echo "error: $*" >&2; exit 1; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        --release)   BUILD_TYPE=Release ;;
        --debug)     BUILD_TYPE=Debug ;;
        --build-type)
            [[ $# -ge 2 ]] || die "--build-type needs a value"
            BUILD_TYPE="$2"; shift ;;
        -j|--jobs)
            [[ $# -ge 2 ]] || die "$1 needs a value"
            JOBS="$2"; shift ;;
        -j*)         JOBS="${1#-j}" ;;
        --clean)     CLEAN=1 ;;
        --test)      DO_TEST=1 ;;
        --run)       DO_RUN=1 ;;
        -h|--help)   usage; exit 0 ;;
        *)           echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

[[ "$JOBS" =~ ^[0-9]+$ ]] && (( JOBS > 0 )) || die "invalid job count: $JOBS"

command -v cmake >/dev/null 2>&1 ||
    die "cmake not found - install it (sudo apt install cmake)"

# pkg-config is how CMake finds SDL2/GLEW on Linux; without the dev packages
# the configure step fails, so check early with a helpful message.
missing=()
if command -v pkg-config >/dev/null 2>&1; then
    pkg-config --exists sdl2 || missing+=("libsdl2-dev")
    pkg-config --exists glew || missing+=("libglew-dev")
else
    missing+=("pkg-config")
fi
if (( ${#missing[@]} )); then
    die "missing build dependencies: ${missing[*]}
  Debian/Ubuntu: sudo apt install ${missing[*]}
  Arch:          sudo pacman -S sdl2 glew pkgconf
  Fedora:        sudo dnf install SDL2-devel glew-devel pkgconf"
fi

# RocketSim is a submodule; a tarball checkout has no .git but already ships it.
if [[ ! -f "$ROOT/RocketSim/CMakeLists.txt" ]]; then
    command -v git >/dev/null 2>&1 ||
        die "RocketSim submodule is missing and git is not installed"
    echo "==> initialising submodules"
    git -C "$ROOT" submodule update --init --recursive
fi

# Arena collision meshes are fetched, not tracked. The game refuses to start
# without them, so fetch before building.
need_assets=0
for i in $(seq 0 15); do
    [[ -s "$ROOT/assets/collision_meshes/soccar/mesh_${i}.cmf" ]] || need_assets=1
done
if (( need_assets )); then
    command -v curl >/dev/null 2>&1 ||
        die "arena collision meshes are missing and curl is not installed"
    echo "==> fetching arena collision meshes"
    "$ROOT/tools/fetch_assets.sh"
fi

if (( CLEAN )); then
    echo "==> removing $BUILD_DIR"
    rm -rf "$BUILD_DIR"
fi

generator=()
# Only choose a generator for a fresh build directory; an existing cache keeps
# whatever generator it was configured with.
if [[ ! -f "$BUILD_DIR/CMakeCache.txt" ]] && command -v ninja >/dev/null 2>&1; then
    generator=(-G Ninja)
fi

echo "==> configuring ($BUILD_TYPE) in $BUILD_DIR"
cmake -S "$ROOT" -B "$BUILD_DIR" "${generator[@]}" \
      -DCMAKE_BUILD_TYPE="$BUILD_TYPE"

echo "==> building with $JOBS job(s)"
cmake --build "$BUILD_DIR" --parallel "$JOBS"

BIN="$BUILD_DIR/carballer"
[[ -x "$BIN" ]] || die "build finished but $BIN is missing"

if (( DO_TEST )); then
    echo "==> self-test"
    if [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]; then
        CARBALLER_SELFTEST=1 "$BIN"
    elif command -v xvfb-run >/dev/null 2>&1; then
        CARBALLER_SELFTEST=1 xvfb-run -a "$BIN"
    else
        die "--test needs a display or xvfb-run (sudo apt install xvfb)"
    fi
    echo "==> self-test passed"
fi

echo "OK: $BIN"

if (( DO_RUN )); then
    # Run from the build directory so assets/ and settings.json resolve there.
    cd "$BUILD_DIR"
    exec ./carballer
fi
