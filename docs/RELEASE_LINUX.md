# carballer on Linux — release notes

## What's in this package

```
carballer-<version>-linux-x86_64/
  carballer           launcher — run this
  bin/carballer       the game binary
  assets/             arena collision meshes, car models, sounds
  docs/screenshots/   images used by README.md
  README-LINUX.md     this file
  README.md           full project readme (features, controls, build docs)
  LICENSE             MIT
```

Everything the game needs at runtime is included — no network access, no
`curl`, no game install.

## Requirements

- OpenGL 3.3-capable GPU (Mesa or a proprietary driver)
- X11, or Wayland through XWayland
- Shared libraries: SDL2, GLEW, OpenGL + GLU, X11, libstdc++

### Exact versions this build links against

Built on Arch Linux (October 2026), so the binary asks for:

| Symbol / library | Minimum |
| --- | --- |
| glibc | **2.43** |
| libstdc++ | GLIBCXX_3.4.32 (GCC 13) |
| GLEW | **2.3** (`libGLEW.so.2.3`) |
| SDL2 | `libSDL2-2.0.so.0` (any 2.x) |
| OpenGL | `libGL.so.1`, `libGLU.so.1` |

Install the libraries:

```bash
# Arch / Manjaro
sudo pacman -S sdl2 glew

# Fedora
sudo dnf install SDL2 glew mesa-libGL mesa-libGLU

# Ubuntu / Debian
sudo apt install libsdl2-2.0-0 libglew2.2 libgl1 libglu1-mesa
```

`ldd bin/carballer` prints the exact list if anything is missing.

**Compatibility note.** glibc 2.43 is recent, so this build targets rolling
releases (Arch, openSUSE Tumbleweed, Fedora 44+, Ubuntu 26.04+, Debian sid).
On an older distribution the game fails immediately with:

```
./carballer: version `GLIBC_2.43' not found (required by ./carballer)
```

That is a build-toolchain limit, not a bug: rebuild the same sources with
`-DCMAKE_BUILD_TYPE=Release` inside a rootfs/chroot of your target
distribution (or run CI there) and repackage with `tools/package_linux.sh`.

## Running

```bash
./carballer
```

The launcher always starts the game from the package root, which is where
`settings.json` and `imgui.ini` are written on exit. Keep the package
directory writable, or run it from a copy you own.

## Troubleshooting

- **Check it runs at all** — a scripted self-test that exits 0 on success:

  ```bash
  CARBALLER_SELFTEST=1 ./carballer
  ```

- **`collision meshes not found`** — the package was extracted somewhere the
  launcher can't reach, or `assets/` was moved. Keep the tree intact and start
  the game via `./carballer`.
- **`error while loading shared libraries`** — install the packages listed
  above; the missing library's name is printed in the error.

## Verify the download

```bash
sha256sum -c carballer-<version>-linux-x86_64.tar.gz.sha256
```
