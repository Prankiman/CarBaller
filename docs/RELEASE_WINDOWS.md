# carballer on Windows — release notes

## What's in this package

```
carballer-<version>-windows-x86_64/
  carballer.bat         launcher (recommended: fixes the working directory)
  carballer.exe         the game
  SDL2.dll, glew32.dll  runtime libraries (see "Runtime DLLs" below)
  assets/               arena collision meshes, car models, sounds
  README-WINDOWS.md     this file
  README.md             full project readme (features, controls, build docs)
  LICENSE               MIT
```

Everything needed to play is included — no Rocket League install, no network.

## Requirements

- **Windows 10 or 11, 64-bit**
- **A GPU with an OpenGL 3.3 driver.** Windows itself only ships a basic
  software adapter (`opengl32.dll` exists either way, but it exposes OpenGL
  1.1), so on a machine without graphics drivers — or over some remote-desktop
  sessions — the game starts and immediately fails to create a GL context.
  Update the NVIDIA / AMD / Intel driver rather than the game.
- **A writable install folder.** `settings.json` and `imgui.ini` are written
  next to `carballer.exe` on exit, so extract to somewhere you own
  (`Documents`, `C:\Games`, …) — **not** `C:\Program Files`, where the write
  is silently lost on exit.

No administrator rights are required.

## Install

1. Extract the zip (keep the folder intact — `assets\` must stay beside
   `carballer.exe`).
2. If Windows warns about an unsigned download, that is expected for a first
   release: **More info → Run anyway**, or right-click the zip → Properties →
   **Unblock** before extracting.
3. Start it with `carballer.bat` (or double-click `carballer.exe` — Explorer
   sets the working directory correctly; a terminal or shortcut may not).

To run it from anywhere, `carballer.bat` does the equivalent of:

```bat
@echo off
cd /d "%~dp0"
start "" "%~dp0carballer.exe" %*
```

`cd /d "%~dp0"` is the important line: the game loads `assets\` relative to
its working directory, so launched from elsewhere it exits with
`[sim] collision meshes not found`.

## Runtime DLLs

The zip must contain every DLL `carballer.exe` links against, sitting **in the
same folder as the exe** (Windows searches the executable's directory first).
How they get there depends on how the build was produced:

| Build | DLLs to ship | How to list them |
| --- | --- | --- |
| MSYS2 (MINGW64) | `SDL2.dll`, `libglew-*.dll`, plus the MinGW runtime: `libstdc++-6.dll`, `libgcc_s_seh-1.dll`, `libwinpthread-1.dll` | `ldd carballer.exe` in the MINGW64 shell; copy from `/mingw64/bin` |
| Visual Studio + vcpkg | `SDL2.dll`, `glew32.dll` (vcpkg stages these next to the exe automatically) | `dumpbin /dependents carballer.exe` |

The MSVC build also needs the **Visual C++ Redistributable**
(`vcruntime140.dll`, `msvcp140.dll`). Windows 10/11 usually has it; if the
game complains about a missing `VCRUNTIME140.dll`, install the x64
redistributable from Microsoft rather than hunting for the DLL.

A quick sanity check before uploading: extract the zip into an empty folder on
a machine that has never seen the sources, and start it from there.

## Troubleshooting

- **`The code execution cannot proceed because SDL2.dll was missing`** — the
  DLLs were not included in the zip; see the table above.
- **`GL 3.3` / "failed to create GL context"** — GPU driver problem, not the
  game; see Requirements.
- **`[sim] collision meshes not found`** — `assets\` is missing or the exe was
  started with a different working directory; use `carballer.bat`.
- **Check it runs at all** — scripted self-test, exits 0 on success:

  ```powershell
  $env:CARBALLER_SELFTEST = "1"; .\carballer.exe
  ```

  ```bat
  set CARBALLER_SELFTEST=1 && carballer.exe
  ```

- **Controls** — keyboard/mouse and XInput gamepads both work; every binding,
  including each stick direction, is rebindable in **Settings → Bindings**.

## Verify the download

```powershell
Get-FileHash -Algorithm SHA256 .\carballer-<version>-windows-x86_64.zip
```

Compare against the `.sha256` published next to the release asset.
