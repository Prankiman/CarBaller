# Configure and build carballer on Windows with Visual Studio + vcpkg.
#
#   powershell -ExecutionPolicy Bypass -File tools\build_windows.ps1
#   powershell -ExecutionPolicy Bypass -File tools\build_windows.ps1 -Run
#
# Requires: Visual Studio 2022 (Desktop C++), CMake, and vcpkg with
# SDL2 + GLEW. The script installs the vcpkg packages if they are missing.
#
# Double-clicking tools\build_windows.bat is equivalent to running this with
# the default options.

[CmdletBinding()]
param(
    [ValidateSet('RelWithDebInfo', 'Release', 'Debug', 'MinSizeRel')]
    [string]$Configuration = 'Release',

    [string]$BuildDir = '',

    [ValidateRange(0, 256)]
    [int]$Jobs = 0,

    [switch]$Clean,
    [switch]$Run,
    [switch]$Test,
    [switch]$SkipDeps,

    [string]$VcpkgRoot = ''
)

$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $PSScriptRoot

if (-not $BuildDir) {
    $BuildDir = Join-Path $Root 'build'
}
$BuildDir = [System.IO.Path]::GetFullPath($BuildDir)

function Write-Step([string]$Message) {
    Write-Host "==> $Message" -ForegroundColor Cyan
}

function Find-Command([string]$Name) {
    $cmd = Get-Command $Name -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    return $null
}

function Resolve-VcpkgRoot {
    if ($VcpkgRoot) { return $VcpkgRoot }
    if ($env:VCPKG_ROOT) { return $env:VCPKG_ROOT }
    if ($env:VCPKG_INSTALLATION_ROOT) { return $env:VCPKG_INSTALLATION_ROOT }

    $exe = Find-Command 'vcpkg'
    if ($exe) {
        # vcpkg.exe normally sits in the repository root.
        return Split-Path -Parent $exe
    }
    return $null
}

function Get-CarballerAssets {
    $dest = Join-Path $Root 'assets/collision_meshes/soccar'
    $missing = @()
    for ($i = 0; $i -le 15; $i++) {
        $file = Join-Path $dest "mesh_$i.cmf"
        if (-not (Test-Path $file) -or (Get-Item $file).Length -le 8) {
            $missing += $i
        }
    }
    if ($missing.Count -eq 0) { return }

    Write-Step 'fetching arena collision meshes'

    # Git Bash ships curl + GNU stat, which fetch_assets.sh relies on.
    $bash = Find-Command 'bash'
    if ($bash) {
        & $bash (Join-Path $Root 'tools/fetch_assets.sh')
        if ($LASTEXITCODE -ne 0) {
            throw 'tools/fetch_assets.sh failed'
        }
        return
    }

    # Native PowerShell fallback: same public mirrors as fetch_assets.sh.
    $sources = @(
        'https://raw.githubusercontent.com/pjsny/RocketSimPy/main/collision_meshes/soccar',
        'https://raw.githubusercontent.com/JeffA233/rlgym-sim-rs/main/collision_meshes/soccar',
        'https://raw.githubusercontent.com/lucas-emery/rocket-league-gym/main/rlgym/rocket_league/sim/collision_meshes/soccar'
    )
    New-Item -ItemType Directory -Force -Path $dest | Out-Null

    foreach ($i in $missing) {
        $out = Join-Path $dest "mesh_$i.cmf"
        $tmp = "$out.tmp"
        $ok = $false
        foreach ($base in $sources) {
            try {
                Invoke-WebRequest -UseBasicParsing -Uri "$base/mesh_$i.cmf" -OutFile $tmp
                $size = (Get-Item $tmp).Length
                if ((($size - 8) % 12) -eq 0 -and $size -gt 8) {
                    Move-Item -Force $tmp $out
                    $ok = $true
                    break
                }
            } catch {
                # Try the next mirror.
            } finally {
                if (Test-Path $tmp) { Remove-Item -Force $tmp -ErrorAction SilentlyContinue }
            }
        }
        if (-not $ok) {
            throw "failed to download mesh_$i.cmf (no working mirror)"
        }
    }
    Write-Host 'OK: 16 soccar collision meshes'
}

$cmake = Find-Command 'cmake'
if (-not $cmake) {
    throw 'cmake not found - install CMake and make sure it is on PATH'
}

# RocketSim is a submodule; a source zip already ships it.
if (-not (Test-Path (Join-Path $Root 'RocketSim/CMakeLists.txt'))) {
    $git = Find-Command 'git'
    if (-not $git) { throw 'RocketSim submodule is missing and git is not installed' }
    Write-Step 'initialising submodules'
    & $git -C $Root submodule update --init --recursive
    if ($LASTEXITCODE -ne 0) { throw 'git submodule update failed' }
}

Get-CarballerAssets

$vcpkg = Resolve-VcpkgRoot
if (-not $vcpkg) {
    throw @'
vcpkg was not found.

Install vcpkg (https://vcpkg.io) and either:
  - set VCPKG_ROOT to its directory, or
  - put vcpkg.exe on PATH.

Then install the dependencies:
  vcpkg install sdl2:x64-windows glew:x64-windows

Alternatively, build from an MSYS2 MINGW64 shell with tools/build_linux.sh.
'@
}

$vcpkgExe = Join-Path $vcpkg 'vcpkg.exe'
if (-not (Test-Path $vcpkgExe)) {
    $vcpkgExe = Find-Command 'vcpkg'
    if (-not $vcpkgExe) { throw "vcpkg.exe not found under $vcpkg" }
}

$installed = Join-Path $vcpkg 'installed/x64-windows'
$haveSdl2 = Test-Path (Join-Path $installed 'share/sdl2')
$haveGlew = Test-Path (Join-Path $installed 'share/glew')
if (-not $haveSdl2 -or -not $haveGlew) {
    if ($SkipDeps) {
        Write-Warning 'SDL2/GLEW are not installed in vcpkg (continuing because -SkipDeps was given)'
    } else {
        Write-Step 'installing vcpkg dependencies (sdl2, glew)'
        & $vcpkgExe install 'sdl2:x64-windows' 'glew:x64-windows'
        if ($LASTEXITCODE -ne 0) { throw 'vcpkg install failed' }
    }
}

if ($Clean -and (Test-Path $BuildDir)) {
    Write-Step "removing $BuildDir"
    Remove-Item -Recurse -Force $BuildDir
}

$toolchain = Join-Path $vcpkg 'scripts/buildsystems/vcpkg.cmake'
if (-not (Test-Path $toolchain)) {
    throw "vcpkg toolchain file not found: $toolchain"
}

Write-Step "configuring ($Configuration) in $BuildDir"
& $cmake -S $Root -B $BuildDir "-DCMAKE_TOOLCHAIN_FILE=$toolchain"
if ($LASTEXITCODE -ne 0) { throw 'CMake configure failed' }

$buildArgs = @('--build', $BuildDir, '--config', $Configuration)
if ($Jobs -gt 0) {
    $buildArgs += @('--parallel', "$Jobs")
} else {
    $buildArgs += '--parallel'
}
Write-Step 'building'
& $cmake @buildArgs
if ($LASTEXITCODE -ne 0) { throw 'CMake build failed' }

$exe = Join-Path $BuildDir "carballer.exe"
if (-not (Test-Path $exe)) {
    $found = Get-ChildItem -Path $BuildDir -Recurse -Filter 'carballer.exe' -File |
             Select-Object -First 1
    if ($found) { $exe = $found.FullName }
}
if (-not (Test-Path $exe)) { throw "build finished but carballer.exe was not found" }

$exeDir = Split-Path -Parent $exe

if ($Test) {
    Write-Step 'self-test'
    $old = $env:CARBALLER_SELFTEST
    try {
        $env:CARBALLER_SELFTEST = '1'
        Push-Location $exeDir
        & $exe
        if ($LASTEXITCODE -ne 0) { throw "self-test failed (exit $LASTEXITCODE)" }
    } finally {
        Pop-Location
        $env:CARBALLER_SELFTEST = $old
    }
    Write-Host 'OK: self-test passed' -ForegroundColor Green
}

Write-Host "OK: $exe" -ForegroundColor Green

if ($Run) {
    # Run from the build directory so assets\ and settings.json resolve there.
    Push-Location $exeDir
    try {
        & $exe
    } finally {
        Pop-Location
    }
}
