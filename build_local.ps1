param(
    [switch]$Clean,
    [switch]$SkipTests,
    [switch]$Dist
)

$ErrorActionPreference = "Stop"

$DkpRoot = "D:\DevEnv\devkitPro"
$MsysBash = Join-Path $DkpRoot "msys2\usr\bin\bash.exe"
$RepoWin = Split-Path -Parent $MyInvocation.MyCommand.Path

if (-not (Test-Path $MsysBash)) {
    throw "MSYS2 bash not found at $MsysBash"
}

$drive = $RepoWin.Substring(0, 1).ToLower()
$rest = ($RepoWin.Substring(2) -replace "\\", "/")
$RepoUnix = "/$drive$rest"
$DkpUnix = "/d/DevEnv/devkitPro"

$extraPaths = @("/usr/bin", "$DkpUnix/tools/bin", "$DkpUnix/devkitA64/bin")
$gitExe = Get-Command git -ErrorAction SilentlyContinue
if ($gitExe) {
    $gitDir = Split-Path -Parent $gitExe.Source
    $gd = $gitDir.Substring(0, 1).ToLower()
    $gr = ($gitDir.Substring(2) -replace "\\", "/")
    $extraPaths += "/$gd$gr"
}
$pathPrefix = $extraPaths -join ":"

$env:MSYSTEM = "MSYS"
$env:CHERE_INVOKING = "1"
$env:LC_ALL = "C"
$env:DEVKITPRO = $DkpUnix
$env:DEVKITARM = "$DkpUnix/devkitARM"
$env:DEVKITA64 = "$DkpUnix/devkitA64"
$env:PORTLIBS_PATH = "$DkpUnix/portlibs"

$cmds = @(
    "set -e",
    "export PATH='${pathPrefix}:'`"`$PATH`"",
    "export LC_ALL=C",
    "cd '$RepoUnix'",
    "git config --global --add safe.directory '*' >/dev/null 2>&1 || true"
)

if (-not $SkipTests) {
    $cmds += "echo '=== Running host regression tests ==='"
    $cmds += "make -C tests test BUILD_DIR=/tmp/missioncontrol-tests"
}

if ($Clean) {
    $cmds += "echo '=== Cleaning previous build ==='"
    $cmds += "make clean"
}

$jobs = [Environment]::ProcessorCount
if ($Dist) {
    $cmds += "echo '=== Building MissionControl dist ==='"
    $cmds += "make -j$jobs dist"
} else {
    $cmds += "echo '=== Building MissionControl exefs.nsp ==='"
    $cmds += "make -j$jobs"
}

$fullCmd = $cmds -join " && "
& $MsysBash -l -c $fullCmd
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

$outNsp = Join-Path $RepoWin "mc_mitm\out\nintendo_nx_arm64_armv8a\release\mc_mitm.nsp"
if (Test-Path $outNsp) {
    $info = Get-Item $outNsp
    $sha = (Get-FileHash $outNsp -Algorithm SHA256).Hash.ToLower()
    Write-Host "Build succeeded: $($info.FullName) ($($info.Length) bytes, sha256=$sha)"
}
