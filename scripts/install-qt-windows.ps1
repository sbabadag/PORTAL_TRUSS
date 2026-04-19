#Requires -Version 5.1
<#
.SYNOPSIS
  Installs Qt 6 Desktop (qtbase) for PortalSolver via aqtinstall.

.PARAMETER QtVersion
  e.g. 6.8.3 — check https://download.qt.io/online/qtsdkrepository/windows_x86/desktop/

.PARAMETER Arch
  Must match your compiler. For Qt 6.8.x on Windows, aqt often only offers
  win64_msvc2022_64 (not MSVC 2019). Run: python -m aqt list-qt windows desktop --arch <version>

.PARAMETER OutputDir
  Root folder; Qt is placed under OutputDir\6.x.x\<arch_suffix>\
#>
param(
    [string]$QtVersion = "6.8.3",
    [ValidateSet("win64_msvc2019_64", "win64_msvc2022_64", "win64_mingw", "win64_llvm_mingw")]
    [string]$Arch = "win64_msvc2022_64",
    [string]$OutputDir = ""
)

$ErrorActionPreference = "Stop"

if (-not $OutputDir) {
    $OutputDir = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot "..\qt-sdk"))
}

Write-Host "Installing Qt $QtVersion ($Arch) into $OutputDir ..."

python -m pip install --upgrade pip aqtinstall

# No "-m": the default desktop install already includes qtbase (Core, Gui, Widgets, Concurrent).
# "-m" is only for *extra* archives (qtcharts, qtmultimedia, …), not qtbase — using "-m qtbase"
# breaks aqt (unknown package names in the SDK XML).
$aqtArgs = @(
    "install-qt", "windows", "desktop", $QtVersion, $Arch,
    "-O", $OutputDir
)
python -m aqt @aqtArgs

$sub = if ($Arch -match "msvc2019") { "msvc2019_64" }
       elseif ($Arch -match "msvc2022") { "msvc2022_64" }
       elseif ($Arch -match "llvm_mingw") { "llvm_mingw_64" }
       elseif ($Arch -match "mingw") { "mingw_64" } else { "msvc2022_64" }

$prefix = Join-Path (Join-Path $OutputDir $QtVersion) $sub
if (-not (Test-Path $prefix)) {
    Write-Warning "Expected CMAKE_PREFIX_PATH candidate not found: $prefix"
    Write-Host "Search under: $OutputDir"
    exit 1
}

Write-Host ""
Write-Host "Configure CMake with:"
Write-Host "  -DCMAKE_PREFIX_PATH=`"$prefix`""
Write-Host ""
