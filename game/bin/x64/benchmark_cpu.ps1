# Benchmarks the vvis_optix.exe CPU path on the validation_harder unit-test map.
# Generate the BSP first by running the vvis "harder" test once:
#   .\unit_test\vvis_optix\run_vvis_tests.ps1 -TestNames harder

$SDK_BIN = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\bin\x64"
$MOD_DIR = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\sourcetest"

# Anchor to this script's own directory (game\bin\x64) so it runs from any CWD.
$BIN_DIR = $PSScriptRoot
$BSP_PATH = Join-Path $BIN_DIR "unit_test\unit_test_output\vvis-cpu\validation_harder.bsp"

# Check existence BEFORE Resolve-Path (Resolve-Path throws on a missing path,
# which would bypass this friendly message).
if (-not (Test-Path $BSP_PATH)) {
    Write-Error "BSP not found at $BSP_PATH. Run '.\unit_test\vvis_optix\run_vvis_tests.ps1 -TestNames harder' once to generate it."
    exit 1
}
$BSP_PATH = (Resolve-Path $BSP_PATH).ProviderPath

# Deploy the freshly built vvis_optix binary (plus its DLL and PTX) into the SDK's
# bin\x64 so the SDK engine loads our build. Mirrors the vvis test harness.
$srcExe = Join-Path $BIN_DIR "vvis_optix.exe"
if (-not (Test-Path $srcExe)) {
    Write-Error "vvis_optix.exe not found at $srcExe. Build it before benchmarking."
    exit 1
}
Copy-Item $srcExe (Join-Path $SDK_BIN "vvis_optix.exe") -Force
$srcDll = Join-Path $BIN_DIR "vvis_optix_dll.dll"
if (Test-Path $srcDll) { Copy-Item $srcDll (Join-Path $SDK_BIN "vvis_optix_dll.dll") -Force }
$srcPtx = Join-Path $BIN_DIR "vvis_optix.ptx"
if (Test-Path $srcPtx) { Copy-Item $srcPtx (Join-Path $SDK_BIN "vvis_optix.ptx") -Force }

# Time the CPU path (no -cuda). Run with the SDK bin as the working directory so
# the tool resolves its DLL/PTX and relative file I/O there.
$start = Get-Date
$argsList = @("-game", "`"$MOD_DIR`"", "`"$BSP_PATH`"")
$proc = Start-Process -FilePath (Join-Path $SDK_BIN "vvis_optix.exe") -ArgumentList $argsList -WorkingDirectory $SDK_BIN -PassThru -Wait -NoNewWindow
$duration = ((Get-Date) - $start).TotalSeconds

if ($proc.ExitCode -ne 0) {
    Write-Host "VVIS Failed with code $($proc.ExitCode)" -ForegroundColor Red
}
else {
    Write-Host "Duration: $duration seconds" -ForegroundColor Green
}
