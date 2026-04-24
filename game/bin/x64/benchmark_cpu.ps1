$SDK_BIN = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\bin\x64"
$MOD_DIR = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\sourcetest"
$BSP_PATH = Resolve-Path "game\bin\x64\bsp_unit_tests\cpu\validation_harder.bsp"

if (-not (Test-Path $BSP_PATH)) {
    Write-Error "BSP not found! Run 'game\bin\x64\run_vvis_tests.ps1 -TestNames harder' once to generate it."
    exit 1
}

# Copy latest vvis_cuda.exe
Copy-Item "game\bin\x64\vvis_cuda.exe" "$SDK_BIN\vvis_cuda.exe" -Force
if (Test-Path "game\bin\x64\vvis_dll_cuda.dll") {
    Copy-Item "game\bin\x64\vvis_dll_cuda.dll" "$SDK_BIN\vvis_dll_cuda.dll" -Force
}

$start = Get-Date

# Construct arguments explicitly
$argsList = @("-game", "`"$MOD_DIR`"", "`"$BSP_PATH`"")

$proc = Start-Process -FilePath "$SDK_BIN\vvis_cuda.exe" -ArgumentList $argsList -PassThru -Wait -NoNewWindow

$end = Get-Date
$duration = ($end - $start).TotalSeconds

if ($proc.ExitCode -ne 0) {
    Write-Host "VVIS Failed with code $($proc.ExitCode)" -ForegroundColor Red
}
else {
    Write-Host "Duration: $duration seconds" -ForegroundColor Green
}
