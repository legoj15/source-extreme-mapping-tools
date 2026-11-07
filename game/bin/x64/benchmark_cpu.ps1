$SDK_BIN = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\bin\x64"
$MOD_DIR = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\sourcetest"
$BSP_PATH = Resolve-Path "game\bin\x64\bsp_unit_tests\cpu\validation_harder.bsp"

if (-not (Test-Path $BSP_PATH)) {
    Write-Error "BSP not found! Run 'game\bin\x64\run_vvis_tests.ps1 -TestNames harder' once to generate it."
    exit 1
}

# Copy the latest VVIS build (vvis_optix.exe and its DLL; nothing in it uses the GPU)
Copy-Item "game\bin\x64\vvis_optix.exe" "$SDK_BIN\vvis_optix.exe" -Force
Copy-Item "game\bin\x64\vvis_optix_dll.dll" "$SDK_BIN\vvis_optix_dll.dll" -Force

$start = Get-Date

# Construct arguments explicitly
$argsList = @("-game", "`"$MOD_DIR`"", "`"$BSP_PATH`"")

$proc = Start-Process -FilePath "$SDK_BIN\vvis_optix.exe" -ArgumentList $argsList -PassThru -Wait -NoNewWindow

$end = Get-Date
$duration = ($end - $start).TotalSeconds

if ($proc.ExitCode -ne 0) {
    Write-Host "VVIS Failed with code $($proc.ExitCode)" -ForegroundColor Red
}
else {
    Write-Host "Duration: $duration seconds" -ForegroundColor Green
}
