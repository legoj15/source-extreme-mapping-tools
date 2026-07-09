param(
    [Parameter(Mandatory)][string]$MapName,
    [string[]]$ExtraArgs = @(),
    [string]$GameDir = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\sourcetest",
    [string]$GameExe = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\hl2.exe"
)

$GAME_MAPS = "$GameDir\maps"
$GAME_SCREENSHOTS = "$GameDir\screenshots"

# The mod name is the final folder in the GameDir path
$ModName = Split-Path $GameDir -Leaf

$SRC_MAP = "unit_test_maps\$MapName.bsp"

if (-not (Test-Path $SRC_MAP)) {
    Write-Host "Error: Cannot find source map at $SRC_MAP"
    exit 1
}

$OUT_DIR = "visual_comparison_$MapName"
if (-not (Test-Path $OUT_DIR)) { New-Item -ItemType Directory -Path $OUT_DIR | Out-Null }

function Take-Screenshot {
    param([string]$BspPath, [string]$TargetTga)

    Write-Host "Taking screenshot for $BspPath..."

    # 1. Copy bsp
    Copy-Item $BspPath "$GAME_MAPS\$MapName.bsp" -Force

    # 2. Run hl2.exe
    $gameArgs = "-game", $ModName, "-novid", "-sw", "-w", "2560", "-h", "1440", "+sv_cheats 1", "+map $MapName", "+cl_mouselook 0", "+cl_drawhud 0", "+r_drawviewmodel 0", "+mat_fullbright 2", "+wait 1000", "+screenshot", "+quit"
    $proc = Start-Process -FilePath $GameExe -ArgumentList $gameArgs -Wait -PassThru
    if ($proc.ExitCode -ne 0) {
        Write-Host "CRITICAL ERROR: hl2.exe exited with code $($proc.ExitCode) for map $BspPath."
        throw "FAIL"
    }

    # 3. Move screenshot
    if (Test-Path "$GAME_SCREENSHOTS\${MapName}0000.tga") {
        Move-Item "$GAME_SCREENSHOTS\${MapName}0000.tga" $TargetTga -Force
    }
    else {
        Write-Host "CRITICAL ERROR: Screenshot not found for $BspPath! Check if the game took it."
        throw "FAIL"
    }
}

# Delete old screenshots just in case
Remove-Item "$GAME_SCREENSHOTS\${MapName}0000.tga" -ErrorAction SilentlyContinue

# --- CPU ---
$cpuBsp = "$PWD\$OUT_DIR\${MapName}_cpu.bsp"
Copy-Item "$PWD\$SRC_MAP" $cpuBsp -Force
Write-Host ""
Write-Host "========================================"
Write-Host "--- Running vrad.exe (CPU)           ---"
Write-Host "========================================"
& ".\vrad_rtx.exe" @ExtraArgs -game $GameDir $cpuBsp
Take-Screenshot $cpuBsp "$PWD\$OUT_DIR\${MapName}_cpu.tga"

# --- Hybrid ---
$hybridBsp = "$PWD\$OUT_DIR\${MapName}_hybrid.bsp"
Copy-Item "$PWD\$SRC_MAP" $hybridBsp -Force
Write-Host ""
Write-Host "========================================"
Write-Host "--- Running vrad_rtx.exe (Hybrid)    ---"
Write-Host "========================================"
& ".\vrad_rtx.exe" @ExtraArgs -cuda -game $GameDir $hybridBsp
Take-Screenshot $hybridBsp "$PWD\$OUT_DIR\${MapName}_hybrid.tga"

# --- Nextgen ---
$nextgenBsp = "$PWD\$OUT_DIR\${MapName}_nextgen.bsp"
Copy-Item "$PWD\$SRC_MAP" $nextgenBsp -Force
Write-Host ""
Write-Host "========================================"
Write-Host "--- Running vrad_nextgen.exe (GPU)   ---"
Write-Host "========================================"
& ".\vrad_nextgen.exe" @ExtraArgs -game $GameDir $nextgenBsp
Take-Screenshot $nextgenBsp "$PWD\$OUT_DIR\${MapName}_nextgen.tga"

# --- Convert to PNG ---
Write-Host ""
Write-Host "========================================"
Write-Host "--- Converting screenshots to PNG    ---"
Write-Host "========================================"
python unit_test\tga2png.py "$PWD\$OUT_DIR\${MapName}_cpu.tga" "$PWD\$OUT_DIR\${MapName}_hybrid.tga" "$PWD\$OUT_DIR\${MapName}_nextgen.tga"

Write-Host "Done! Check the '$OUT_DIR' folder for the PNG screenshots."
