param()

$MapName = "validation_vrad_gridlines"
$GameDir = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\sourcetest"
$GameExe = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\hl2.exe"

$GAME_MAPS = "$GameDir\maps"
$GAME_SCREENSHOTS = "$GameDir\screenshots"
$ModName = Split-Path $GameDir -Leaf

$BspPath = "$PSScriptRoot\visual_comparison_validation_vrad_gridlines\validation_vrad_gridlines_nextgen_injected.bsp"
$TargetTga = "$PSScriptRoot\visual_comparison_validation_vrad_gridlines\validation_vrad_gridlines_nextgen_injected.tga"

Remove-Item "$GAME_SCREENSHOTS\${MapName}0000.tga" -ErrorAction SilentlyContinue

Copy-Item $BspPath "$GAME_MAPS\$MapName.bsp" -Force

$gameArgs = "-game", $ModName, "-novid", "-sw", "-w", "2560", "-h", "1440", "+sv_cheats 1", "+map $MapName", "+cl_mouselook 0", "+cl_drawhud 0", "+r_drawviewmodel 0", "+mat_fullbright 2", "+wait 1000", "+screenshot", "+quit"
$proc = Start-Process -FilePath $GameExe -ArgumentList $gameArgs -Wait -PassThru

if (Test-Path "$GAME_SCREENSHOTS\${MapName}0000.tga") {
    Move-Item "$GAME_SCREENSHOTS\${MapName}0000.tga" $TargetTga -Force
    Write-Host "Success taking screenshot!"
} else {
    Write-Host "Fail"
}
