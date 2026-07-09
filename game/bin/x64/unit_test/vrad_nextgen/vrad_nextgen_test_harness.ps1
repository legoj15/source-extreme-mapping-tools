# VRAD-Nextgen Two-Way Test Harness
# Compares vrad_rtx.exe (CPU path) against vrad_nextgen.exe.
# Called by run_vrad_nextgen_tests.ps1 with a per-test configuration hashtable.

param(
    [Parameter(Mandatory)][hashtable]$TestConfig,
    [int]$TimeoutExtensionMinutes = 0,
    [switch]$SkipVisualCheck = $false
)

# --- Unpack test config ---
$MAP_NAME = $TestConfig.MapName
$CPU_EXTRA_ARGS = $TestConfig.CpuExtraArgs            # array of strings
$NEXTGEN_EXTRA_ARGS = $TestConfig.NextgenExtraArgs     # array of strings
$TIMEOUT_MULT = $TestConfig.TimeoutMultiplier
$CPU_NEXTGEN_TOL = $TestConfig.CpuNextgenTolerance
$LIGHTMAP_THRESH = $TestConfig.LightmapThreshold
$ARCHIVE_SUFFIX = $TestConfig.ArchiveSuffix

# --- Constants ---
$MOD_DIR = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\sourcetest"
$LOG_FILE = "test_vrad_nextgen_$ARCHIVE_SUFFIX.log"

$CPU_DIR = "..\unit_test_maps\cpu-nextgen"
$NEXTGEN_DIR = "..\unit_test_maps\nextgen"

$CPU_LOG = "$CPU_DIR\$MAP_NAME.log"
$NEXTGEN_LOG = "$NEXTGEN_DIR\$MAP_NAME.log"

$GAME_EXE = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\hl2.exe"
$GAME_MAPS = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\sourcetest\maps"
$GAME_SCREENSHOTS = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\sourcetest\screenshots"

# --- Process cleanup ---
$tools = @("vrad", "vrad_rtx", "vrad_nextgen")
foreach ($tool in $tools) {
    Get-Process -Name $tool -ErrorAction SilentlyContinue | Stop-Process -Force
}

# --- Logging ---
"" | Out-File -FilePath $LOG_FILE -Encoding utf8

function Write-LogMessage {
    param([string]$Message, [bool]$ToLog = $true)
    Write-Host $Message
    if ($ToLog) {
        $Message | Out-File -FilePath $LOG_FILE -Append -Encoding utf8
    }
}

# --- Screenshot helper ---
function Take-Screenshot {
    param([string]$BspPath, [string]$TargetTga)

    Write-LogMessage "Taking screenshot for $BspPath..."

    # 1. Copy bsp
    Copy-Item $BspPath "$GAME_MAPS\$MAP_NAME.bsp" -Force

    # 2. Run hl2.exe
    $gameArgs = "-game", "sourcetest", "-novid", "-sw", "-w", "2560", "-h", "1440", "+sv_cheats 1", "+map $MAP_NAME", "+cl_mouselook 0", "+cl_drawhud 0", "+r_drawviewmodel 0", "+mat_fullbright 2", "+wait 1000", "+screenshot", "+quit"
    $proc = Start-Process -FilePath $GAME_EXE -ArgumentList $gameArgs -Wait -PassThru
    if ($proc.ExitCode -ne 0) {
        Write-LogMessage "CRITICAL ERROR: hl2.exe exited with code $($proc.ExitCode) for map $BspPath."
        throw "FAIL"
    }

    # 3. Move screenshot
    if (Test-Path "$GAME_SCREENSHOTS\${MAP_NAME}0000.tga") {
        Move-Item "$GAME_SCREENSHOTS\${MAP_NAME}0000.tga" $TargetTga -Force
    }
    else {
        Write-LogMessage "CRITICAL ERROR: Screenshot not found for $BspPath!"
        throw "FAIL"
    }
}

# ============================================================
#  Main test flow
# ============================================================
$testFailed = $false
try {

    Write-LogMessage "=== Test: $ARCHIVE_SUFFIX ($MAP_NAME) ==="
    Write-LogMessage "Mode: vrad_rtx CPU vs vrad_nextgen"

    # Ensure directories exist
    if (!(Test-Path $CPU_DIR)) { New-Item -ItemType Directory -Path $CPU_DIR | Out-Null }
    if (!(Test-Path $NEXTGEN_DIR)) { New-Item -ItemType Directory -Path $NEXTGEN_DIR | Out-Null }

    $MAP_SRC = "..\..\unit_test_maps\$MAP_NAME.bsp"
    if (!(Test-Path $MAP_SRC)) {
        Write-LogMessage "CRITICAL ERROR: Source map $MAP_SRC not found!"
        throw "FAIL"
    }
    Copy-Item $MAP_SRC "$CPU_DIR\$MAP_NAME.bsp" -Force
    Copy-Item $MAP_SRC "$NEXTGEN_DIR\$MAP_NAME.bsp" -Force

    # --- Phase 1: CPU Baseline (vrad_rtx.exe with -bounce 0) ---
    Write-LogMessage "--- Phase 1: CPU Baseline (vrad_rtx.exe CPU, -bounce 0) ---"
    $fullLogPath = Join-Path (Get-Location).Path $CPU_LOG
    Write-LogMessage "vrad_rtx.exe CPU Log: $fullLogPath"
    $start = Get-Date
    & "..\..\vrad_rtx.exe" @CPU_EXTRA_ARGS -game $MOD_DIR "$CPU_DIR\$MAP_NAME" *>$null
    if ($LASTEXITCODE -ne 0) {
        Write-LogMessage "CRITICAL ERROR: vrad_rtx.exe (CPU baseline) failed with exit code $LASTEXITCODE."
        throw "FAIL"
    }
    $cpuTime = (Get-Date) - $start

    # --- Phase 2: Nextgen (vrad_nextgen.exe) ---
    Write-LogMessage "--- Phase 2: Nextgen (vrad_nextgen.exe) ---"
    $fullLogPath = Join-Path (Get-Location).Path $NEXTGEN_LOG
    Write-LogMessage "vrad_nextgen.exe Log: $fullLogPath"
    $start = Get-Date

    # Use Start-Process with file redirection for safe pipe draining.
    $bspFullPath = (Resolve-Path "$NEXTGEN_DIR\$MAP_NAME.bsp").ProviderPath
    # Cut off the .bsp extension because VRAD expects the name without it
    $bspFullPath = $bspFullPath.Substring(0, $bspFullPath.Length - 4)
    $nextgenArgs = @($NEXTGEN_EXTRA_ARGS) + @("-game", "`"$MOD_DIR`"", "`"$bspFullPath`"")

    $outLogTmp = "$env:TEMP\vrad_nextgen_out.txt"
    $errLogTmp = "$env:TEMP\vrad_nextgen_err.txt"
    Remove-Item $outLogTmp -ErrorAction SilentlyContinue
    Remove-Item $errLogTmp -ErrorAction SilentlyContinue

    try {
        $process = Start-Process -FilePath "..\..\vrad_nextgen.exe" `
            -ArgumentList $nextgenArgs `
            -RedirectStandardOutput $outLogTmp `
            -RedirectStandardError $errLogTmp `
            -NoNewWindow -PassThru
    }
    catch {
        Write-LogMessage "CRITICAL ERROR: Failed to start vrad_nextgen.exe: $($_.Exception.Message)"
        throw "FAIL"
    }

    if ($null -eq $process) {
        Write-LogMessage "CRITICAL ERROR: Failed to start vrad_nextgen.exe. Process object is NULL."
        throw "FAIL"
    }

    $timedOut = $false
    $maxSeconds = ($cpuTime.TotalSeconds * $TIMEOUT_MULT) + ($TimeoutExtensionMinutes * 60)
    $hasWarnedByExceedingControl = $false

    while (-not $process.HasExited) {
        $elapsed = (Get-Date) - $start
        if (($elapsed.TotalSeconds -gt $cpuTime.TotalSeconds) -and (-not $hasWarnedByExceedingControl)) {
            $remaining = [math]::Round($maxSeconds - $elapsed.TotalSeconds)
            Write-LogMessage "WARNING: Nextgen run has exceeded the CPU baseline time ($([math]::Round($cpuTime.TotalSeconds))s). Will terminate in ${remaining}s."
            $hasWarnedByExceedingControl = $true
        }
        if ($elapsed.TotalSeconds -gt $maxSeconds) {
            $process.Kill()
            $timedOut = $true
            Write-LogMessage "CRITICAL ERROR: vrad_nextgen.exe hung or is significantly slower than CPU baseline! Use -TimeoutExtensionMinutes <minutes> to extend wait time."
            break
        }
        Start-Sleep -Seconds 1
    }

    if (-not $timedOut) {
        $process.WaitForExit()
    }

    # Append standard output to full log
    if (Test-Path $outLogTmp) {
        Get-Content $outLogTmp | Out-File -FilePath $fullLogPath -Append -Encoding utf8
    }

    # Final refresh to ensure exit code is captured
    $exitCode = if ($process.HasExited) { $process.ExitCode } else { $null }

    if (-not $timedOut -and ($null -eq $exitCode -or $exitCode -ne 0)) {
        $errMessage = if ($null -eq $exitCode) { "UNKNOWN (NULL)" } else { $exitCode }
        Write-LogMessage "CRITICAL ERROR: vrad_nextgen.exe failed with exit code $errMessage."
        throw "FAIL"
    }
    $nextgenTime = (Get-Date) - $start

    # --- Timing Summary ---
    Write-LogMessage "`n--- Timing Summary ---"
    Write-LogMessage "vrad_rtx.exe Time (CPU, -bounce 0):`t`t$($cpuTime.TotalSeconds.ToString("F2"))s"
    if ($timedOut) {
        Write-LogMessage "vrad_nextgen.exe Time:`t`t`t`tDid not finish"
    }
    else {
        Write-LogMessage "vrad_nextgen.exe Time:`t`t`t`t$($nextgenTime.TotalSeconds.ToString("F2"))s"
    }

    # --- Phase 3: CPU vs Nextgen Comparison ---
    if (-not $timedOut) {
        Write-LogMessage "`n--- Phase 3: Comparing CPU vs Nextgen ---"
        $pythonDiff = python "..\bsp_diff_lightmaps.py" "$CPU_DIR\$MAP_NAME.bsp" "$NEXTGEN_DIR\$MAP_NAME.bsp" --threshold $LIGHTMAP_THRESH 2>&1
        $bspDiffExitCode = $LASTEXITCODE
        $pythonDiff | Write-Host
        $pythonDiff | Out-File -FilePath $LOG_FILE -Append -Encoding utf8

        if ($bspDiffExitCode -eq 0) {
            Write-LogMessage "RESULT: PASS"
        }
        else {
            if ($SkipVisualCheck) {
                Write-LogMessage "WARNING: Lightmaps differ. Visual comparison skipped (cpu vs nextgen)."
            }
            else {
                Write-LogMessage "Initiating visual comparison (cpu vs nextgen)..."
                Take-Screenshot "$CPU_DIR\$MAP_NAME.bsp" "screenshot_cpu-nextgen_$MAP_NAME.tga"
                Take-Screenshot "$NEXTGEN_DIR\$MAP_NAME.bsp" "screenshot_nextgen_$MAP_NAME.tga"

                $diffOutput = python "..\python_ssim_diff.py" "screenshot_cpu-nextgen_$MAP_NAME.tga" "screenshot_nextgen_$MAP_NAME.tga" "screenshot_diff_cpu_nextgen_$MAP_NAME" 2>&1
                $diffMatch = $diffOutput | Select-String "Difference: ([\d\.]+)%"
                if ($diffMatch) {
                    $percentDiff = [double]$diffMatch.Matches.Groups[1].Value
                    Write-LogMessage "Visual Difference (cpu vs nextgen): $percentDiff%"
                    if ($percentDiff -ge $CPU_NEXTGEN_TOL) {
                        Write-LogMessage "RESULT: FAIL (Visual difference $percentDiff% >= $CPU_NEXTGEN_TOL%)"
                        throw "FAIL"
                    }
                    else {
                        Write-LogMessage "RESULT: PASS (Visual difference $percentDiff% < $CPU_NEXTGEN_TOL%)"
                    }
                }
                else {
                    Write-LogMessage "Warning: Could not parse ssim diff output for cpu vs nextgen."
                    Write-LogMessage "ssim diff output: $diffOutput"
                    throw "FAIL"
                }
            }
        }
    }

}
catch {
    $testFailed = $true
}

# --- Archive Logs ---
$ARCHIVE_DIR = "..\unit_test_logs"
if (!(Test-Path $ARCHIVE_DIR)) { New-Item -ItemType Directory -Path $ARCHIVE_DIR | Out-Null }

# Convert any generated TGA files to PNG for easier viewing
Write-LogMessage "Converting TGA screenshots to PNG..."
$tgaFiles = @(
    "screenshot_cpu-nextgen_$MAP_NAME.tga",
    "screenshot_nextgen_$MAP_NAME.tga"
)
$tgaArgs = @()
foreach ($tga in $tgaFiles) {
    if (Test-Path $tga) {
        $tgaArgs += $tga
    }
}
if ($tgaArgs.Count -gt 0) {
    python "..\tga2png.py" $tgaArgs | Write-Host
}

$timestamp = (Get-Date).ToString("yyyy-MM-ddTHH-mm-ss")
$RUN_DIR = "$ARCHIVE_DIR\${timestamp}_$ARCHIVE_SUFFIX"
New-Item -ItemType Directory -Path $RUN_DIR | Out-Null

$logMap = @{
    $LOG_FILE    = "test_$ARCHIVE_SUFFIX.log"
    $CPU_LOG     = "vrad_cpu.log"
    $NEXTGEN_LOG = "vrad_nextgen.log"
}
foreach ($entry in $logMap.GetEnumerator()) {
    if (Test-Path $entry.Key) {
        Move-Item $entry.Key "$RUN_DIR\$($entry.Value)" -Force
    }
}

# Archive PNG screenshots alongside logs
$pngFiles = @(
    "screenshot_cpu-nextgen_$MAP_NAME.png",
    "screenshot_nextgen_$MAP_NAME.png",
    "screenshot_diff_cpu_nextgen_$MAP_NAME-alpha.png",
    "screenshot_diff_cpu_nextgen_$MAP_NAME-alphacontrast.png"
)
foreach ($png in $pngFiles) {
    if (Test-Path $png) {
        Move-Item $png "$RUN_DIR\$png" -Force
    }
}

# Cleanup original TGAs
foreach ($tga in $tgaFiles) {
    if (Test-Path $tga) {
        Remove-Item $tga -Force
    }
}

# Archive any stray diagnostic logs
$strayLogs = @("debug_out.txt", "jump_log.txt", "final_vis_results.log", "vis_debug.txt", "crash.txt", "test_vmf_faces_final.log", "test_vmf_faces_fast.log", "test_vmf_faces_log.txt")
foreach ($stray in $strayLogs) {
    if (Test-Path $stray) {
        Move-Item $stray "$RUN_DIR\$stray" -Force
    }
}
Write-LogMessage "Logs and screenshots archived to $RUN_DIR"

if ($testFailed -or $timedOut) { exit 1 }
