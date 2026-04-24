# VRAD-RTX Three-Way Test Harness
# Shared engine for all validation tests. Called by run_vrad_tests.ps1 with a
# per-test configuration hashtable.
#
# This file contains ALL the logic that was previously duplicated across
# test_vrad_rtx.ps1, test_vrad_rtx_quick.ps1, test_vrad_rtx_props.ps1,
# test_vrad_rtx_radiosity.ps1, and test_vrad_rtx_supersampling.ps1.

param(
    [Parameter(Mandatory)][hashtable]$TestConfig,
    [int]$TimeoutExtensionMinutes = 0,
    [switch]$SkipVisualCheck = $false
)

# --- Unpack test config ---
$MAP_NAME = $TestConfig.MapName
$EXTRA_ARGS = $TestConfig.ExtraArgs          # array of strings
$TIMEOUT_MULT = $TestConfig.TimeoutMultiplier
$REF_CPU_TOL = $TestConfig.RefCpuTolerance
$CPU_GPU_TOL = $TestConfig.CpuGpuTolerance
$LIGHTMAP_THRESH = $TestConfig.LightmapThreshold
$ARCHIVE_SUFFIX = $TestConfig.ArchiveSuffix

# --- Constants ---
$MOD_DIR = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\sourcetest"
$LOG_FILE = "test_vrad_rtx_$ARCHIVE_SUFFIX.log"

$REF_DIR = "bsp_unit_tests\ref-cpu"
$CPU_DIR = "bsp_unit_tests\cpu"
$GPU_DIR = "bsp_unit_tests\gpu"

$REF_LOG = "$REF_DIR\$MAP_NAME.log"
$CPU_LOG = "$CPU_DIR\$MAP_NAME.log"
$GPU_LOG = "$GPU_DIR\$MAP_NAME.log"

$GAME_EXE = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\hl2.exe"
$GAME_MAPS = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\sourcetest\maps"
$GAME_SCREENSHOTS = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\sourcetest\screenshots"

# --- Process cleanup ---
$tools = @("vbsp", "vvis", "vrad", "vvis_cuda", "vrad_rtx")
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

    # Ensure directories exist
    if (!(Test-Path $REF_DIR)) { New-Item -ItemType Directory -Path $REF_DIR | Out-Null }
    if (!(Test-Path $CPU_DIR)) { New-Item -ItemType Directory -Path $CPU_DIR | Out-Null }
    if (!(Test-Path $GPU_DIR)) { New-Item -ItemType Directory -Path $GPU_DIR | Out-Null }

    # Refresh maps from source
    $MAP_SRC = "bsp_unit_tests\$MAP_NAME.bsp"
    if (!(Test-Path $MAP_SRC)) {
        Write-LogMessage "CRITICAL ERROR: Source map $MAP_SRC not found!"
        throw "FAIL"
    }
    Copy-Item $MAP_SRC "$CPU_DIR\$MAP_NAME.bsp" -Force
    Copy-Item $MAP_SRC "$GPU_DIR\$MAP_NAME.bsp" -Force

    $HasReference = Test-Path ".\vrad.exe"

    # --- Phase 1: ref-cpu (vrad.exe, CPU only) ---
    # Skip recompilation if the source BSP and test manifest haven't changed.
    # We hash the manifest (vrad_tests.psd1) because per-test parameters like
    # ExtraArgs and tolerances live there, not in this shared harness script.
    $MANIFEST_PATH = Join-Path $PSScriptRoot "vrad_tests.psd1"
    $REF_HASH_FILE = "$REF_DIR\$MAP_NAME.ref_hash"
    $currentHash = (Get-FileHash $MAP_SRC -Algorithm SHA256).Hash + ":" + (Get-FileHash $MANIFEST_PATH -Algorithm SHA256).Hash
    $cachedHash = if (Test-Path $REF_HASH_FILE) { (Get-Content $REF_HASH_FILE -Raw).Trim() } else { "" }
    $needsRefCompile = ($currentHash -ne $cachedHash) -or !(Test-Path "$REF_DIR\$MAP_NAME.bsp")

    $refTime = New-TimeSpan
    $refTimeCached = $false
    if ($HasReference) {
        if ($needsRefCompile) {
            Copy-Item $MAP_SRC "$REF_DIR\$MAP_NAME.bsp" -Force
            Write-LogMessage "--- Compiling ref-cpu map (Original vrad.exe) ---"
            $fullLogPath = Join-Path (Get-Location).Path $REF_LOG
            Write-LogMessage "vrad.exe (SDK Reference) Log: $fullLogPath"
            $start = Get-Date
            & ".\vrad.exe" @EXTRA_ARGS -game $MOD_DIR "$REF_DIR\$MAP_NAME" *>$null
            if ($LASTEXITCODE -ne 0) {
                Write-LogMessage "WARNING: vrad.exe (reference) failed with exit code $LASTEXITCODE. Skipping ref-cpu comparison."
                $HasReference = $false
            }
            else {
                $refTime = (Get-Date) - $start
                $currentHash | Out-File -FilePath $REF_HASH_FILE -Encoding utf8 -NoNewline
                $refTime.TotalSeconds.ToString("F2") | Out-File -FilePath "$REF_DIR\$MAP_NAME.ref_time" -Encoding utf8 -NoNewline
            }
        }
        else {
            Write-LogMessage "ref-cpu unchanged (BSP and script identical), using cached result."
            $refTimeFile = "$REF_DIR\$MAP_NAME.ref_time"
            if (Test-Path $refTimeFile) {
                $refTime = [TimeSpan]::FromSeconds([double](Get-Content $refTimeFile -Raw).Trim())
                $refTimeCached = $true
            }
        }
    }
    else {
        Write-LogMessage "vrad.exe not found, skipping ref-cpu pass."
    }

    # --- Phase 2: cpu (vrad_rtx.exe CPU) ---
    Write-LogMessage "--- Compiling cpu map (vrad_rtx.exe CPU only) ---"
    $fullLogPath = Join-Path (Get-Location).Path $CPU_LOG
    Write-LogMessage "vrad_rtx.exe CPU Log: $fullLogPath"
    $start = Get-Date
    & ".\vrad_rtx.exe" @EXTRA_ARGS -game $MOD_DIR "$CPU_DIR\$MAP_NAME" *>$null
    if ($LASTEXITCODE -ne 0) {
        Write-LogMessage "CRITICAL ERROR: vrad_rtx.exe (control, CPU only) failed with exit code $LASTEXITCODE."
        throw "FAIL"
    }
    $cpuTime = (Get-Date) - $start

    # --- Phase 3: ref-cpu vs cpu Comparison ---
    if ($HasReference) {
        Write-LogMessage "`n--- Comparing ref-cpu vs cpu (CPU Parity Check) ---"
        $pythonDiff = python bsp_diff_lightmaps.py "$REF_DIR\$MAP_NAME.bsp" "$CPU_DIR\$MAP_NAME.bsp" --threshold $LIGHTMAP_THRESH 2>&1
        $bspDiffExitCode = $LASTEXITCODE
        $pythonDiff | Write-Host
        $pythonDiff | Out-File -FilePath $LOG_FILE -Append -Encoding utf8

        if ($bspDiffExitCode -eq 0) {
            Write-LogMessage "RESULT: PASS"
        }
        else {
            if ($SkipVisualCheck) {
                Write-LogMessage "WARNING: Lightmaps differ. Visual comparison skipped (ref-cpu vs cpu)."
            }
            else {
                Write-LogMessage "Lightmaps differ. Initiating visual comparison (ref-cpu vs cpu)..."
                if ($needsRefCompile -or !(Test-Path "screenshot_ref-cpu_$MAP_NAME.tga")) {
                    Take-Screenshot "$REF_DIR\$MAP_NAME.bsp" "screenshot_ref-cpu_$MAP_NAME.tga"
                }
                else {
                    Write-LogMessage "Using cached ref-cpu screenshot."
                }
                Take-Screenshot "$CPU_DIR\$MAP_NAME.bsp" "screenshot_cpu_$MAP_NAME.tga"

                $diffOutput = python python_ssim_diff.py screenshot_ref-cpu_$MAP_NAME.tga screenshot_cpu_$MAP_NAME.tga screenshot_diff_ref_cpu_$MAP_NAME 2>&1
                $diffMatch = $diffOutput | Select-String "Difference: ([\d\.]+)%"
                if ($diffMatch) {
                    $percentDiff = [double]$diffMatch.Matches.Groups[1].Value
                    Write-LogMessage "Visual Difference (ref-cpu vs cpu): $percentDiff%"
                    if ($percentDiff -gt $REF_CPU_TOL) {
                        Write-LogMessage "CRITICAL ERROR: Fundamental issue detected! CPU tests should be identical. Difference: $percentDiff% > $REF_CPU_TOL% tolerance."
                        throw "FAIL"
                    }
                    else {
                        Write-LogMessage "CPU Visual tests passed within $REF_CPU_TOL% margin of error."
                    }
                }
                else {
                    Write-LogMessage "CRITICAL ERROR: Could not parse ssim diff output for ref-cpu vs cpu."
                    Write-LogMessage "ssim diff output: $diffOutput"
                    throw "FAIL"
                }
            }
        }
    }

    # --- Phase 4: gpu (vrad_rtx.exe -cuda) ---
    Write-LogMessage "--- Compiling gpu map (vrad_rtx.exe -cuda) ---"
    $fullLogPath = Join-Path (Get-Location).Path $GPU_LOG
    Write-LogMessage "vrad_rtx.exe -cuda Log: $fullLogPath"
    $start = Get-Date

    # Use native Start-Process with file redirection to drain output safely.
    # This prevents pipe buffer (4KB) deadlocks on Windows without needing
    # complex .NET async event handlers.
    $cudaArgs = @($EXTRA_ARGS) + @("-game", "`"$MOD_DIR`"", "-cuda", "`"$GPU_DIR\$MAP_NAME`"")
    
    $outLogTmp = "$env:TEMP\vrad_rtx_cuda_out.txt"
    $errLogTmp = "$env:TEMP\vrad_rtx_cuda_err.txt"
    Remove-Item $outLogTmp -ErrorAction SilentlyContinue
    Remove-Item $errLogTmp -ErrorAction SilentlyContinue

    try {
        $process = Start-Process -FilePath ".\vrad_rtx.exe" `
            -ArgumentList $cudaArgs `
            -RedirectStandardOutput $outLogTmp `
            -RedirectStandardError $errLogTmp `
            -NoNewWindow -PassThru
    }
    catch {
        Write-LogMessage "CRITICAL ERROR: Failed to start vrad_rtx.exe -cuda: $($_.Exception.Message)"
        throw "FAIL"
    }

    if ($null -eq $process) {
        Write-LogMessage "CRITICAL ERROR: Failed to start vrad_rtx.exe -cuda. Process object is NULL."
        throw "FAIL"
    }

    $timedOut = $false
    $maxSeconds = ($cpuTime.TotalSeconds * $TIMEOUT_MULT) + ($TimeoutExtensionMinutes * 60)
    $hasWarnedByExceedingControl = $false

    while (-not $process.HasExited) {
        $elapsed = (Get-Date) - $start
        if (($elapsed.TotalSeconds -gt $cpuTime.TotalSeconds) -and (-not $hasWarnedByExceedingControl)) {
            $remaining = [math]::Round($maxSeconds - $elapsed.TotalSeconds)
            Write-LogMessage "WARNING: Test run has exceeded the cpu test time ($([math]::Round($cpuTime.TotalSeconds))s). Will terminate in ${remaining}s."
            $hasWarnedByExceedingControl = $true
        }
        if ($elapsed.TotalSeconds -gt $maxSeconds) {
            $process.Kill()
            $timedOut = $true
            Write-LogMessage "CRITICAL ERROR: vrad_rtx -cuda hung or is significantly slower than cpu test! Use -TimeoutExtensionMinutes <minutes> to extend wait time."
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
        Write-LogMessage "CRITICAL ERROR: vrad_rtx.exe -cuda failed with exit code $errMessage."
        throw "FAIL"
    }
    $gpuTime = (Get-Date) - $start

    # --- Timing Summary ---
    Write-LogMessage "`n--- Timing Summary ---"
    if ($HasReference) {
        $cachedLabel = if ($refTimeCached) { " (cached)" } else { "" }
        Write-LogMessage "vrad.exe Time (Source SDK 2013 Unmodified):`t$($refTime.TotalSeconds.ToString("F2"))s$cachedLabel"
    }
    Write-LogMessage "vrad_rtx.exe Time (CPU only):`t`t`t$($cpuTime.TotalSeconds.ToString("F2"))s"
    if ($timedOut) {
        Write-LogMessage "vrad_rtx.exe -cuda Time (GPU accelerated):`tDid not finish"
    }
    else {
        Write-LogMessage "vrad_rtx.exe -cuda Time (GPU accelerated):`t$($gpuTime.TotalSeconds.ToString("F2"))s"
    }

    # --- Phase 5: cpu vs gpu (GPU Parity Check) ---
    if (-not $timedOut) {
        Write-LogMessage "`n--- Comparing cpu vs gpu (GPU Parity Check) ---"
        $pythonDiff = python bsp_diff_lightmaps.py "$CPU_DIR\$MAP_NAME.bsp" "$GPU_DIR\$MAP_NAME.bsp" --threshold $LIGHTMAP_THRESH 2>&1
        $bspDiffExitCode = $LASTEXITCODE
        $pythonDiff | Write-Host
        $pythonDiff | Out-File -FilePath $LOG_FILE -Append -Encoding utf8

        if ($bspDiffExitCode -eq 0) {
            Write-LogMessage "RESULT: PASS"
        }
        else {
            if ($SkipVisualCheck) {
                Write-LogMessage "WARNING: Lightmaps differ. Visual comparison skipped (cpu vs gpu)."
            }
            else {
                Write-LogMessage "Initiating visual comparison for GPU..."
                # Always retake the CPU screenshot fresh to avoid stale resolution mismatches
                # (e.g. a prior run at a different resolution cached on disk).
                Take-Screenshot "$CPU_DIR\$MAP_NAME.bsp" "screenshot_cpu_$MAP_NAME.tga"
                Take-Screenshot "$GPU_DIR\$MAP_NAME.bsp" "screenshot_gpu_$MAP_NAME.tga"

                $diffOutput = python python_ssim_diff.py screenshot_cpu_$MAP_NAME.tga screenshot_gpu_$MAP_NAME.tga screenshot_diff_cpu_gpu_$MAP_NAME 2>&1
                $diffMatch = $diffOutput | Select-String "Difference: ([\d\.]+)%"
                if ($diffMatch) {
                    $percentDiff = [double]$diffMatch.Matches.Groups[1].Value
                    Write-LogMessage "Visual Difference (cpu vs gpu): $percentDiff%"
                    if ($percentDiff -ge $CPU_GPU_TOL) {
                        Write-LogMessage "RESULT: FAIL (Visual difference $percentDiff% >= $CPU_GPU_TOL%)"
                        throw "FAIL"
                    }
                    else {
                        Write-LogMessage "RESULT: PASS (Visual difference $percentDiff% < $CPU_GPU_TOL%)"
                    }
                }
                else {
                    Write-LogMessage "Warning: Could not parse ssim diff output for cpu vs gpu."
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
$ARCHIVE_DIR = "bsp_unit_test_logs"
if (!(Test-Path $ARCHIVE_DIR)) { New-Item -ItemType Directory -Path $ARCHIVE_DIR | Out-Null }

# Convert any generated TGA files to PNG for easier viewing
Write-LogMessage "Converting TGA screenshots to PNG..."
$tgaFiles = @(
    "screenshot_ref-cpu_$MAP_NAME.tga",
    "screenshot_cpu_$MAP_NAME.tga",
    "screenshot_gpu_$MAP_NAME.tga"
)
$tgaArgs = @()
foreach ($tga in $tgaFiles) {
    if (Test-Path $tga) {
        $tgaArgs += $tga
    }
}
if ($tgaArgs.Count -gt 0) {
    python tga2png.py $tgaArgs | Write-Host
}

$timestamp = (Get-Date).ToString("yyyy-MM-ddTHH-mm-ss")
$RUN_DIR = "$ARCHIVE_DIR\${timestamp}_$ARCHIVE_SUFFIX"
New-Item -ItemType Directory -Path $RUN_DIR | Out-Null

$logMap = @{
    $LOG_FILE = "test_$ARCHIVE_SUFFIX.log"
    $REF_LOG  = "vrad_ref-cpu.log"
    $CPU_LOG  = "vrad_cpu.log"
    $GPU_LOG  = "vrad_gpu.log"
}
foreach ($entry in $logMap.GetEnumerator()) {
    if (Test-Path $entry.Key) {
        Move-Item $entry.Key "$RUN_DIR\$($entry.Value)" -Force
    }
}

# Archive PNG screenshots alongside logs
$pngFiles = @(
    "screenshot_ref-cpu_$MAP_NAME.png",
    "screenshot_cpu_$MAP_NAME.png",
    "screenshot_gpu_$MAP_NAME.png",
    "screenshot_diff_ref_cpu_$MAP_NAME-alpha.png",
    "screenshot_diff_cpu_gpu_$MAP_NAME-alpha.png",
    "screenshot_diff_ref_cpu_$MAP_NAME-alphacontrast.png",
    "screenshot_diff_cpu_gpu_$MAP_NAME-alphacontrast.png"
)
foreach ($png in $pngFiles) {
    if (Test-Path $png) {
        Copy-Item $png "$RUN_DIR\$png" -Force
    }
}
Write-LogMessage "Logs and screenshots archived to $RUN_DIR"

if ($testFailed -or $timedOut) { exit 1 }
