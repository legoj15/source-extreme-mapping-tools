# VVIS-OPTIX Three-Way Test Harness
# Shared engine for all validation tests. Called by run_vvis_tests.ps1 with a
# per-test configuration hashtable.
#
# Three-Way Strategy:
#   ref-cpu  = vvis.exe        (SDK reference)   -> ground truth
#   cpu      = vvis_optix.exe   (CPU path)         -> must match ref-cpu (bit-perfect)
#   gpu      = vvis_optix.exe   -cuda (GPU path)   -> must match cpu (visual parity)

param(
    [Parameter(Mandatory)][hashtable]$TestConfig,
    [int]$TimeoutExtensionMinutes = 0,
    [switch]$SkipVisualCheck = $false
)

# --- Unpack test config ---
$MAP_NAME = $TestConfig.MapName
$MOD_DIR = $TestConfig.ModDir
$VBSP_SOURCE = $TestConfig.VbspSource          # "local" or "sdk"
$EXTRA_VBSP_ARGS = $TestConfig.ExtraVbspArgs        # array of strings
$TIMEOUT_MULT = $TestConfig.TimeoutMultiplier
$MIN_TIMEOUT = $TestConfig.MinTimeoutSeconds
$REF_CPU_TOL = $TestConfig.RefCpuTolerance
$CPU_GPU_TOL = $TestConfig.CpuGpuTolerance
$ARCHIVE_SUFFIX = $TestConfig.ArchiveSuffix

# --- Constants ---
$SDK_BIN = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\bin\x64"
$SCRIPT_DIR = $PSScriptRoot  # Absolute path to game\bin\x64 in the repo
$LOG_FILE = Join-Path $SCRIPT_DIR "test_vvis_optix_$ARCHIVE_SUFFIX.log"

# All test paths must be absolute so they work from any CWD
$REF_DIR = Join-Path $SCRIPT_DIR "bsp_unit_tests\ref-cpu"
$CONTROL_DIR = Join-Path $SCRIPT_DIR "bsp_unit_tests\cpu"
$TEST_DIR = Join-Path $SCRIPT_DIR "bsp_unit_tests\gpu"

$REF_LOG = Join-Path $REF_DIR "$MAP_NAME.log"
$CONTROL_LOG = Join-Path $CONTROL_DIR "$MAP_NAME.log"
$CUDA_LOG = Join-Path $TEST_DIR "$MAP_NAME.log"

$GAME_EXE = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\hl2.exe"
$GAME_MAPS = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\sourcetest\maps"
$GAME_SCREENSHOTS = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\sourcetest\screenshots"

# --- Resolve VBSP path ---
if ($VBSP_SOURCE -eq "local") {
    $VBSP_EXE = ".\vbsp.exe"
}
else {
    $VBSP_EXE = "$SDK_BIN\vbsp.exe"
}

# --- Process cleanup ---
$tools = @("vbsp", "vvis", "vrad", "vvis_optix", "vrad_rtx")
foreach ($tool in $tools) {
    Get-Process -Name $tool -ErrorAction SilentlyContinue | Stop-Process -Force
}

# --- Logging ---
"" | Out-File -FilePath $LOG_FILE -Encoding utf8

# --- Deploy vvis_optix binary and DLL to SDK directory ---
# Source engine tools load DLLs relative to the executable's directory.
# vvis_optix.exe needs vvis_optix_dll.dll alongside it in the SDK's bin\x64.
$srcExe = Join-Path $SCRIPT_DIR "vvis_optix.exe"
$srcDll = Join-Path $SCRIPT_DIR "vvis_optix_dll.dll"
$srcPtx = Join-Path $SCRIPT_DIR "vvis_optix.ptx"
$dstExe = Join-Path $SDK_BIN "vvis_optix.exe"
$dstDll = Join-Path $SDK_BIN "vvis_optix_dll.dll"
$dstPtx = Join-Path $SDK_BIN "vvis_optix.ptx"
Copy-Item $srcExe $dstExe -Force
if (Test-Path $srcDll) { Copy-Item $srcDll $dstDll -Force }
if (Test-Path $srcPtx) { Copy-Item $srcPtx $dstPtx -Force }

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
    $gameArgs = "-game", "sourcetest", "-novid", "+sv_cheats 1", "+mat_wireframe 3", "+map $MAP_NAME", "+cl_mouselook 0", "+r_drawportals 1", "+cl_drawhud 0", "+r_drawviewmodel 0", "+mat_leafvis 3", "+wait 1000", "+screenshot", "+quit"
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

# --- Visual comparison helper ---
function Compare-Visual {
    param([string]$Label, [string]$Tga1, [string]$Tga2, [string]$DiffTga, [double]$Tolerance)

    Write-LogMessage "Comparing screenshots using tgadiff ($Label)..."
    $tgadiffExe = Join-Path $SCRIPT_DIR "tgadiff.exe"
    if (!(Test-Path $tgadiffExe)) {
        Write-LogMessage "WARNING: tgadiff.exe not found at $tgadiffExe. Skipping visual comparison."
        return
    }
    $diffOutput = & $tgadiffExe $Tga1 $Tga2 $DiffTga 2>&1
    $diffOutput | Out-File -FilePath $LOG_FILE -Append -Encoding utf8

    $diffMatch = $diffOutput | Select-String "Difference: ([\d\.]+)%"
    if ($diffMatch) {
        $percentDiff = [double]$diffMatch.Matches.Groups[1].Value
        Write-LogMessage "Visual Difference ($Label): $percentDiff%"
        if ($percentDiff -ge $Tolerance) {
            Write-LogMessage "RESULT: FAIL (Visual difference $percentDiff% >= ${Tolerance}%)"
            throw "FAIL"
        }
        else {
            Write-LogMessage "RESULT: PASS (Visual difference $percentDiff% < ${Tolerance}%)"
        }
    }
    else {
        Write-LogMessage "CRITICAL ERROR: Could not parse tgadiff output ($Label)."
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
    foreach ($dir in @($REF_DIR, $CONTROL_DIR, $TEST_DIR)) {
        if (!(Test-Path $dir)) { New-Item -ItemType Directory -Path $dir | Out-Null }
    }

    # --- Phase 0: VBSP Compilation ---
    # Compile the VMF once, then copy the resulting BSP to all three test directories.
    # Skip recompilation if the VMF and test manifest haven't changed.
    $VMF_SRC = Join-Path $SCRIPT_DIR "bsp_unit_tests\$MAP_NAME.vmf"
    if (!(Test-Path $VMF_SRC)) {
        Write-LogMessage "CRITICAL ERROR: Source VMF $VMF_SRC not found!"
        throw "FAIL"
    }

    $MANIFEST_PATH = Join-Path $PSScriptRoot "vvis_tests.psd1"
    $VBSP_HASH_FILE = Join-Path $CONTROL_DIR "$MAP_NAME.vbsp_hash"
    $currentVbspHash = (Get-FileHash $VMF_SRC -Algorithm SHA256).Hash + ":" + (Get-FileHash $MANIFEST_PATH -Algorithm SHA256).Hash
    $cachedVbspHash = if (Test-Path $VBSP_HASH_FILE) { (Get-Content $VBSP_HASH_FILE -Raw).Trim() } else { "" }
    $needsVbspCompile = ($currentVbspHash -ne $cachedVbspHash) -or !(Test-Path "$CONTROL_DIR\$MAP_NAME.bsp") -or !(Test-Path "$CONTROL_DIR\$MAP_NAME.prt")

    Write-LogMessage "--- Phase 0: Compiling map (VBSP) ---"

    if ($needsVbspCompile) {
        # Copy VMF to control dir for compilation
        Copy-Item $VMF_SRC "$CONTROL_DIR\$MAP_NAME.vmf" -Force
        # Also copy .vmx if it exists (Hammer auto-save companion)
        $vmxSrc = Join-Path $SCRIPT_DIR "bsp_unit_tests\$MAP_NAME.vmx"
        if (Test-Path $vmxSrc) { Copy-Item $vmxSrc "$CONTROL_DIR\$MAP_NAME.vmx" -Force }

        if (!(Test-Path $VBSP_EXE)) {
            Write-LogMessage "CRITICAL ERROR: vbsp.exe not found at $VBSP_EXE"
            throw "FAIL"
        }

        $vbspLogFile = Join-Path $CONTROL_DIR "vbsp_$MAP_NAME.log"
        $fullLogPath = $vbspLogFile
        Write-LogMessage "VBSP Log: $fullLogPath"

        # Source engine tools need the SDK's bin\x64 as CWD for DLL loading.
        # Use Start-Process to fully isolate from PowerShell's pipe environment.
        $vbspProc = Start-Process -FilePath $VBSP_EXE `
            -ArgumentList "$($EXTRA_VBSP_ARGS -join ' ') -game `"$MOD_DIR`" `"$CONTROL_DIR\$MAP_NAME`"" `
            -WorkingDirectory $SDK_BIN `
            -PassThru -Wait -NoNewWindow
        $vbspExit = $vbspProc.ExitCode
        if ($vbspExit -ne 0) {
            Write-LogMessage "CRITICAL ERROR: vbsp.exe failed with exit code $vbspExit. See log: $fullLogPath"
            throw "FAIL"
        }

        # Save hash after successful compilation
        $currentVbspHash | Out-File -FilePath $VBSP_HASH_FILE -Encoding utf8 -NoNewline

        # Distribute the compiled BSP and PRT (portal file) to all three directories
        # VVIS requires the .prt file that VBSP generates alongside the BSP.
        foreach ($destDir in @($REF_DIR, $TEST_DIR)) {
            Copy-Item "$CONTROL_DIR\$MAP_NAME.bsp" "$destDir\$MAP_NAME.bsp" -Force
            $prtFile = "$CONTROL_DIR\$MAP_NAME.prt"
            if (Test-Path $prtFile) {
                Copy-Item $prtFile "$destDir\$MAP_NAME.prt" -Force
            }
        }
    }
    else {
        Write-LogMessage "VBSP unchanged (VMF and manifest identical), using cached result."
        # Still distribute cached BSP and PRT to ref/test directories
        foreach ($destDir in @($REF_DIR, $TEST_DIR)) {
            Copy-Item "$CONTROL_DIR\$MAP_NAME.bsp" "$destDir\$MAP_NAME.bsp" -Force
            $prtFile = "$CONTROL_DIR\$MAP_NAME.prt"
            if (Test-Path $prtFile) {
                Copy-Item $prtFile "$destDir\$MAP_NAME.prt" -Force
            }
        }
    }

    # --- Phase 1: Reference Run (vvis.exe from SDK — ref-cpu) ---
    # Skip recompilation if the compiled BSP and test manifest haven't changed.
    $HasReference = Test-Path "$SDK_BIN\vvis.exe"

    $REF_HASH_FILE = Join-Path $REF_DIR "$MAP_NAME.ref_hash"
    $REF_TIME_FILE = Join-Path $REF_DIR "$MAP_NAME.ref_time"
    # Reuse VMF+manifest hash — the CONTROL_DIR BSP gets modified in-place by the
    # Phase 2 cpu VVIS run, so hashing it would always produce a different value on
    # the next run, defeating the cache.
    $currentRefHash = $currentVbspHash
    $cachedRefHash = if (Test-Path $REF_HASH_FILE) { (Get-Content $REF_HASH_FILE -Raw).Trim() } else { "" }
    $needsRefCompile = ($currentRefHash -ne $cachedRefHash) -or !(Test-Path "$REF_DIR\$MAP_NAME.bsp")

    $refTime = New-TimeSpan
    $refTimeCached = $false
    if ($HasReference) {
        if ($needsRefCompile) {
            Write-LogMessage "--- Phase 1: ref-cpu (SDK vvis.exe) ---"
            Remove-Item $REF_LOG -ErrorAction SilentlyContinue

            $fullLogPath = $REF_LOG
            Write-LogMessage "vvis.exe (SDK Reference) Log: $fullLogPath"

            $start = Get-Date
            $refProc = Start-Process -FilePath "$SDK_BIN\vvis.exe" `
                -ArgumentList "-game `"$MOD_DIR`" `"$REF_DIR\$MAP_NAME`"" `
                -WorkingDirectory $SDK_BIN `
                -PassThru -Wait -NoNewWindow
            $refExit = $refProc.ExitCode
            if ($refExit -ne 0) {
                Write-LogMessage "WARNING: vvis.exe (ref-cpu) failed with exit code $refExit. Skipping reference comparison."
                $HasReference = $false
            }
            else {
                $refTime = (Get-Date) - $start
                $currentRefHash | Out-File -FilePath $REF_HASH_FILE -Encoding utf8 -NoNewline
                $refTime.TotalSeconds.ToString("F2") | Out-File -FilePath $REF_TIME_FILE -Encoding utf8 -NoNewline
            }
        }
        else {
            Write-LogMessage "--- Phase 1: ref-cpu (SDK vvis.exe) ---"
            Write-LogMessage "Reference unchanged (BSP and manifest identical), using cached result."
            if (Test-Path $REF_TIME_FILE) {
                $refTime = [TimeSpan]::FromSeconds([double](Get-Content $REF_TIME_FILE -Raw).Trim())
                $refTimeCached = $true
            }
        }
    }
    else {
        Write-LogMessage "SDK vvis.exe not found at $SDK_BIN\vvis.exe, skipping ref-cpu pass."
    }

    # --- Phase 2: Control Run (vvis_optix.exe CPU — cpu) ---
    Write-LogMessage "--- Phase 2: cpu (vvis_optix.exe, CPU path) ---"
    Remove-Item $CONTROL_LOG -ErrorAction SilentlyContinue

    $fullLogPath = $CONTROL_LOG
    Write-LogMessage "vvis_optix.exe CPU Log: $fullLogPath"

    $start = Get-Date
    # Use Start-Process to fully isolate vvis_optix.exe from PowerShell's pipe
    # environment. Source engine tools crash with heap corruption (0xC0000374)
    # when invoked through & in nested script contexts due to inherited handles.
    $cpuProc = Start-Process -FilePath "cmd.exe" `
        -ArgumentList "/c `"cd /d `"$SDK_BIN`" && `"$SDK_BIN\vvis_optix.exe`" -game `"$MOD_DIR`" `"$CONTROL_DIR\$MAP_NAME`"`"" `
        -PassThru -Wait -NoNewWindow
    $cpuExit = $cpuProc.ExitCode
    if ($cpuExit -ne 0) {
        Write-LogMessage "CRITICAL ERROR: vvis_optix.exe (cpu) failed with exit code $cpuExit."
        throw "FAIL"
    }
    $controlTime = (Get-Date) - $start

    # --- Phase 3: ref-cpu vs cpu Parity Check (Early Validation Gate) ---
    if ($HasReference) {
        Write-LogMessage "`n--- Phase 3: Comparing ref-cpu vs cpu (CPU Parity Check) ---"

        $fcOutput = fc.exe /b /LB1 "$REF_DIR\$MAP_NAME.bsp" "$CONTROL_DIR\$MAP_NAME.bsp" 2>&1
        if ($LASTEXITCODE -eq 0) {
            Write-LogMessage "RESULT: PASS (ref-cpu and cpu BSPs are binary identical)"
        }
        else {
            Write-LogMessage "Warning: ref-cpu and cpu BSPs differ at binary level. Checking visibility data..."
            $bspDiffOutput = python (Join-Path $SCRIPT_DIR "bsp_diff_visibility.py") "$REF_DIR\$MAP_NAME.bsp" "$CONTROL_DIR\$MAP_NAME.bsp" 2>&1
            $bspDiffOutput | Out-File -FilePath $LOG_FILE -Append -Encoding utf8
            $bspDiffOutput | Write-Host

            if ($bspDiffOutput -like "*Visibility data is identical.*") {
                Write-LogMessage "RESULT: PASS (BSP bytes differ, but all visibility data is identical)"
            }
            else {
                $tgadiffExe = Join-Path $SCRIPT_DIR "tgadiff.exe"
                $hasVisualTools = (Test-Path $tgadiffExe) -and (Test-Path $GAME_EXE)

                if ($SkipVisualCheck -or -not $hasVisualTools) {
                    if (-not $hasVisualTools) {
                        Write-LogMessage "WARNING: Visual comparison tools not found. Skipping visual check."
                    }
                    Write-LogMessage "WARNING: Visibility data differs between ref-cpu and cpu. Visual comparison skipped."
                }
                else {
                    Write-LogMessage "Visibility data differs. Initiating visual comparison (ref-cpu vs cpu)..."
                    Take-Screenshot "$REF_DIR\$MAP_NAME.bsp" "screenshot_ref-cpu_$MAP_NAME.tga"
                    Take-Screenshot "$CONTROL_DIR\$MAP_NAME.bsp" "screenshot_cpu_$MAP_NAME.tga"
                    Compare-Visual "ref-cpu vs cpu" "screenshot_ref-cpu_$MAP_NAME.tga" "screenshot_cpu_$MAP_NAME.tga" "screenshot_diff_ref_cpu_$MAP_NAME.tga" $REF_CPU_TOL
                }
            }
        }
    }

    # --- Phase 4: GPU Run (vvis_optix.exe -cuda — gpu) ---
    Write-LogMessage "--- Phase 4: gpu (vvis_optix.exe -cuda) ---"
    Remove-Item $CUDA_LOG -ErrorAction SilentlyContinue

    $fullLogPath = $CUDA_LOG
    Write-LogMessage "vvis_optix.exe -cuda Log: $fullLogPath"

    $start = Get-Date

    # Use native Start-Process with file redirection to drain output safely.
    # This prevents pipe buffer (4KB) deadlocks on Windows without needing
    # complex .NET async event handlers.
    $outLogTmp = "$env:TEMP\vvis_optix_cuda_out.txt"
    $errLogTmp = "$env:TEMP\vvis_optix_cuda_err.txt"
    Remove-Item $outLogTmp -ErrorAction SilentlyContinue
    Remove-Item $errLogTmp -ErrorAction SilentlyContinue

    try {
        $process = Start-Process -FilePath (Join-Path $SDK_BIN "vvis_optix.exe") `
            -ArgumentList "-cuda -game `"$MOD_DIR`" `"$TEST_DIR\$MAP_NAME`"" `
            -WorkingDirectory $SDK_BIN `
            -RedirectStandardOutput $outLogTmp `
            -RedirectStandardError $errLogTmp `
            -NoNewWindow -PassThru
    }
    catch {
        Write-LogMessage "CRITICAL ERROR: Failed to start vvis_optix.exe -cuda: $($_.Exception.Message)"
        throw "FAIL"
    }

    if ($null -eq $process) {
        Write-LogMessage "CRITICAL ERROR: Failed to start vvis_optix.exe -cuda. Process object is NULL."
        throw "FAIL"
    }

    $timedOut = $false
    $maxSeconds = [math]::Max($MIN_TIMEOUT, $controlTime.TotalSeconds * $TIMEOUT_MULT) + ($TimeoutExtensionMinutes * 60)
    $hasWarnedByExceedingControl = $false

    while (-not $process.HasExited) {
        $elapsed = (Get-Date) - $start
        if (($elapsed.TotalSeconds -gt $controlTime.TotalSeconds) -and (-not $hasWarnedByExceedingControl)) {
            $remaining = [math]::Max(0, [math]::Round($maxSeconds - $elapsed.TotalSeconds))
            Write-LogMessage "WARNING: gpu run has exceeded the cpu time ($([math]::Round($controlTime.TotalSeconds))s). Will terminate in ${remaining}s."
            $hasWarnedByExceedingControl = $true
        }
        if ($elapsed.TotalSeconds -gt $maxSeconds) {
            $process.Kill()
            $timedOut = $true

            # Sample current usage snapshot
            $currentGpu = nvidia-smi --query-gpu=utilization.gpu --format="csv,noheader,nounits" 2>$null
            $currentCpu = (Get-Counter '\Processor(_Total)\% Processor Time' -ErrorAction SilentlyContinue).CounterSamples.CookedValue

            Write-LogMessage "CRITICAL ERROR: vvis_optix -cuda hung or is significantly slower than cpu!"
            if ($currentCpu) { Write-LogMessage "CPU Usage at time of termination: $($currentCpu.ToString('F1'))%" }
            if ($currentGpu) { Write-LogMessage "GPU Usage at time of termination: $($currentGpu.Trim())%" }
            Write-LogMessage "Notice: You can extend the runtime by adding: -TimeoutExtensionMinutes <minutes>"
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

    # Save captured stderr if any
    if (Test-Path $errLogTmp) {
        $errText = Get-Content $errLogTmp -Raw
        if ($errText -and $errText.Trim()) {
            Copy-Item $errLogTmp "$TEST_DIR\vvis_optix_stderr.txt" -Force
        }
    }

    # Final refresh to ensure exit code is captured
    $exitCode = if ($process.HasExited) { $process.ExitCode } else { $null }

    if (-not $timedOut -and ($null -eq $exitCode -or $exitCode -ne 0)) {
        $errMessage = if ($null -eq $exitCode) { "UNKNOWN (NULL)" } else { $exitCode }
        Write-LogMessage "CRITICAL ERROR: vvis_optix.exe -cuda failed with exit code $errMessage."
        throw "FAIL"
    }
    $cudaTime = (Get-Date) - $start

    # --- Timing Summary ---
    Write-LogMessage "`n--- Timing Summary ---"
    if ($HasReference) {
        $cachedLabel = if ($refTimeCached) { " (cached)" } else { "" }
        Write-LogMessage "vvis.exe Time (ref-cpu):`t`t$($refTime.TotalSeconds.ToString("F2"))s$cachedLabel"
    }
    Write-LogMessage "vvis_optix.exe Time (cpu):`t`t$($controlTime.TotalSeconds.ToString("F2"))s"
    if ($timedOut) {
        Write-LogMessage "vvis_optix.exe -cuda Time (gpu):`tDid not finish"
    }
    else {
        Write-LogMessage "vvis_optix.exe -cuda Time (gpu):`t$($cudaTime.TotalSeconds.ToString("F2"))s"
    }

    # --- Phase 5: cpu vs gpu Parity Check ---
    if (-not $timedOut) {
        Write-LogMessage "`n--- Phase 5: Comparing cpu vs gpu (GPU Parity Check) ---"

        $fcOutput = fc.exe /b /LB1 "$CONTROL_DIR\$MAP_NAME.bsp" "$TEST_DIR\$MAP_NAME.bsp" 2>&1
        if ($LASTEXITCODE -eq 0) {
            Write-LogMessage "RESULT: PASS (cpu and gpu BSPs are binary identical)"
        }
        else {
            Write-LogMessage "Warning: cpu and gpu BSPs differ at binary level. Checking visibility data..."
            $CPU_GPU_TOL = if ($TestConfig.CpuGpuTolerance) { $TestConfig.CpuGpuTolerance } else { 0.0 }
            $bspDiffOutput = python (Join-Path $SCRIPT_DIR "bsp_diff_visibility.py") "$CONTROL_DIR\$MAP_NAME.bsp" "$TEST_DIR\$MAP_NAME.bsp" $CPU_GPU_TOL 2>&1
            $bspDiffOutput | Out-File -FilePath $LOG_FILE -Append -Encoding utf8
            $bspDiffOutput | Write-Host

            if ($bspDiffOutput -match "Visibility data is identical" -or $bspDiffOutput -match "Visibility data meets tolerance") {
                Write-LogMessage "RESULT: PASS (BSP bytes differ, but visibility data is identical or within acceptable tolerance)"
            }
            else {
                $tgadiffExe = Join-Path $SCRIPT_DIR "tgadiff.exe"
                $hasVisualTools = (Test-Path $tgadiffExe) -and (Test-Path $GAME_EXE)

                if ($SkipVisualCheck -or -not $hasVisualTools) {
                    if (-not $hasVisualTools) {
                        Write-LogMessage "WARNING: Visual comparison tools not found (tgadiff.exe or hl2.exe). Skipping visual check."
                    }
                    Write-LogMessage "CRITICAL ERROR: Visibility data differs between cpu and gpu. Visual comparison skipped. Marking as FAIL."
                    throw "FAIL"
                }
                else {
                    Write-LogMessage "Visibility data differs. Initiating visual comparison (cpu vs gpu)..."
                    if (!(Test-Path "screenshot_cpu_$MAP_NAME.tga")) {
                        Take-Screenshot "$CONTROL_DIR\$MAP_NAME.bsp" "screenshot_cpu_$MAP_NAME.tga"
                    }
                    Take-Screenshot "$TEST_DIR\$MAP_NAME.bsp" "screenshot_gpu_$MAP_NAME.tga"
                    Compare-Visual "cpu vs gpu" "screenshot_cpu_$MAP_NAME.tga" "screenshot_gpu_$MAP_NAME.tga" "screenshot_diff_cpu_gpu_$MAP_NAME.tga" $CPU_GPU_TOL
                }
            }
        }
    }

}
catch {
    $testFailed = $true
}

# --- Archive Logs ---
$ARCHIVE_DIR = Join-Path $SCRIPT_DIR "bsp_unit_test_logs"
if (!(Test-Path $ARCHIVE_DIR)) { New-Item -ItemType Directory -Path $ARCHIVE_DIR | Out-Null }

# Convert any generated TGA files to PNG for easier viewing
$tgaFiles = @(
    "screenshot_ref-cpu_$MAP_NAME.tga",
    "screenshot_cpu_$MAP_NAME.tga",
    "screenshot_gpu_$MAP_NAME.tga",
    "screenshot_diff_ref_cpu_$MAP_NAME.tga",
    "screenshot_diff_cpu_gpu_$MAP_NAME.tga"
)
$tgaArgs = @()
foreach ($tga in $tgaFiles) {
    if (Test-Path $tga) {
        $tgaArgs += $tga
    }
}
$tga2png = Join-Path $SCRIPT_DIR "tga2png.py"
if (($tgaArgs.Count -gt 0) -and (Test-Path $tga2png)) {
    python $tga2png $tgaArgs | Write-Host
}

$timestamp = (Get-Date).ToString("yyyy-MM-ddTHH-mm-ss")
$RUN_DIR = "$ARCHIVE_DIR\${timestamp}_$ARCHIVE_SUFFIX"
New-Item -ItemType Directory -Path $RUN_DIR | Out-Null

$logMap = @{
    $LOG_FILE    = "test_$ARCHIVE_SUFFIX.log"
    $REF_LOG     = "vvis_ref-cpu.log"
    $CONTROL_LOG = "vvis_cpu.log"
    $CUDA_LOG    = "vvis_gpu.log"
}
foreach ($entry in $logMap.GetEnumerator()) {
    if (Test-Path $entry.Key) {
        Copy-Item $entry.Key "$RUN_DIR\$($entry.Value)" -Force
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
