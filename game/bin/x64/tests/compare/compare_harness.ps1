# Three-Way Test Harness for the vrad and vvis comparison tests
# Shared engine for all validation tests. Called by run_compare_tests.ps1 with a
# tool name (vrad or vvis) and a per-test configuration hashtable.
#
# Three-way strategy (both tools):
#   ref-cpu  = the SDK reference binary (vrad.exe / vvis.exe)  -> the result to stay close to
#   cpu      = our binary, CPU only (vrad_rtx.exe / vvis_optix.exe -threads ControlThreads)
#              -> compared to ref-cpu (RefCpuTolerance)
#   gpu      = vrad_rtx.exe -cuda / vvis_optix.exe with all threads -> compared to cpu (CpuGpuTolerance)
# vvis: the PVS does not depend on the thread count (CpuGpuTolerance is 0), so its cpu and gpu legs differ
# only in -threads. The "gpu" leg name, log names and headings are historical: VVIS has no GPU path since #46.
#
# Layout of this file: the shared flow (config, logging, screenshots, the supervised gpu run, the log archive)
# first, then one clearly marked block of tool-specific step functions per tool (Invoke-VradXxx, Invoke-VvisXxx),
# then the main flow, which runs the same phases for both tools and calls the tool's step for each phase.
# The steps: Prepare (maps into place), RefRun, CpuRun, CompareRefCpu, GpuRunSpec, GpuTimeout, CompareCpuGpu.

param(
    [Parameter(Mandatory)][ValidateSet('vrad', 'vvis')][string]$Tool,
    [Parameter(Mandatory)][hashtable]$TestConfig,
    [int]$TimeoutExtensionMinutes = 0,
    [switch]$SkipVisualCheck = $false
)

# --- Unpack test config (keys both manifests have) ---
$MAP_NAME = $TestConfig.MapName
$TIMEOUT_MULT = $TestConfig.TimeoutMultiplier
$REF_CPU_TOL = $TestConfig.RefCpuTolerance
$CPU_GPU_TOL = $TestConfig.CpuGpuTolerance
$ARCHIVE_SUFFIX = $TestConfig.ArchiveSuffix

# --- Constants ---
$SDK_ROOT = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer"
$SDK_BIN = "$SDK_ROOT\bin\x64"
$GAME_EXE = "$SDK_ROOT\hl2.exe"
$GAME_MAPS = "$SDK_ROOT\sourcetest\maps"
$GAME_SCREENSHOTS = "$SDK_ROOT\sourcetest\screenshots"

# --- Locations (this file is game\bin\x64\tests\compare) ---
$TESTS_DIR = Split-Path -Parent $PSScriptRoot
$BIN_DIR = Split-Path -Parent $TESTS_DIR    # game\bin\x64: the tools load their PTX from the working directory
$MAPS_DIR = Join-Path $PSScriptRoot "maps"
$WORK_DIR = Join-Path $TESTS_DIR "out\compare"   # per-run logs and screenshots; also the working directory (see Push-Location around each tool)
$IMAGE_COMPARE = Join-Path $PSScriptRoot "image_compare.py"
New-Item -ItemType Directory -Path $WORK_DIR -Force | Out-Null
Push-Location $WORK_DIR
# Every exit, a throw included, puts the caller back where it started.
trap { Pop-Location; break }

# All test paths must be absolute so they work from any CWD
$REF_DIR = Join-Path $MAPS_DIR "ref-cpu"
$CPU_DIR = Join-Path $MAPS_DIR "cpu"
$GPU_DIR = Join-Path $MAPS_DIR "gpu"

$REF_LOG = "$REF_DIR\$MAP_NAME.log"
$CPU_LOG = "$CPU_DIR\$MAP_NAME.log"
$GPU_LOG = "$GPU_DIR\$MAP_NAME.log"

# --- Per-tool constants: everything that differs between the two tools without being code ---
switch ($Tool) {
    'vrad' {
        $EXTRA_ARGS = $TestConfig.ExtraArgs          # array of strings
        $LIGHTMAP_THRESH = $TestConfig.LightmapThreshold
        $MOD_DIR = "$SDK_ROOT\sourcetest"
        $MIN_TIMEOUT = 0
        $P = @{
            Title             = 'Vrad'
            LogFile           = Join-Path $WORK_DIR "test_vrad_rtx_$ARCHIVE_SUFFIX.log"
            # The BSPs are git-ignored fixtures in tests\maps (the VMFs here are tracked); take
            # one from tests\compare\maps first if someone put it there.
            SourceFile        = @((Join-Path $MAPS_DIR "$MAP_NAME.bsp"), (Join-Path $TESTS_DIR "maps\$MAP_NAME.bsp")) |
                                Where-Object { Test-Path $_ } | Select-Object -First 1
            SourceKind        = 'map'
            # The SDK install's own vrad.exe, run from its own folder (it loads vrad_dll.dll from there),
            # like the vvis reference. It used to be looked for in game\bin\x64, where nothing puts it.
            RefExe            = "$SDK_BIN\vrad.exe"
            RefMissingMessage = "SDK vrad.exe not found at $SDK_BIN\vrad.exe, skipping ref-cpu pass."
            RefCachedMessages = @("ref-cpu unchanged (BSP and script identical), using cached result.")
            GameArgs          = @("-game", "sourcetest", "-novid", "-sw", "-w", "2560", "-h", "1440", "+sv_cheats 1", "+map $MAP_NAME", "+cl_mouselook 0", "+cl_drawhud 0", "+r_drawviewmodel 0", "+mat_fullbright 2", "+wait 1000", "+screenshot", "+quit")
            GpuLabel          = "vrad_rtx.exe -cuda"
            GpuTempName       = "vrad_rtx_cuda"
            GpuOverrunText    = "Test run has exceeded the cpu test time"
            GpuStderrFile     = $null
            TimingRef         = "vrad.exe Time (Source SDK 2013 Unmodified):`t"
            TimingCpu         = "vrad_rtx.exe Time (CPU only):`t`t`t"
            TimingGpu         = "vrad_rtx.exe -cuda Time (GPU accelerated):`t"
            TgaFiles          = @("screenshot_ref-cpu_$MAP_NAME.tga", "screenshot_cpu_$MAP_NAME.tga", "screenshot_gpu_$MAP_NAME.tga")
            ArchiveMoveLogs   = $true                # vrad moves its per-map logs into the archive, vvis copies them
        }
    }
    'vvis' {
        $MOD_DIR = $TestConfig.ModDir
        $VBSP_SOURCE = $TestConfig.VbspSource          # "local" or "sdk"
        $EXTRA_VBSP_ARGS = $TestConfig.ExtraVbspArgs        # array of strings
        $MIN_TIMEOUT = $TestConfig.MinTimeoutSeconds
        $CONTROL_THREADS = if ($TestConfig.ControlThreads) { $TestConfig.ControlThreads } else { 1 }
        $CPU_GPU_TOL = if ($TestConfig.CpuGpuTolerance) { $TestConfig.CpuGpuTolerance } else { 0.0 }
        $VBSP_EXE = if ($VBSP_SOURCE -eq "local") { Join-Path $BIN_DIR "vbsp.exe" } else { "$SDK_BIN\vbsp.exe" }
        $P = @{
            Title             = 'Vvis'
            LogFile           = Join-Path $WORK_DIR "test_vvis_optix_$ARCHIVE_SUFFIX.log"
            SourceFile        = Join-Path $MAPS_DIR "$MAP_NAME.vmf"
            SourceKind        = 'VMF'
            RefExe            = "$SDK_BIN\vvis.exe"
            RefMissingMessage = "SDK vvis.exe not found at $SDK_BIN\vvis.exe, skipping ref-cpu pass."
            RefCachedMessages = @("--- Phase 1: ref-cpu (SDK vvis.exe) ---", "Reference unchanged (BSP and manifest identical), using cached result.")
            GameArgs          = @("-game", "sourcetest", "-novid", "+sv_cheats 1", "+mat_wireframe 3", "+map $MAP_NAME", "+cl_mouselook 0", "+r_drawportals 1", "+cl_drawhud 0", "+r_drawviewmodel 0", "+mat_leafvis 3", "+wait 1000", "+screenshot", "+quit")
            GpuLabel          = "vvis_optix.exe (default flow)"
            GpuTempName       = "vvis_optix_cuda"
            GpuOverrunText    = "gpu run has exceeded the cpu time"
            GpuStderrFile     = "vvis_optix_stderr.txt"
            TimingRef         = "vvis.exe Time (ref-cpu):`t`t"
            TimingCpu         = "vvis_optix.exe Time (cpu):`t`t"
            TimingGpu         = "vvis_optix.exe (default flow) Time (gpu):`t"
            TgaFiles          = @("screenshot_ref-cpu_$MAP_NAME.tga", "screenshot_cpu_$MAP_NAME.tga", "screenshot_gpu_$MAP_NAME.tga", "screenshot_diff_ref_cpu_$MAP_NAME.tga", "screenshot_diff_cpu_gpu_$MAP_NAME.tga")
            ArchiveMoveLogs   = $false
        }
    }
}
$LOG_FILE = $P.LogFile

# What the steps hand back to the main flow
$State = @{
    HasReference  = $false
    RefTime       = New-TimeSpan
    RefTimeCached = $false
    CpuTime       = New-TimeSpan
    GpuTime       = New-TimeSpan
    TimedOut      = $false
}

# --- Process cleanup ---
# (PowerShell variable names are case-insensitive: the loop variable must not be called $tool, the -Tool parameter.)
$toolProcesses = @("vbsp", "vvis", "vrad", "vvis_optix", "vrad_rtx")
foreach ($processName in $toolProcesses) {
    Get-Process -Name $processName -ErrorAction SilentlyContinue | Stop-Process -Force
}

# --- Logging ---
"" | Out-File -FilePath $LOG_FILE -Encoding utf8

# ============================================================
#  Shared helpers
# ============================================================

function Write-LogMessage {
    param([string]$Message, [bool]$ToLog = $true)
    Write-Host $Message
    if ($ToLog) {
        $Message | Out-File -FilePath $LOG_FILE -Append -Encoding utf8
    }
}

# Runs the tool's step function for this phase: Invoke-<Vrad|Vvis><Step>
function Invoke-ToolStep {
    param([Parameter(Mandatory)][string]$Step)
    & "Invoke-$($P.Title)$Step"
}

# --- Screenshot helper ---
function Take-Screenshot {
    param([string]$BspPath, [string]$TargetTga)

    Write-LogMessage "Taking screenshot for $BspPath..."

    # 1. Copy bsp
    Copy-Item $BspPath "$GAME_MAPS\$MAP_NAME.bsp" -Force

    # 2. Run hl2.exe
    $proc = Start-Process -FilePath $GAME_EXE -ArgumentList $P.GameArgs -Wait -PassThru
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

# --- The gpu run: start the process the tool's GpuRunSpec step describes, watch it against the timeout ---
# Sets $State.TimedOut and $State.GpuTime; throws "FAIL" on a failed start or a bad exit code.
function Invoke-GpuRun {
    param([Parameter(Mandatory)][hashtable]$Spec)

    $start = Get-Date

    # Use native Start-Process with file redirection to drain output safely.
    # This prevents pipe buffer (4KB) deadlocks on Windows without needing
    # complex .NET async event handlers.
    $outLogTmp = "$env:TEMP\$($P.GpuTempName)_out.txt"
    $errLogTmp = "$env:TEMP\$($P.GpuTempName)_err.txt"
    Remove-Item $outLogTmp -ErrorAction SilentlyContinue
    Remove-Item $errLogTmp -ErrorAction SilentlyContinue

    try {
        $process = Start-Process -FilePath $Spec.FilePath `
            -ArgumentList $Spec.ArgumentList `
            -WorkingDirectory $Spec.WorkingDirectory `
            -RedirectStandardOutput $outLogTmp `
            -RedirectStandardError $errLogTmp `
            -NoNewWindow -PassThru
    }
    catch {
        Write-LogMessage "CRITICAL ERROR: Failed to start $($P.GpuLabel): $($_.Exception.Message)"
        throw "FAIL"
    }

    if ($null -eq $process) {
        Write-LogMessage "CRITICAL ERROR: Failed to start $($P.GpuLabel). Process object is NULL."
        throw "FAIL"
    }

    $maxSeconds = [math]::Max($MIN_TIMEOUT, $State.CpuTime.TotalSeconds * $TIMEOUT_MULT) + ($TimeoutExtensionMinutes * 60)
    $hasWarnedByExceedingControl = $false

    while (-not $process.HasExited) {
        $elapsed = (Get-Date) - $start
        if (($elapsed.TotalSeconds -gt $State.CpuTime.TotalSeconds) -and (-not $hasWarnedByExceedingControl)) {
            $remaining = [math]::Max(0, [math]::Round($maxSeconds - $elapsed.TotalSeconds))
            Write-LogMessage "WARNING: $($P.GpuOverrunText) ($([math]::Round($State.CpuTime.TotalSeconds))s). Will terminate in ${remaining}s."
            $hasWarnedByExceedingControl = $true
        }
        if ($elapsed.TotalSeconds -gt $maxSeconds) {
            $process.Kill()
            $State.TimedOut = $true
            Invoke-ToolStep 'GpuTimeout'
            break
        }
        Start-Sleep -Seconds 1
    }

    if (-not $State.TimedOut) {
        $process.WaitForExit()
    }

    # Append standard output to full log
    if (Test-Path $outLogTmp) {
        Get-Content $outLogTmp | Out-File -FilePath $GPU_LOG -Append -Encoding utf8
    }

    # Save captured stderr if any (vvis only)
    if ($P.GpuStderrFile -and (Test-Path $errLogTmp)) {
        $errText = Get-Content $errLogTmp -Raw
        if ($errText -and $errText.Trim()) {
            Copy-Item $errLogTmp "$GPU_DIR\$($P.GpuStderrFile)" -Force
        }
    }

    # Final refresh to ensure exit code is captured
    $exitCode = if ($process.HasExited) { $process.ExitCode } else { $null }

    if (-not $State.TimedOut -and ($null -eq $exitCode -or $exitCode -ne 0)) {
        $errMessage = if ($null -eq $exitCode) { "UNKNOWN (NULL)" } else { $exitCode }
        Write-LogMessage "CRITICAL ERROR: $($P.GpuLabel) failed with exit code $errMessage."
        throw "FAIL"
    }
    $State.GpuTime = (Get-Date) - $start
}

# ============================================================
#  VRAD steps
#    ref-cpu = vrad.exe (SDK), cpu = vrad_rtx.exe, gpu = vrad_rtx.exe -cuda
#    Compared on the lighting lump (image_compare.py lightmaps), then on screenshots (image_compare.py ssim).
# ============================================================

# The source map is a compiled BSP: give the cpu and gpu legs their own copy.
function Invoke-VradPrepare {
    Copy-Item $P.SourceFile "$CPU_DIR\$MAP_NAME.bsp" -Force
    Copy-Item $P.SourceFile "$GPU_DIR\$MAP_NAME.bsp" -Force
}

# Returns the time it took, or $null when vrad.exe failed (the reference is then skipped).
function Invoke-VradRefRun {
    Copy-Item $P.SourceFile "$REF_DIR\$MAP_NAME.bsp" -Force
    Write-LogMessage "--- Compiling ref-cpu map (Original vrad.exe) ---"
    Write-LogMessage "vrad.exe (SDK Reference) Log: $REF_LOG"
    $start = Get-Date
    Push-Location $SDK_BIN
    & $P.RefExe @EXTRA_ARGS -game $MOD_DIR "$REF_DIR\$MAP_NAME" *>$null
    Pop-Location
    if ($LASTEXITCODE -ne 0) {
        Write-LogMessage "WARNING: vrad.exe (reference) failed with exit code $LASTEXITCODE. Skipping ref-cpu comparison."
        return $null
    }
    return (Get-Date) - $start
}

# Returns the time it took; throws "FAIL" when vrad_rtx.exe fails.
function Invoke-VradCpuRun {
    Write-LogMessage "--- Compiling cpu map (vrad_rtx.exe CPU only) ---"
    Write-LogMessage "vrad_rtx.exe CPU Log: $CPU_LOG"
    $start = Get-Date
    Push-Location $BIN_DIR
    & ".\vrad_rtx.exe" @EXTRA_ARGS -game $MOD_DIR "$CPU_DIR\$MAP_NAME" *>$null
    Pop-Location
    if ($LASTEXITCODE -ne 0) {
        Write-LogMessage "CRITICAL ERROR: vrad_rtx.exe (control, CPU only) failed with exit code $LASTEXITCODE."
        throw "FAIL"
    }
    return (Get-Date) - $start
}

function Invoke-VradCompareRefCpu {
    Write-LogMessage "`n--- Comparing ref-cpu vs cpu (CPU Parity Check) ---"
    $pythonDiff = python -I $IMAGE_COMPARE lightmaps "$REF_DIR\$MAP_NAME.bsp" "$CPU_DIR\$MAP_NAME.bsp" --threshold $LIGHTMAP_THRESH 2>&1
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
            if ($State.NeedsRefCompile -or !(Test-Path "screenshot_ref-cpu_$MAP_NAME.tga")) {
                Take-Screenshot "$REF_DIR\$MAP_NAME.bsp" "screenshot_ref-cpu_$MAP_NAME.tga"
            }
            else {
                Write-LogMessage "Using cached ref-cpu screenshot."
            }
            Take-Screenshot "$CPU_DIR\$MAP_NAME.bsp" "screenshot_cpu_$MAP_NAME.tga"

            $diffOutput = python -I $IMAGE_COMPARE ssim screenshot_ref-cpu_$MAP_NAME.tga screenshot_cpu_$MAP_NAME.tga screenshot_diff_ref_cpu_$MAP_NAME 2>&1
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

function Invoke-VradGpuRunSpec {
    Write-LogMessage "--- Compiling gpu map (vrad_rtx.exe -cuda) ---"
    Write-LogMessage "vrad_rtx.exe -cuda Log: $GPU_LOG"
    return @{
        FilePath         = Join-Path $BIN_DIR "vrad_rtx.exe"
        ArgumentList     = @($EXTRA_ARGS) + @("-game", "`"$MOD_DIR`"", "-cuda", "`"$GPU_DIR\$MAP_NAME`"")
        WorkingDirectory = $BIN_DIR
    }
}

function Invoke-VradGpuTimeout {
    Write-LogMessage "CRITICAL ERROR: vrad_rtx -cuda hung or is significantly slower than cpu test! Use -TimeoutExtensionMinutes <minutes> to extend wait time."
}

function Invoke-VradCompareCpuGpu {
    Write-LogMessage "`n--- Comparing cpu vs gpu (GPU Parity Check) ---"
    $pythonDiff = python -I $IMAGE_COMPARE lightmaps "$CPU_DIR\$MAP_NAME.bsp" "$GPU_DIR\$MAP_NAME.bsp" --threshold $LIGHTMAP_THRESH 2>&1
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

            $diffOutput = python -I $IMAGE_COMPARE ssim screenshot_cpu_$MAP_NAME.tga screenshot_gpu_$MAP_NAME.tga screenshot_diff_cpu_gpu_$MAP_NAME 2>&1
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

# ============================================================
#  VVIS steps
#    ref-cpu = vvis.exe (SDK), cpu = vvis_optix.exe -threads ControlThreads, gpu = vvis_optix.exe (all threads)
#    The source map is a VMF: VBSP compiles it first. Compared on the visibility lump
#    (bsp_diff_visibility.py), then on screenshots (tgadiff.exe).
# ============================================================

# Source engine tools load DLLs relative to the executable's directory:
# vvis_optix.exe needs vvis_optix_dll.dll alongside it in the SDK's bin\x64.
function Install-VvisBinaries {
    $srcExe = Join-Path $BIN_DIR "vvis_optix.exe"
    $srcDll = Join-Path $BIN_DIR "vvis_optix_dll.dll"
    $dstExe = Join-Path $SDK_BIN "vvis_optix.exe"
    $dstDll = Join-Path $SDK_BIN "vvis_optix_dll.dll"
    Copy-Item $srcExe $dstExe -Force -ErrorAction Stop
    Copy-Item $srcDll $dstDll -Force -ErrorAction Stop
}

# Phase 0: compile the VMF once with VBSP (the cpu dir holds the result), then copy the BSP and PRT to the ref and gpu dirs.
# Skips the compile if the VMF and test manifest haven't changed ($State.SourceHash).
function Invoke-VvisPrepare {
    $VBSP_HASH_FILE = Join-Path $CPU_DIR "$MAP_NAME.vbsp_hash"
    $cachedVbspHash = if (Test-Path $VBSP_HASH_FILE) { "$(Get-Content $VBSP_HASH_FILE -Raw)".Trim() } else { "" }
    $needsVbspCompile = ($State.SourceHash -ne $cachedVbspHash) -or !(Test-Path "$CPU_DIR\$MAP_NAME.bsp") -or !(Test-Path "$CPU_DIR\$MAP_NAME.prt")

    Write-LogMessage "--- Phase 0: Compiling map (VBSP) ---"

    if ($needsVbspCompile) {
        # Copy VMF to cpu dir for compilation
        Copy-Item $P.SourceFile "$CPU_DIR\$MAP_NAME.vmf" -Force
        # Also copy .vmx if it exists (Hammer auto-save companion)
        $vmxSrc = Join-Path $MAPS_DIR "$MAP_NAME.vmx"
        if (Test-Path $vmxSrc) { Copy-Item $vmxSrc "$CPU_DIR\$MAP_NAME.vmx" -Force }

        if (!(Test-Path $VBSP_EXE)) {
            Write-LogMessage "CRITICAL ERROR: vbsp.exe not found at $VBSP_EXE"
            throw "FAIL"
        }

        $vbspLogFile = Join-Path $CPU_DIR "vbsp_$MAP_NAME.log"
        Write-LogMessage "VBSP Log: $vbspLogFile"

        # Source engine tools need the SDK's bin\x64 as CWD for DLL loading.
        # Use Start-Process to fully isolate from PowerShell's pipe environment.
        $vbspProc = Start-Process -FilePath $VBSP_EXE `
            -ArgumentList "$($EXTRA_VBSP_ARGS -join ' ') -game `"$MOD_DIR`" `"$CPU_DIR\$MAP_NAME`"" `
            -WorkingDirectory $SDK_BIN `
            -PassThru -Wait -NoNewWindow
        $vbspExit = $vbspProc.ExitCode
        if ($vbspExit -ne 0) {
            Write-LogMessage "CRITICAL ERROR: vbsp.exe failed with exit code $vbspExit. See log: $vbspLogFile"
            throw "FAIL"
        }

        # Save hash after successful compilation
        $State.SourceHash | Out-File -FilePath $VBSP_HASH_FILE -Encoding utf8 -NoNewline
    }
    else {
        Write-LogMessage "VBSP unchanged (VMF and manifest identical), using cached result."
    }

    # Distribute the compiled (or cached) BSP and PRT (portal file) to the ref and gpu directories.
    # VVIS requires the .prt file that VBSP generates alongside the BSP.
    foreach ($destDir in @($REF_DIR, $GPU_DIR)) {
        Copy-Item "$CPU_DIR\$MAP_NAME.bsp" "$destDir\$MAP_NAME.bsp" -Force
        $prtFile = "$CPU_DIR\$MAP_NAME.prt"
        if (Test-Path $prtFile) {
            Copy-Item $prtFile "$destDir\$MAP_NAME.prt" -Force
        }
    }
}

# Returns the time it took, or $null when vvis.exe failed (the reference is then skipped).
function Invoke-VvisRefRun {
    Write-LogMessage "--- Phase 1: ref-cpu (SDK vvis.exe) ---"
    Remove-Item $REF_LOG -ErrorAction SilentlyContinue

    Write-LogMessage "vvis.exe (SDK Reference) Log: $REF_LOG"

    $start = Get-Date
    $refProc = Start-Process -FilePath "$SDK_BIN\vvis.exe" `
        -ArgumentList "-game `"$MOD_DIR`" `"$REF_DIR\$MAP_NAME`"" `
        -WorkingDirectory $SDK_BIN `
        -PassThru -Wait -NoNewWindow
    $refExit = $refProc.ExitCode
    if ($refExit -ne 0) {
        Write-LogMessage "WARNING: vvis.exe (ref-cpu) failed with exit code $refExit. Skipping reference comparison."
        return $null
    }
    return (Get-Date) - $start
}

# Returns the time it took; throws "FAIL" when vvis_optix.exe fails.
function Invoke-VvisCpuRun {
    Write-LogMessage "--- Phase 2: cpu (vvis_optix.exe -threads $CONTROL_THREADS) ---"
    Remove-Item $CPU_LOG -ErrorAction SilentlyContinue

    Write-LogMessage "vvis_optix.exe CPU Log: $CPU_LOG"

    $start = Get-Date
    # Use Start-Process to fully isolate vvis_optix.exe from PowerShell's pipe
    # environment. Source engine tools crash with heap corruption (0xC0000374)
    # when invoked through & in nested script contexts due to inherited handles.
    $cpuProc = Start-Process -FilePath "cmd.exe" `
        -ArgumentList "/c `"cd /d `"$SDK_BIN`" && `"$SDK_BIN\vvis_optix.exe`" -threads $CONTROL_THREADS -game `"$MOD_DIR`" `"$CPU_DIR\$MAP_NAME`"`"" `
        -PassThru -Wait -NoNewWindow
    $cpuExit = $cpuProc.ExitCode
    if ($cpuExit -ne 0) {
        Write-LogMessage "CRITICAL ERROR: vvis_optix.exe (cpu) failed with exit code $cpuExit."
        throw "FAIL"
    }
    return (Get-Date) - $start
}

# Visual comparison of two screenshots with tgadiff.exe; throws "FAIL" at or above the tolerance.
function Compare-VvisVisual {
    param([string]$Label, [string]$Tga1, [string]$Tga2, [string]$DiffTga, [double]$Tolerance)

    Write-LogMessage "Comparing screenshots using tgadiff ($Label)..."
    $tgadiffExe = Join-Path $BIN_DIR "tgadiff.exe"
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

function Invoke-VvisCompareRefCpu {
    Write-LogMessage "`n--- Phase 3: Comparing ref-cpu vs cpu (CPU Parity Check) ---"

    $fcOutput = fc.exe /b /LB1 "$REF_DIR\$MAP_NAME.bsp" "$CPU_DIR\$MAP_NAME.bsp" 2>&1
    if ($LASTEXITCODE -eq 0) {
        Write-LogMessage "RESULT: PASS (ref-cpu and cpu BSPs are binary identical)"
    }
    else {
        Write-LogMessage "Warning: ref-cpu and cpu BSPs differ at binary level. Checking visibility data..."
        $bspDiffOutput = python -I (Join-Path $BIN_DIR "scripts\bsp_diff_visibility.py") "$REF_DIR\$MAP_NAME.bsp" "$CPU_DIR\$MAP_NAME.bsp" 2>&1
        $bspDiffExit = $LASTEXITCODE  # 0 = identical (no tolerance given)
        $bspDiffOutput | Out-File -FilePath $LOG_FILE -Append -Encoding utf8
        $bspDiffOutput | Write-Host

        if ($bspDiffExit -eq 0) {
            Write-LogMessage "RESULT: PASS (BSP bytes differ, but all visibility data is identical)"
        }
        else {
            $tgadiffExe = Join-Path $BIN_DIR "tgadiff.exe"
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
                Take-Screenshot "$CPU_DIR\$MAP_NAME.bsp" "screenshot_cpu_$MAP_NAME.tga"
                Compare-VvisVisual "ref-cpu vs cpu" "screenshot_ref-cpu_$MAP_NAME.tga" "screenshot_cpu_$MAP_NAME.tga" "screenshot_diff_ref_cpu_$MAP_NAME.tga" $REF_CPU_TOL
            }
        }
    }
}

function Invoke-VvisGpuRunSpec {
    Write-LogMessage "--- Phase 4: gpu (vvis_optix.exe, all threads) ---"
    Remove-Item $GPU_LOG -ErrorAction SilentlyContinue

    Write-LogMessage "vvis_optix.exe (default flow) Log: $GPU_LOG"
    return @{
        FilePath         = Join-Path $SDK_BIN "vvis_optix.exe"
        ArgumentList     = "-game `"$MOD_DIR`" `"$GPU_DIR\$MAP_NAME`""
        WorkingDirectory = $SDK_BIN
    }
}

function Invoke-VvisGpuTimeout {
    # Sample current usage snapshot
    $currentGpu = nvidia-smi --query-gpu=utilization.gpu --format="csv,noheader,nounits" 2>$null
    $currentCpu = (Get-Counter '\Processor(_Total)\% Processor Time' -ErrorAction SilentlyContinue).CounterSamples.CookedValue

    Write-LogMessage "CRITICAL ERROR: vvis_optix (default flow) hung or is significantly slower than cpu!"
    if ($currentCpu) { Write-LogMessage "CPU Usage at time of termination: $($currentCpu.ToString('F1'))%" }
    if ($currentGpu) { Write-LogMessage "GPU Usage at time of termination: $($currentGpu.Trim())%" }
    Write-LogMessage "Notice: You can extend the runtime by adding: -TimeoutExtensionMinutes <minutes>"
}

function Invoke-VvisCompareCpuGpu {
    Write-LogMessage "`n--- Phase 5: Comparing cpu vs gpu (GPU Parity Check) ---"

    $fcOutput = fc.exe /b /LB1 "$CPU_DIR\$MAP_NAME.bsp" "$GPU_DIR\$MAP_NAME.bsp" 2>&1
    if ($LASTEXITCODE -eq 0) {
        Write-LogMessage "RESULT: PASS (cpu and gpu BSPs are binary identical)"
    }
    else {
        Write-LogMessage "Warning: cpu and gpu BSPs differ at binary level. Checking visibility data..."
        $bspDiffOutput = python -I (Join-Path $BIN_DIR "scripts\bsp_diff_visibility.py") "$CPU_DIR\$MAP_NAME.bsp" "$GPU_DIR\$MAP_NAME.bsp" $CPU_GPU_TOL 2>&1
        $bspDiffExit = $LASTEXITCODE  # 0 = identical or within CpuGpuTolerance
        $bspDiffOutput | Out-File -FilePath $LOG_FILE -Append -Encoding utf8
        $bspDiffOutput | Write-Host

        if ($bspDiffExit -eq 0) {
            Write-LogMessage "RESULT: PASS (BSP bytes differ, but visibility data is identical or within acceptable tolerance)"
        }
        else {
            $tgadiffExe = Join-Path $BIN_DIR "tgadiff.exe"
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
                    Take-Screenshot "$CPU_DIR\$MAP_NAME.bsp" "screenshot_cpu_$MAP_NAME.tga"
                }
                Take-Screenshot "$GPU_DIR\$MAP_NAME.bsp" "screenshot_gpu_$MAP_NAME.tga"
                Compare-VvisVisual "cpu vs gpu" "screenshot_cpu_$MAP_NAME.tga" "screenshot_gpu_$MAP_NAME.tga" "screenshot_diff_cpu_gpu_$MAP_NAME.tga" $CPU_GPU_TOL
            }
        }
    }
}

# ============================================================
#  Main test flow (the same phases for both tools)
# ============================================================
if ($Tool -eq 'vvis') { Install-VvisBinaries }

$testFailed = $false
try {

    Write-LogMessage "=== Test: $ARCHIVE_SUFFIX ($MAP_NAME) ==="

    # Ensure directories exist
    foreach ($dir in @($REF_DIR, $CPU_DIR, $GPU_DIR)) {
        if (!(Test-Path $dir)) { New-Item -ItemType Directory -Path $dir | Out-Null }
    }

    if (!$P.SourceFile) { $P.SourceFile = Join-Path $TESTS_DIR "maps\$MAP_NAME.bsp" }
    if (!(Test-Path $P.SourceFile)) {
        Write-LogMessage "CRITICAL ERROR: Source $($P.SourceKind) $($P.SourceFile) not found!"
        throw "FAIL"
    }

    # The reference and the compile caches are keyed on the source file and the test manifest (per-test
    # parameters like ExtraArgs and tolerances live there, not in this shared harness script).
    $MANIFEST_PATH = Join-Path $PSScriptRoot "${Tool}_tests.psd1"
    $currentHash = (Get-FileHash $P.SourceFile -Algorithm SHA256).Hash + ":" + (Get-FileHash $MANIFEST_PATH -Algorithm SHA256).Hash
    $State.SourceHash = $currentHash   # read by the tool steps (Invoke-VvisPrepare)

    # --- Prepare: put the source map into the ref/cpu/gpu dirs ---
    Invoke-ToolStep 'Prepare'

    # --- Phase 1: ref-cpu (the SDK's reference binary) ---
    # Skip recompilation if the source and test manifest haven't changed.
    $State.HasReference = Test-Path $P.RefExe
    $REF_HASH_FILE = "$REF_DIR\$MAP_NAME.ref_hash"
    $REF_TIME_FILE = "$REF_DIR\$MAP_NAME.ref_time"
    $cachedHash = if (Test-Path $REF_HASH_FILE) { "$(Get-Content $REF_HASH_FILE -Raw)".Trim() } else { "" }
    $needsRefCompile = ($currentHash -ne $cachedHash) -or !(Test-Path "$REF_DIR\$MAP_NAME.bsp")
    $State.NeedsRefCompile = $needsRefCompile   # read by Invoke-VradCompareRefCpu

    if ($State.HasReference) {
        if ($needsRefCompile) {
            $refElapsed = Invoke-ToolStep 'RefRun'
            if ($null -eq $refElapsed) {
                $State.HasReference = $false
            }
            else {
                $State.RefTime = $refElapsed
                $currentHash | Out-File -FilePath $REF_HASH_FILE -Encoding utf8 -NoNewline
                $State.RefTime.TotalSeconds.ToString("F2") | Out-File -FilePath $REF_TIME_FILE -Encoding utf8 -NoNewline
            }
        }
        else {
            foreach ($line in $P.RefCachedMessages) { Write-LogMessage $line }
            if (Test-Path $REF_TIME_FILE) {
                $State.RefTime = [TimeSpan]::FromSeconds([double]"$(Get-Content $REF_TIME_FILE -Raw)".Trim())
                $State.RefTimeCached = $true
            }
        }
    }
    else {
        Write-LogMessage $P.RefMissingMessage
    }

    # --- Phase 2: cpu (our binary, CPU only) ---
    $State.CpuTime = Invoke-ToolStep 'CpuRun'

    # --- Phase 3: ref-cpu vs cpu Comparison (CPU Parity Check) ---
    if ($State.HasReference) {
        Invoke-ToolStep 'CompareRefCpu'
    }

    # --- Phase 4: gpu ---
    Invoke-GpuRun (Invoke-ToolStep 'GpuRunSpec')

    # --- Timing Summary ---
    Write-LogMessage "`n--- Timing Summary ---"
    if ($State.HasReference) {
        $cachedLabel = if ($State.RefTimeCached) { " (cached)" } else { "" }
        Write-LogMessage "$($P.TimingRef)$($State.RefTime.TotalSeconds.ToString("F2"))s$cachedLabel"
    }
    Write-LogMessage "$($P.TimingCpu)$($State.CpuTime.TotalSeconds.ToString("F2"))s"
    if ($State.TimedOut) {
        Write-LogMessage "$($P.TimingGpu)Did not finish"
    }
    else {
        Write-LogMessage "$($P.TimingGpu)$($State.GpuTime.TotalSeconds.ToString("F2"))s"
    }

    # --- Phase 5: cpu vs gpu (GPU Parity Check) ---
    if (-not $State.TimedOut) {
        Invoke-ToolStep 'CompareCpuGpu'
    }

}
catch {
    $testFailed = $true
}

# ============================================================
#  Archive logs and screenshots
# ============================================================
$ARCHIVE_DIR = Join-Path $TESTS_DIR "out\logs"
if (!(Test-Path $ARCHIVE_DIR)) { New-Item -ItemType Directory -Path $ARCHIVE_DIR | Out-Null }

# Convert any generated TGA files to PNG for easier viewing
Write-LogMessage "Converting TGA screenshots to PNG..."
$tgaArgs = @($P.TgaFiles | Where-Object { Test-Path $_ })
if ($tgaArgs.Count -gt 0) {
    python -I $IMAGE_COMPARE tga2png @tgaArgs | Write-Host
}

$timestamp = (Get-Date).ToString("yyyy-MM-ddTHH-mm-ss")
$RUN_DIR = "$ARCHIVE_DIR\${timestamp}_$ARCHIVE_SUFFIX"
New-Item -ItemType Directory -Path $RUN_DIR | Out-Null

$logMap = @{
    $LOG_FILE = "test_$ARCHIVE_SUFFIX.log"
    $REF_LOG  = "${Tool}_ref-cpu.log"
    $CPU_LOG  = "${Tool}_cpu.log"
    $GPU_LOG  = "${Tool}_gpu.log"
}
foreach ($entry in $logMap.GetEnumerator()) {
    if (Test-Path $entry.Key) {
        if ($P.ArchiveMoveLogs) {
            Move-Item $entry.Key "$RUN_DIR\$($entry.Value)" -Force
        }
        else {
            Copy-Item $entry.Key "$RUN_DIR\$($entry.Value)" -Force
        }
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

Pop-Location
if ($testFailed -or $State.TimedOut) { exit 1 }
