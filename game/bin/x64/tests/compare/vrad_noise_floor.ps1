# VRAD Lightmap Noise Floor Measurement
# Compiles the maps of vrad_tests.psd1 several times each (the SDK reference vrad.exe once,
# our vrad_rtx.exe on the CPU and with -cuda, -Runs times each) and reports how far apart
# image_compare.py lightmaps puts the results: the run-to-run noise of one build (the floor
# any LightmapThreshold has to clear) and the distance between the builds. The thresholds in
# vrad_tests.psd1 cover both, so both are measured. This script only measures: it never edits
# vrad_tests.psd1 and never fails because a figure is large.
#
# Usage:
#   .\vrad_noise_floor.ps1 -All                         # every test in vrad_tests.psd1
#   .\vrad_noise_floor.ps1 -Group core                  # every test of one group
#   .\vrad_noise_floor.ps1 -TestNames quick,full        # named tests ("a,b" is split)
#   .\vrad_noise_floor.ps1 -TestNames quick -Runs 3     # three cpu and three gpu runs each
#   .\vrad_noise_floor.ps1 -All -SkipRef                # without the SDK reference runs
#   .\vrad_noise_floor.ps1 -All -Threads 4              # -threads 4 on the cpu legs only
#   (also -OutDir <dir> and -BinDir <dir>; defaults below)
#
# Per test it makes <OutDir>\<test>\ref, cpu1..cpuN, gpu1..gpuN (default OutDir:
# <BinDir>\tests\out\vrad_noise_floor, the git-ignored output folder), copies the source BSP
# into each and lights it in place: vrad.exe for ref (the SDK install's own build, run from the
# SDK's bin folder), vrad_rtx.exe for cpu, vrad_rtx.exe -cuda for gpu, always
# -game <SDK sourcetest>, one run after another and interleaved (ref, cpu1, gpu1, cpu2, gpu2,
# ...) so a drift in the machine's load does not fall on one kind only. Each run's console
# output goes to <folder>\<MapName>.console.log. A non-zero exit code of vrad_rtx.exe stops
# the measuring (the last lines of that log are printed, the tests measured before it still go
# into the table and the CSV, exit code 1); a missing or failing vrad.exe only skips that
# test's "ref vs CPU" column.
#
# Columns of the final table (the same rows go to <OutDir>\noise_floor.csv, overwritten each
# run). Every percentage is image_compare.py lightmaps' share of the decoded light that
# differs, four decimals:
#   Test, Map      the test name in vrad_tests.psd1 and its map
#   CPU noise      the largest figure between two cpu runs of the same build: the CPU noise floor
#   GPU noise      the same between two gpu runs
#   ref vs CPU     the smallest..largest figure between the reference and each cpu run
#   CPU vs GPU     the smallest..largest figure between every cpu and every gpu run
#   Thr            the test's current LightmapThreshold in vrad_tests.psd1, for comparison
#   CPU s, GPU s   the mean wall time of one cpu / gpu compile
# A cell is marked "structural" when one of its comparisons found the two BSPs do not have
# the same lit faces (image_compare.py's "Structural difference:" line: not a matter of
# tolerance). "-" means there was no reference run. Exit code 0 when every selected test was
# measured, 1 otherwise.

param(
    [Parameter(Position = 0)][string[]]$TestNames,
    [string]$Group,
    [switch]$All,
    [int]$Runs = 2,
    [string]$OutDir,
    [string]$BinDir,
    [switch]$SkipRef = $false,
    [int]$Threads = 0
)

# --- Load manifest ---
$manifestPath = Join-Path $PSScriptRoot "vrad_tests.psd1"
if (!(Test-Path $manifestPath)) {
    Write-Host "CRITICAL ERROR: Test manifest not found at $manifestPath"
    exit 1
}
$allTests = Import-PowerShellDataFile $manifestPath

# --- Locations (this file is game\bin\x64\tests\compare) ---
$TESTS_DIR = Split-Path -Parent $PSScriptRoot
if (!$BinDir) { $BinDir = Split-Path -Parent $TESTS_DIR }    # game\bin\x64: the tools load their PTX from the working directory
if (!$OutDir) { $OutDir = Join-Path (Join-Path $TESTS_DIR "out") "vrad_noise_floor" }   # git-ignored output folder
# Absolute from here on: a relative -BinDir or -OutDir would mean one folder before the
# Push-Location below and another after it.
$BinDir = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($BinDir)
$OutDir = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($OutDir)
$MAPS_DIR = Join-Path $PSScriptRoot "maps"
$IMAGE_COMPARE = Join-Path $PSScriptRoot "image_compare.py"

# --- Constants (compare_harness.ps1) ---
$SDK_ROOT = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer"
$MOD_DIR = "$SDK_ROOT\sourcetest"
$SDK_BIN = "$SDK_ROOT\bin\x64"
$REF_EXE = "$SDK_BIN\vrad.exe"    # the SDK install's own build, run from its own folder (compare_harness.ps1 does the same)
$VRAD_EXE = Join-Path $BinDir "vrad_rtx.exe"

if ($Runs -lt 2) {
    Write-Host "ERROR: -Runs must be at least 2 (one run of a leg gives no noise figure)."
    exit 1
}
if (!(Test-Path $VRAD_EXE)) {
    Write-Host "CRITICAL ERROR: vrad_rtx.exe not found at $VRAD_EXE"
    exit 1
}
if (-not (Get-Command python -ErrorAction SilentlyContinue)) {
    Write-Host "CRITICAL ERROR: python not found on PATH (image_compare.py is run with it)."
    exit 1
}

# --- Collect available groups ---
$availableGroups = @{}
foreach ($entry in $allTests.GetEnumerator()) {
    $groups = $entry.Value.Groups
    if ($groups) {
        foreach ($g in $groups) {
            if (-not $availableGroups.ContainsKey($g)) { $availableGroups[$g] = @() }
            $availableGroups[$g] += $entry.Key
        }
    }
}

# --- Resolve which tests to measure (the same way run_compare_tests.ps1 selects them) ---
if ($All) {
    $selectedNames = $allTests.Keys | Sort-Object
}
elseif ($Group) {
    if (-not $availableGroups.ContainsKey($Group)) {
        Write-Host "ERROR: Unknown group '$Group'. Available groups: $($availableGroups.Keys | Sort-Object | ForEach-Object { "$_ ($($availableGroups[$_] -join ', '))" })"
        exit 1
    }
    $selectedNames = $availableGroups[$Group] | Sort-Object
}
elseif ($TestNames -and $TestNames.Count -gt 0) {
    # pwsh -File passes "a,b" as one string; split it so the launchers can name several tests.
    $selectedNames = @($TestNames | ForEach-Object { $_ -split "," } | ForEach-Object { $_.Trim() } | Where-Object { $_ })
}
else {
    Write-Host "Usage: vrad_noise_floor.ps1 -All | -TestNames <name1>,<name2>,... | -Group <group> [-Runs N] [-OutDir <dir>] [-BinDir <dir>] [-SkipRef] [-Threads N]"
    Write-Host "Available tests: $($allTests.Keys | Sort-Object | ForEach-Object { $_ }) "
    Write-Host "Available groups: $($availableGroups.Keys | Sort-Object | ForEach-Object { "$_ ($($availableGroups[$_] -join ', '))" })"
    exit 1
}

# Validate all names exist
foreach ($name in $selectedNames) {
    if (-not $allTests.ContainsKey($name)) {
        Write-Host "ERROR: Unknown test '$name'. Available: $($allTests.Keys -join ', ')"
        exit 1
    }
}

# ============================================================
#  Helpers
# ============================================================

# The leg a BSP belongs to ("ref", "cpu1", "gpu2") from its folder name.
function Get-LegName {
    param([Parameter(Mandatory)][string]$BspPath)
    return Split-Path -Leaf (Split-Path -Parent $BspPath)
}

# Lights one copy of the map and returns the exit code and the wall time. The tool's whole
# console output goes to $LogFile. The caller has already Push-Location'd the bin folder.
function Invoke-VradLeg {
    param(
        [Parameter(Mandatory)][string]$Label,
        [Parameter(Mandatory)][string]$Exe,
        [Parameter(Mandatory)][string[]]$ArgList,
        [Parameter(Mandatory)][string]$LogFile,
        [string]$WorkDir    # run the tool from here instead of the bin folder (the SDK reference)
    )
    Write-Host "  [$Label] $(Split-Path -Leaf $Exe) $($ArgList -join ' ')"
    $start = Get-Date
    if ($WorkDir) { Push-Location $WorkDir }
    try {
        & $Exe @ArgList 2>&1 | Out-File -FilePath $LogFile -Encoding utf8
        $exitCode = $LASTEXITCODE
    }
    finally { if ($WorkDir) { Pop-Location } }
    $seconds = ((Get-Date) - $start).TotalSeconds
    Write-Host "  [$Label] exit code $exitCode, $($seconds.ToString('F1'))s, log: $LogFile"
    return @{ ExitCode = $exitCode; Seconds = $seconds }
}

# One image_compare.py lightmaps comparison. --threshold 100 so nothing can be "over" it:
# this script measures, it does not judge. Percent is $null when the output carried no
# "Lightmap Difference:" line (unreadable BSP); Structural mirrors the "Structural
# difference:" line (the two files do not have the same lit faces).
function Compare-Lightmaps {
    param(
        [Parameter(Mandatory)][string]$BspA,
        [Parameter(Mandatory)][string]$BspB
    )
    $output = python -I $IMAGE_COMPARE lightmaps $BspA $BspB --threshold 100 2>&1
    $diffMatch = $output | Select-String "Lightmap Difference:\s*([\d.]+)%"
    if ($diffMatch) {
        $percent = [double]$diffMatch.Matches.Groups[1].Value
    }
    else {
        $percent = $null
        Write-Host "  WARNING: no 'Lightmap Difference:' line between $(Get-LegName $BspA) and $(Get-LegName $BspB):"
        $output | Write-Host
    }
    $structural = $null -ne ($output | Select-String "Structural difference:")
    if ($structural) {
        Write-Host "  NOTE: structural difference between $(Get-LegName $BspA) and $(Get-LegName $BspB) (not the same lit faces)." -ForegroundColor Yellow
    }
    return @{ Percent = $percent; Structural = $structural }
}

# Compares a list of pairs (@{ A = bsp; B = bsp }) and keeps the smallest and largest figure.
# Measured is $false as soon as one comparison produced no figure.
function Measure-Pairs {
    param([Parameter(Mandatory)][object[]]$Pairs)
    $percents = @()
    $missing = $false
    $structural = $false
    foreach ($pair in $Pairs) {
        $result = Compare-Lightmaps $pair.A $pair.B
        if ($null -ne $result.Percent) { $percents += $result.Percent } else { $missing = $true }
        if ($result.Structural) { $structural = $true }
    }
    $stats = @{ Structural = $structural; Measured = (($percents.Count -gt 0) -and (-not $missing)) }
    if ($percents.Count -gt 0) {
        $stats.Min = ($percents | Measure-Object -Minimum).Minimum
        $stats.Max = ($percents | Measure-Object -Maximum).Maximum
    }
    return $stats
}

# One table cell: the largest figure for a noise cell, the smallest..largest otherwise
# (one number when the two ends fall together).
function Format-Cell {
    param(
        [Parameter(Mandatory)][hashtable]$Stats,
        [switch]$Range
    )
    if (!$Stats.Measured) { return "no figure" }
    # Invariant culture: a decimal comma would split the CSV's columns.
    $inv = [cultureinfo]::InvariantCulture
    $figures = if ($Range -and $Stats.Min -ne $Stats.Max) { $Stats.Min.ToString("F4", $inv) + ".." + $Stats.Max.ToString("F4", $inv) } else { $Stats.Max.ToString("F4", $inv) }
    if ($Stats.Structural) { return "structural: $figures" }
    return $figures
}

# ============================================================
#  Main flow
# ============================================================

New-Item -ItemType Directory -Path $OutDir -Force | Out-Null

if ($SkipRef) {
    Write-Host "-SkipRef given: no SDK reference runs, the 'ref vs CPU' column will be '-'." -ForegroundColor Yellow
}
elseif (!(Test-Path $REF_EXE)) {
    Write-Host "vrad.exe not found at ${REF_EXE}: no SDK reference runs, the 'ref vs CPU' column will be '-'." -ForegroundColor Yellow
}

$rows = @()
$allMeasured = $true

# The tools load their PTX from the working directory, so run them from the bin folder; every
# path handed to a tool is absolute. Pop-Location on every exit path (a throw or exit in the
# try block included).
Push-Location $BinDir
try {

    foreach ($name in $selectedNames) {
        $config = $allTests[$name]
        $mapName = $config.MapName
        $extraArgs = @($config.ExtraArgs)
        Write-Host ""
        Write-Host ">>> Test: $name ($mapName) <<<" -ForegroundColor Cyan

        # The BSPs are git-ignored fixtures in tests\maps (the VMFs here are tracked); take
        # one from tests\compare\maps first if someone put it there.
        $sourceFile = @((Join-Path $MAPS_DIR "$mapName.bsp"), (Join-Path (Join-Path $TESTS_DIR "maps") "$mapName.bsp")) |
            Where-Object { Test-Path $_ } | Select-Object -First 1
        if (!$sourceFile) {
            Write-Host "CRITICAL ERROR: Source BSP $mapName.bsp not found in $MAPS_DIR or $(Join-Path $TESTS_DIR 'maps')." -ForegroundColor Red
            $allMeasured = $false
            $rows += [ordered]@{ Test = $name; Map = $mapName; CpuNoise = "no source"; GpuNoise = "no source"; RefVsCpu = "-"; CpuVsGpu = "no source"; Threshold = $config.LightmapThreshold; CpuSeconds = "-"; GpuSeconds = "-" }
            continue
        }

        # One folder per run, each with its own copy of the source BSP (the tool lights it in place).
        $testDir = Join-Path $OutDir $name
        $legs = @("ref") + @(1..$Runs | ForEach-Object { "cpu$_" }) + @(1..$Runs | ForEach-Object { "gpu$_" })
        $legPaths = @{}
        foreach ($leg in $legs) {
            $legDir = Join-Path $testDir $leg
            New-Item -ItemType Directory -Path $legDir -Force | Out-Null
            Copy-Item $sourceFile (Join-Path $legDir "$mapName.bsp") -Force
            $legPaths[$leg] = @{
                Bsp      = Join-Path $legDir "$mapName.bsp"
                MapNoExt = Join-Path $legDir $mapName
                Log      = Join-Path $legDir "$mapName.console.log"
            }
        }

        $seconds = @{ cpu = @(); gpu = @() }
        $hasRef = (-not $SkipRef) -and (Test-Path $REF_EXE)

        # ref: the SDK reference. A failure only skips this test's reference columns.
        if ($hasRef) {
            $refArgs = $extraArgs + @("-game", $MOD_DIR, $legPaths['ref'].MapNoExt)
            $result = Invoke-VradLeg -Label 'ref' -Exe $REF_EXE -ArgList $refArgs -LogFile $legPaths['ref'].Log -WorkDir $SDK_BIN
            if ($result.ExitCode -ne 0) {
                Write-Host "  WARNING: vrad.exe (reference) failed with exit code $($result.ExitCode), skipping the 'ref vs CPU' column of this test." -ForegroundColor Yellow
                $hasRef = $false
            }
        }

        for ($k = 1; $k -le $Runs; $k++) {
            # Interleave the cpu and gpu runs so a drift in the machine's load does not fall on one kind only.
            foreach ($kind in @("cpu", "gpu")) {
                $leg = "${kind}$k"
                $toolArgs = $extraArgs + @()
                if ($kind -eq "cpu" -and $Threads -gt 0) { $toolArgs += @("-threads", "$Threads") }
                $toolArgs += @("-game", $MOD_DIR)
                if ($kind -eq "gpu") { $toolArgs += "-cuda" }
                $toolArgs += $legPaths[$leg].MapNoExt
                $result = Invoke-VradLeg -Label $leg -Exe $VRAD_EXE -ArgList $toolArgs -LogFile $legPaths[$leg].Log
                if ($result.ExitCode -ne 0) {
                    Write-Host "CRITICAL ERROR: vrad_rtx.exe ($leg) failed with exit code $($result.ExitCode). Last lines of $($legPaths[$leg].Log):" -ForegroundColor Red
                    if (Test-Path $legPaths[$leg].Log) { Get-Content $legPaths[$leg].Log -Tail 25 | ForEach-Object { Write-Host "  $_" } }
                    # To the catch below: the tests already measured are still printed and written
                    # (an exit here left the previous run's CSV on disk), and the exit code is 1.
                    throw "vrad_rtx.exe ($leg) failed on test '$name'; the tests from there on were not measured."
                }
                $seconds[$kind] += $result.Seconds
            }
        }

        # --- Comparisons ---
        $cpuPairs = @()
        $gpuPairs = @()
        $refPairs = @()
        $crossPairs = @()
        for ($i = 1; $i -le $Runs; $i++) {
            for ($j = $i + 1; $j -le $Runs; $j++) {
                $cpuPairs += @{ A = $legPaths["cpu$i"].Bsp; B = $legPaths["cpu$j"].Bsp }
                $gpuPairs += @{ A = $legPaths["gpu$i"].Bsp; B = $legPaths["gpu$j"].Bsp }
            }
            if ($hasRef) {
                $refPairs += @{ A = $legPaths['ref'].Bsp; B = $legPaths["cpu$i"].Bsp }
            }
            for ($j = 1; $j -le $Runs; $j++) {
                $crossPairs += @{ A = $legPaths["cpu$i"].Bsp; B = $legPaths["gpu$j"].Bsp }
            }
        }

        $cpuNoise = Measure-Pairs $cpuPairs
        $gpuNoise = Measure-Pairs $gpuPairs
        $refStats = if ($hasRef) { Measure-Pairs $refPairs } else { $null }
        $crossStats = Measure-Pairs $crossPairs

        $rows += [ordered]@{
            Test       = $name
            Map        = $mapName
            CpuNoise   = Format-Cell $cpuNoise
            GpuNoise   = Format-Cell $gpuNoise
            RefVsCpu   = if ($null -ne $refStats) { Format-Cell $refStats -Range } else { "-" }
            CpuVsGpu   = Format-Cell $crossStats -Range
            Threshold  = $config.LightmapThreshold
            CpuSeconds = if ($seconds['cpu'].Count -gt 0) { ($seconds['cpu'] | Measure-Object -Average).Average.ToString("F1", [cultureinfo]::InvariantCulture) } else { "-" }
            GpuSeconds = if ($seconds['gpu'].Count -gt 0) { ($seconds['gpu'] | Measure-Object -Average).Average.ToString("F1", [cultureinfo]::InvariantCulture) } else { "-" }
        }
        if (-not ($cpuNoise.Measured -and $gpuNoise.Measured -and $crossStats.Measured -and (($null -eq $refStats) -or $refStats.Measured))) {
            $allMeasured = $false
        }
    }

}
catch {
    # An error nobody planned for (a tool that cannot be started, for one): the tests from
    # here on were not measured, and the exit code has to say so.
    $allMeasured = $false
    Write-Host "ERROR: $($_.Exception.Message)" -ForegroundColor Red
}
finally { Pop-Location }

# --- Result table ---
$columnFormat = "{0,-22} {1,-37} {2,10} {3,10} {4,25} {5,25} {6,6} {7,8} {8,8}"
Write-Host ""
Write-Host "======================================"
Write-Host "  VRAD Lightmap Noise Floor ($($selectedNames -join ', '))"
Write-Host "======================================"
Write-Host ""
Write-Host ($columnFormat -f "Test", "Map", "CPU noise", "GPU noise", "ref vs CPU", "CPU vs GPU", "Thr", "CPU s", "GPU s")
Write-Host ($columnFormat -f ("-" * 22), ("-" * 37), ("-" * 10), ("-" * 10), ("-" * 25), ("-" * 25), ("-" * 6), ("-" * 8), ("-" * 8))
foreach ($row in $rows) {
    Write-Host ($columnFormat -f $row['Test'], $row['Map'], $row['CpuNoise'], $row['GpuNoise'], $row['RefVsCpu'], $row['CpuVsGpu'], $row['Threshold'], $row['CpuSeconds'], $row['GpuSeconds'])
}

$csvPath = Join-Path $OutDir "noise_floor.csv"
$csvLines = @("Test,Map,CpuNoise,GpuNoise,RefVsCpu,CpuVsGpu,Threshold,CpuSeconds,GpuSeconds")
foreach ($row in $rows) {
    $csvLines += "$($row['Test']),$($row['Map']),$($row['CpuNoise']),$($row['GpuNoise']),$($row['RefVsCpu']),$($row['CpuVsGpu']),$($row['Threshold']),$($row['CpuSeconds']),$($row['GpuSeconds'])"
}
$csvLines | Out-File -FilePath $csvPath -Encoding utf8
Write-Host ""
Write-Host "Rows also written to $csvPath"

if (!$allMeasured) {
    Write-Host ""
    Write-Host "Not every selected test was measured." -ForegroundColor Red
    exit 1
}
