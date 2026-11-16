# Comparison Test Runner (vrad-RTX and VVIS-OPTIX)
# Orchestrates sequential execution of the validation tests defined in vrad_tests.psd1
# (-Tool vrad) or vvis_tests.psd1 (-Tool vvis), one compare_harness.ps1 run per test.
#
# Usage:
#   .\run_compare_tests.ps1 -Tool vrad -All                             # Run all tests
#   .\run_compare_tests.ps1 -Tool vvis -TestNames basic,harder           # Run specific tests
#   .\run_compare_tests.ps1 -Tool vrad -Group core                       # Run all tests in a group
#   .\run_compare_tests.ps1 -Tool vvis -TestNames basic -SkipVisualCheck # Skip screenshot comparisons
#   .\run_compare_tests.ps1 -Tool vrad -All -TimeoutExtensionMinutes 10  # Extend GPU timeout

param(
    [ValidateSet('vrad', 'vvis')][string]$Tool,
    [Parameter(Position = 0)][string[]]$TestNames,
    [string]$Group,
    [switch]$All,
    [int]$TimeoutExtensionMinutes = 0,
    [switch]$SkipVisualCheck = $false
)

# --- Version gate ---
if ($PSVersionTable.PSVersion.Major -lt 7 -or ($PSVersionTable.PSVersion.Major -eq 7 -and $PSVersionTable.PSVersion.Minor -lt 6)) {
    Throw "This script requires PowerShell 7.6 or later. Your version: $($PSVersionTable.PSVersion)"
}

if (-not $Tool) {
    Write-Host "Usage: run_compare_tests.ps1 -Tool vrad|vvis -All | -TestNames <name1>,<name2>,... | -Group <group>"
    exit 1
}

# --- Load manifest ---
$manifestPath = Join-Path $PSScriptRoot "${Tool}_tests.psd1"
if (!(Test-Path $manifestPath)) {
    Write-Host "CRITICAL ERROR: Test manifest not found at $manifestPath"
    exit 1
}
$allTests = Import-PowerShellDataFile $manifestPath

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

# --- Resolve which tests to run ---
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
    Write-Host "Usage: run_compare_tests.ps1 -Tool $Tool -All | -TestNames <name1>,<name2>,... | -Group <group>"
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

# --- Run tests sequentially ---
$harnessPath = Join-Path $PSScriptRoot "compare_harness.ps1"
$suiteTitle = if ($Tool -eq 'vrad') { "vrad-RTX Test Suite" } else { "VVIS-OPTIX Test Suite" }
$results = @{}
$totalStart = Get-Date

Write-Host ""
Write-Host "======================================"
Write-Host "  $suiteTitle"
Write-Host "  Tests: $($selectedNames -join ', ')"
Write-Host "======================================"
Write-Host ""

foreach ($name in $selectedNames) {
    $config = $allTests[$name]
    Write-Host ">>> Starting test: $name ($($config.MapName)) <<<" -ForegroundColor Cyan
    Write-Host ""

    $testStart = Get-Date

    $passThrough = @{
        Tool                    = $Tool
        TestConfig              = $config
        TimeoutExtensionMinutes = $TimeoutExtensionMinutes
        SkipVisualCheck         = $SkipVisualCheck
    }

    & $harnessPath @passThrough
    $testExitCode = $LASTEXITCODE

    $testDuration = (Get-Date) - $testStart

    if ($testExitCode -eq 0) {
        $results[$name] = @{ Status = "PASS"; Duration = $testDuration }
        Write-Host ""
        Write-Host ">>> $name : PASS ($($testDuration.TotalSeconds.ToString('F1'))s) <<<" -ForegroundColor Green
    }
    else {
        $results[$name] = @{ Status = "FAIL"; Duration = $testDuration }
        Write-Host ""
        Write-Host ">>> $name : FAIL ($($testDuration.TotalSeconds.ToString('F1'))s) <<<" -ForegroundColor Red
    }
    Write-Host ""
}

# --- Aggregate summary ---
$totalDuration = (Get-Date) - $totalStart
$passed = @($results.Values | Where-Object { $_.Status -eq "PASS" }).Count
$failed = @($results.Values | Where-Object { $_.Status -eq "FAIL" }).Count
$total = $results.Count

Write-Host "======================================"
Write-Host "  Summary: $passed/$total PASSED, $failed/$total FAILED"
Write-Host "  Total time: $($totalDuration.TotalSeconds.ToString('F1'))s"
Write-Host "======================================"
Write-Host ""

foreach ($name in $selectedNames) {
    $r = $results[$name]
    $color = if ($r.Status -eq "PASS") { "Green" } else { "Red" }
    Write-Host "  [$($r.Status)]  $name  ($($r.Duration.TotalSeconds.ToString('F1'))s)" -ForegroundColor $color
}

Write-Host ""

if ($failed -gt 0) { exit 1 }
