# vrad-RTX Test Runner
# Orchestrates sequential execution of validation tests defined in vrad_tests.psd1.
#
# Usage:
#   .\run_vrad_tests.ps1 -All                             # Run all tests
#   .\run_vrad_tests.ps1 -TestNames basic,harder           # Run specific tests
#   .\run_vrad_tests.ps1 -Group core                       # Run all tests in a group
#   .\run_vrad_tests.ps1 -TestNames basic -SkipVisualCheck # Skip screenshot comparisons
#   .\run_vrad_tests.ps1 -All -TimeoutExtensionMinutes 10  # Extend GPU timeout

param(
    [string[]]$TestNames,
    [string]$Group,
    [switch]$All,
    [int]$TimeoutExtensionMinutes = 0,
    [switch]$SkipVisualCheck = $false
)

# --- Version gate ---
if ($PSVersionTable.PSVersion.Major -lt 7 -or ($PSVersionTable.PSVersion.Major -eq 7 -and $PSVersionTable.PSVersion.Minor -lt 6)) {
    Throw "This script requires PowerShell 7.6 or later. Your version: $($PSVersionTable.PSVersion)"
}

# --- Load manifest ---
$manifestPath = Join-Path $PSScriptRoot "vrad_tests.psd1"
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
    $selectedNames = $TestNames
}
else {
    Write-Host "Usage: run_vrad_tests.ps1 -All | -TestNames <name1>,<name2>,... | -Group <group>"
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
$harnessPath = Join-Path $PSScriptRoot "vrad_test_harness.ps1"
$results = @{}
$totalStart = Get-Date

Write-Host ""
Write-Host "======================================"
Write-Host "  vrad-RTX Test Suite"
Write-Host "  Tests: $($selectedNames -join ', ')"
Write-Host "======================================"
Write-Host ""

foreach ($name in $selectedNames) {
    $config = $allTests[$name]
    Write-Host ">>> Starting test: $name ($($config.MapName)) <<<" -ForegroundColor Cyan
    Write-Host ""

    $testStart = Get-Date

    $passThrough = @{
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
