# Times VRAD's static prop lighting stage across per-texel lightmap resolutions.
#
# For each resolution, sprp_lightmap.py writes a copy of the map with the chosen
# props set to that resolution (and per-texel off on the rest), then CPU VRAD
# runs on it. Reported per run:
#   PropStage  - VRAD's own "StaticPropLighting: N seconds" line
#   Cores      - average busy cores while that stage ran (CPU time / wall time)
#   PeakMB     - peak working set of the whole run
#
#   .\bench_prop_lightmap.ps1 -Map unit_test_maps\validation_vrad_staticpropbounce.bsp -Resolutions 256,512,1024
#
# -Exe defaults to vrad_rtx.exe beside this script; pass another build to A/B.
param(
    [Parameter(Mandatory)] [string]$Map,
    [int[]]$Resolutions = @(256, 512, 1024),
    [string]$Props = "",
    [string]$Exe = (Join-Path $PSScriptRoot "vrad_rtx.exe"),
    [string]$Game = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\sourcetest",
    [string[]]$VradArgs = @("-StaticPropLighting", "-StaticPropPolys", "-final"),
    [string]$OutDir = (Join-Path ([IO.Path]::GetTempPath()) "bench_prop_lightmap"),
    [int]$TimeoutMinutes = 120
)

$ErrorActionPreference = "Stop"
New-Item -ItemType Directory -Force $OutDir | Out-Null
$Map = (Resolve-Path $Map).Path
$Exe = (Resolve-Path $Exe).Path
$tool = Join-Path $PSScriptRoot "sprp_lightmap.py"
$base = [IO.Path]::GetFileNameWithoutExtension($Map)
$results = @()

foreach ($res in $Resolutions) {
    $bsp = Join-Path $OutDir "${base}_$res.bsp"
    $log = Join-Path $OutDir "${base}_$res.log"
    $setArgs = @($tool, "set", $Map, $bsp, "--res", $res, "--only")
    if ($Props) { $setArgs += @("--props", $Props) }
    python @setArgs | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "sprp_lightmap.py failed for $res" }

    $argList = @($VradArgs) + @("-game", "`"$Game`"", "`"$bsp`"")
    $sw = [Diagnostics.Stopwatch]::StartNew()
    $p = Start-Process $Exe -ArgumentList $argList -WorkingDirectory (Split-Path $Exe) `
        -RedirectStandardOutput $log -RedirectStandardError "$log.err" -NoNewWindow -PassThru
    # VRAD's stdout is block-buffered, so the log can't mark when the stage
    # starts. Keep a (time, cpu) history instead; the prop stage is the last
    # long stage of a run, so its window is the final PropStage seconds.
    $peak = 0; $hist = [Collections.Generic.List[double[]]]::new()
    while (-not $p.HasExited) {
        try {
            $p.Refresh()
            $peak = [math]::Max($peak, $p.PeakWorkingSet64)
            $hist.Add(@($sw.Elapsed.TotalSeconds, $p.TotalProcessorTime.TotalSeconds))
        } catch {}
        if ($sw.Elapsed.TotalMinutes -gt $TimeoutMinutes) { $p.Kill(); break }
        Start-Sleep -Milliseconds 250
    }
    $wall = $sw.Elapsed.TotalSeconds
    $text = Get-Content $log -Raw
    $stage = if ($text -match "StaticPropLighting:\s*([\d.]+)\s*seconds") { [double]$Matches[1] } else { $null }
    $cores = $null
    if ($stage -and $stage -gt 2 -and $hist.Count -gt 4) {
        $last = $hist[$hist.Count - 1]
        $first = $hist | Where-Object { $_[0] -ge $last[0] - $stage + 0.5 } | Select-Object -First 1
        if ($first -and ($last[0] - $first[0]) -gt 1) {
            $cores = [math]::Round(($last[1] - $first[1]) / ($last[0] - $first[0]), 1)
        }
    }
    $proptexel = ($text -split "`n" | Where-Object { $_ -match "^\[proptexel\]" }) -join "`n"
    if ($proptexel) { Write-Host $proptexel }
    $row = [pscustomobject]@{
        Res = $res; MTexels = [math]::Round($res * $res / 1e6, 2); PropStage = $stage
        Wall = [math]::Round($wall, 1); Cores = $cores; PeakMB = [math]::Round($peak / 1MB)
        Exit = $p.ExitCode
    }
    $row | Format-Table -HideTableHeaders | Out-String | Write-Host
    $results += $row
}

$results | Format-Table -AutoSize
