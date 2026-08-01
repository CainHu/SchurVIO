param(
    [double]$Duration = 30,
    [int]$Features = 600,
    [string]$BuildDirectory = "cmake-build-release",
    [switch]$Resume,
    [string[]]$ForceTags = @(),
    [bool]$ObservabilityConstraint = $true,
    [bool]$ObservabilityProjection = $false
)

$ErrorActionPreference = "Stop"
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot ".."))
$buildPath = [IO.Path]::GetFullPath((Join-Path $projectRoot $BuildDirectory))
$analysis = Join-Path $buildPath "VinsAnalysis.exe"
$report = Join-Path $buildPath "VinsReport.exe"

if (-not (Test-Path -LiteralPath $analysis) -or -not (Test-Path -LiteralPath $report)) {
    throw "Build VinsAnalysis and VinsReport first: cmake --build $BuildDirectory --target VinsAnalysis VinsReport"
}

# Locate the MinGW runtime used by this build.
$cache = Join-Path $buildPath "CMakeCache.txt"
if (Test-Path -LiteralPath $cache) {
    $compilerLine = Select-String -Path $cache -Pattern '^CMAKE_CXX_COMPILER:(?:STRING|FILEPATH)=(.+)$' |
        Select-Object -First 1
    if ($compilerLine -and $compilerLine.Matches.Count) {
        $compilerDirectory = Split-Path $compilerLine.Matches[0].Groups[1].Value
        if ($compilerDirectory -and (Test-Path -LiteralPath $compilerDirectory)) {
            $env:Path = $compilerDirectory + [IO.Path]::PathSeparator + $env:Path
        }
    }
}

# OFAT rows differ from abl_full in exactly one factor. abl_old_pipeline is a
# separate interaction check reproducing the old GT/quiet-IMU/no-bias-RW flow.
$configs = @(
    [pscustomobject]@{ Tag = "abl_full";         Init = "tri"; Refine = "1"; Noise = "density"; BiasRw = "1" },
    [pscustomobject]@{ Tag = "abl_gt_init";      Init = "tri_gt"; Refine = "1"; Noise = "density"; BiasRw = "1" },
    [pscustomobject]@{ Tag = "abl_no_refine";    Init = "tri"; Refine = "0"; Noise = "density"; BiasRw = "1" },
    [pscustomobject]@{ Tag = "abl_legacy_white"; Init = "tri"; Refine = "1"; Noise = "legacy";  BiasRw = "1" },
    [pscustomobject]@{ Tag = "abl_no_bias_rw";   Init = "tri"; Refine = "1"; Noise = "density"; BiasRw = "0" },
    [pscustomobject]@{ Tag = "abl_old_pipeline"; Init = "gt";  Refine = "1"; Noise = "legacy";  BiasRw = "0" }
)
$scenarios = @("circle_out", "circle_in", "helix_3d", "stop_go")

function Remove-DuplicateAblationRows([string]$Path) {
    if (-not (Test-Path -LiteralPath $Path)) { return 0 }
    $lines = [IO.File]::ReadAllLines($Path)
    if ($lines.Count -lt 2) { return 0 }
    $seen = [Collections.Generic.HashSet[string]]::new()
    $keptReverse = [Collections.Generic.List[string]]::new()
    for ($index = $lines.Count - 1; $index -ge 1; --$index) {
        $columns = $lines[$index].Split(',')
        if ($columns.Count -lt 2) { continue }
        $key = "$($columns[0])|$($columns[1])"
        if ($seen.Add($key)) { $keptReverse.Add($lines[$index]) }
    }
    $keptReverse.Reverse()
    $output = [Collections.Generic.List[string]]::new()
    $output.Add($lines[0])
    $output.AddRange($keptReverse)
    [IO.File]::WriteAllLines($Path, $output)
    return ($lines.Count - $output.Count)
}

Push-Location $projectRoot
try {
    $runningAnalysis = Get-Process -Name "VinsAnalysis" -ErrorAction SilentlyContinue
    if ($runningAnalysis) {
        throw "Another VinsAnalysis process is still running (PID $($runningAnalysis.Id -join ', ')). Wait for it before starting or resuming."
    }

    $completed = @{}
    $summaryPath = Join-Path $projectRoot "out\ablation_summary.csv"
    if ($Resume -and (Test-Path -LiteralPath $summaryPath)) {
        $removed = Remove-DuplicateAblationRows $summaryPath
        if ($removed) { Write-Host "Removed $removed duplicate summary row(s) before resume" }
        foreach ($row in (Import-Csv -LiteralPath $summaryPath)) {
            $completed["$($row.scenario)|$($row.tag)"] = $true
        }
        Write-Host "Resume mode: $($completed.Count) completed rows found"
    }

    foreach ($config in $configs) {
        foreach ($scenario in $scenarios) {
            $key = "$scenario|$($config.Tag)"
            if ($Resume -and $completed.ContainsKey($key) -and $ForceTags -notcontains $config.Tag) {
                Write-Host "Skip completed $($config.Tag) / $scenario"
                continue
            }
            Write-Host "Ablation $($config.Tag) / $scenario"
            $ocEnabled = if ($ObservabilityConstraint) { "1" } else { "0" }
            $ocProjection = if ($ObservabilityProjection) { "1" } else { "0" }
            & $analysis "0.01" $config.Tag "1.0" $scenario "$Duration" "$Features" `
                $config.Init $config.Refine $config.Noise $config.BiasRw "ablation" `
                $ocEnabled $ocProjection "independent" "5.0"
            if ($LASTEXITCODE -ne 0) {
                throw "Ablation $($config.Tag) / $scenario failed with exit code $LASTEXITCODE"
            }
        }
    }

    $removed = Remove-DuplicateAblationRows $summaryPath
    if ($removed) { Write-Host "Removed $removed duplicate summary row(s)" }
    & $report
    if ($LASTEXITCODE -ne 0) { throw "Report generation failed with exit code $LASTEXITCODE" }
    Write-Host "Done: $projectRoot\out\ablation_summary.csv"
    Write-Host "Report: $projectRoot\out\report.html"
}
finally {
    Pop-Location
}
