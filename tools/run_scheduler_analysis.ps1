param(
    [double]$Duration = 30,
    [int]$Features = 600,
    [string]$BuildDirectory = "cmake-build-release",
    [int]$Jobs = 4,
    [switch]$Quick
)

$ErrorActionPreference = "Stop"
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot ".."))
$buildPath = [IO.Path]::GetFullPath((Join-Path $projectRoot $BuildDirectory))
$analysis = Join-Path $buildPath "VinsAnalysis.exe"
$currentScheduler = ""

if ($Quick) {
    $Duration = [Math]::Min($Duration, 5)
    $Features = [Math]::Min($Features, 150)
    $scenarios = @("circle_out", "rotation_translation")
}
else {
    $scenarios = @("circle_out", "circle_in", "helix_3d", "stop_go", "rotation_translation")
}

function Add-CompilerRuntimeToPath {
    $cache = Join-Path $buildPath "CMakeCache.txt"
    if (-not (Test-Path -LiteralPath $cache)) { return }
    $compilerLine = Select-String -Path $cache `
        -Pattern '^CMAKE_CXX_COMPILER:(?:STRING|FILEPATH)=(.+)$' | Select-Object -First 1
    if ($compilerLine -and $compilerLine.Matches.Count) {
        $compilerDirectory = Split-Path $compilerLine.Matches[0].Groups[1].Value
        if ($compilerDirectory -and (Test-Path -LiteralPath $compilerDirectory)) {
            $env:Path = $compilerDirectory + [IO.Path]::PathSeparator + $env:Path
        }
    }
}

function Select-Scheduler([string]$scheduler) {
    Write-Host "Configure visual scheduler: $scheduler"
    & cmake -S $projectRoot -B $buildPath "-DSCHUR_VIO_VISUAL_SCHEDULER=$scheduler"
    if ($LASTEXITCODE -ne 0) { throw "CMake configure failed for $scheduler" }
    & cmake --build $buildPath --target VinsAnalysis -j $Jobs
    if ($LASTEXITCODE -ne 0) { throw "Build failed for $scheduler" }
    $script:currentScheduler = $scheduler
    Add-CompilerRuntimeToPath
}

# Legacy must run first because its first circle_out row recreates the summary.
# MSCKF runs last so the build directory is left on the recommended default.
$schedulers = @("LEGACY", "SCHURVINS", "VINS_MONO", "RDVIO", "MSCKF")

Push-Location $projectRoot
try {
    foreach ($scheduler in $schedulers) {
        Select-Scheduler $scheduler
        $tag = "scheduler_" + $scheduler.ToLowerInvariant()
        foreach ($scenario in $scenarios) {
            Write-Host "Scheduler $scheduler / $scenario"
            & $analysis "0.01" $tag "1.0" $scenario "$Duration" "$Features" `
                "tri" "1" "density" "1" "scheduler" "1" "0" `
                "retriangulate" "3.0" "0" "0.001" "1.0"
            if ($LASTEXITCODE -ne 0) {
                throw "Scheduler $scheduler / $scenario failed with exit code $LASTEXITCODE"
            }
        }
    }

    $summaryPath = Join-Path $projectRoot "out\scheduler_summary.csv"
    $msckfRows = Import-Csv -LiteralPath $summaryPath | Where-Object scheduler -eq "msckf"
    foreach ($row in $msckfRows) {
        if ([int64]$row.reused_observations -ne 0 -or
            [int64]$row.duplicate_observations_blocked -ne 0) {
            throw "MSCKF one-shot invariant failed for $($row.scenario)"
        }
    }
    Write-Host "Done: $summaryPath"
}
finally {
    # Keep normal development builds on the recommended scheduler even if an
    # earlier experiment fails. Rebuild only when the last successful mode was
    # not already MSCKF.
    if ($currentScheduler -ne "MSCKF") {
        & cmake -S $projectRoot -B $buildPath "-DSCHUR_VIO_VISUAL_SCHEDULER=MSCKF"
        if ($LASTEXITCODE -eq 0) {
            & cmake --build $buildPath --target VinsAnalysis -j $Jobs
        }
    }
    Pop-Location
}
