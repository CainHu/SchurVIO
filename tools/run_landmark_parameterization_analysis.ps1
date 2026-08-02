param(
    [double]$Duration = 20,
    [int]$Features = 400,
    [string]$BuildDirectory = "cmake-build-release",
    [int]$Jobs = 4,
    [switch]$Quick
)

$ErrorActionPreference = "Stop"
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot ".."))
$buildPath = [IO.Path]::GetFullPath((Join-Path $projectRoot $BuildDirectory))
$analysis = Join-Path $buildPath "VinsAnalysis.exe"

if ($Quick) {
    $Duration = [Math]::Min($Duration, 5)
    $Features = [Math]::Min($Features, 150)
    $scenarios = @("circle_out")
}
else {
    $scenarios = @("circle_out", "circle_in", "helix_3d", "stop_go")
}

function Add-CompilerRuntimeToPath {
    $cache = Join-Path $buildPath "CMakeCache.txt"
    $compilerLine = Select-String -Path $cache `
        -Pattern '^CMAKE_CXX_COMPILER:(?:STRING|FILEPATH)=(.+)$' | Select-Object -First 1
    if ($compilerLine -and $compilerLine.Matches.Count) {
        $compilerDirectory = Split-Path $compilerLine.Matches[0].Groups[1].Value
        $env:Path = $compilerDirectory + [IO.Path]::PathSeparator + $env:Path
    }
}

$modes = @(
    [pscustomobject]@{ CMake = "WORLD_XYZ";         Tag = "param_world_xyz" },
    [pscustomobject]@{ CMake = "ANCHORED_XYZ";      Tag = "param_anchored_xyz" },
    [pscustomobject]@{ CMake = "ANCHORED_INV_DEPTH";Tag = "param_inv_depth_3d" },
    [pscustomobject]@{ CMake = "ANCHORED_LOG_DEPTH";Tag = "param_log_depth_3d" }
)

Push-Location $projectRoot
try {
    foreach ($mode in $modes) {
        & cmake -S $projectRoot -B $buildPath `
            "-DSCHUR_VIO_VISUAL_SCHEDULER=MSCKF" `
            "-DSCHUR_VIO_LANDMARK_PARAMETERIZATION=$($mode.CMake)"
        if ($LASTEXITCODE -ne 0) { throw "Configure failed for $($mode.CMake)" }
        & cmake --build $buildPath --target VinsAnalysis -j $Jobs
        if ($LASTEXITCODE -ne 0) { throw "Build failed for $($mode.CMake)" }
        Add-CompilerRuntimeToPath

        foreach ($scenario in $scenarios) {
            & $analysis "0.01" $mode.Tag "1.0" $scenario "$Duration" "$Features" `
                "tri" "1" "density" "1" "parameterization" "1" "0" `
                "fixed" "3.0" "0" "0.001" "1.0"
            if ($LASTEXITCODE -ne 0) {
                throw "$($mode.CMake) / $scenario failed with exit code $LASTEXITCODE"
            }
        }
    }
}
finally {
    # Restore the production defaults even when an experiment fails.
    & cmake -S $projectRoot -B $buildPath `
        "-DSCHUR_VIO_VISUAL_SCHEDULER=MSCKF" `
        "-DSCHUR_VIO_LANDMARK_PARAMETERIZATION=WORLD_XYZ"
    if ($LASTEXITCODE -eq 0) {
        & cmake --build $buildPath --target VinsAnalysis -j $Jobs
    }
    Pop-Location
}

Write-Host "Done: $(Join-Path $projectRoot 'out\parameterization_summary.csv')"
