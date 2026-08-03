param(
    [double]$Duration = 30,
    [int]$Features = 600,
    [int]$RetainedClones = 20,
    [string]$BuildDirectory = "cmake-build-release",
    [int]$Jobs = 4,
    [switch]$Quick
)

$ErrorActionPreference = "Stop"
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot ".."))
$buildPath = [IO.Path]::GetFullPath((Join-Path $projectRoot $BuildDirectory))
$analysis = Join-Path $buildPath "VinsAnalysis.exe"
$currentPolicy = ""

if ($RetainedClones -lt 1 -or $RetainedClones -gt 29) {
    throw "RetainedClones must be in [1, 29]"
}

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
        -Pattern '^CMAKE_CXX_COMPILER:(?:STRING|FILEPATH|UNINITIALIZED)=(.+)$' |
        Select-Object -First 1
    if ($compilerLine -and $compilerLine.Matches.Count) {
        $compilerDirectory = Split-Path $compilerLine.Matches[0].Groups[1].Value
        if ($compilerDirectory -and (Test-Path -LiteralPath $compilerDirectory)) {
            $env:Path = $compilerDirectory + [IO.Path]::PathSeparator + $env:Path
        }
    }
}

function Select-FramePolicy([string]$policy) {
    Write-Host "Configure MSCKF frame policy: $policy / clones=$RetainedClones"
    & cmake -S $projectRoot -B $buildPath `
        "-DSCHUR_VIO_VISUAL_SCHEDULER=MSCKF" `
        "-DSCHUR_VIO_FRAME_POLICY=$policy" `
        "-DSCHUR_VIO_FRAME_WINDOW_SIZE=$RetainedClones" `
        "-DSCHUR_VIO_LANDMARK_PARAMETERIZATION=WORLD_XYZ"
    if ($LASTEXITCODE -ne 0) { throw "CMake configure failed for $policy" }
    & cmake --build $buildPath --target VinsAnalysis -j $Jobs
    if ($LASTEXITCODE -ne 0) { throw "Build failed for $policy" }
    $script:currentPolicy = $policy
    Add-CompilerRuntimeToPath
}

$policies = @(
    "KEYFRAME_ONLY",
    "KEYFRAME_REDUNDANCY",
    "FIFO",
    "KEYFRAME_PRIORITY",
    "VINS_MONO",
    "RDVIO"
)

Push-Location $projectRoot
try {
    foreach ($policy in $policies) {
        Select-FramePolicy $policy
        $tag = "frame_" + $policy.ToLowerInvariant()
        foreach ($scenario in $scenarios) {
            Write-Host "Frame policy $policy / $scenario"
            & $analysis "0.01" $tag "1.0" $scenario "$Duration" "$Features" `
                "tri" "1" "density" "1" "frame_policy" "1" "0" `
                "retriangulate" "2.0" "0" "0.001" "1.0"
            if ($LASTEXITCODE -ne 0) {
                throw "Frame policy $policy / $scenario failed with exit code $LASTEXITCODE"
            }
        }
    }

    $summaryPath = Join-Path $projectRoot "out\frame_policy_summary.csv"
    $rows = Import-Csv -LiteralPath $summaryPath
    foreach ($row in $rows) {
        if ($row.scheduler -ne "msckf" -or [int]$row.one_shot -ne 1) {
            throw "Frame-policy comparison did not keep the MSCKF backend"
        }
        if ([int]$row.retained_clones -ne $RetainedClones) {
            throw "Frame-policy comparison used an inconsistent clone budget"
        }
        if ([int64]$row.reused_observations -ne 0 -or
            [int64]$row.duplicate_observations_blocked -ne 0) {
            throw "MSCKF one-shot invariant failed for $($row.frame_policy)/$($row.scenario)"
        }
        if ($row.frame_policy -eq "rdvio" -and
            ([int64]$row.rotation_constraints -ne 0 -or
             [int64]$row.zero_translation_constraints -ne 0)) {
            throw "RDVIO frame-policy ablation leaked RDVIO backend constraints"
        }
    }
    Write-Host "Done: $summaryPath"
}
finally {
    if ($currentPolicy -ne "AUTO") {
        & cmake -S $projectRoot -B $buildPath `
            "-DSCHUR_VIO_VISUAL_SCHEDULER=MSCKF" `
            "-DSCHUR_VIO_FRAME_POLICY=AUTO" `
            "-DSCHUR_VIO_FRAME_WINDOW_SIZE=0" `
            "-DSCHUR_VIO_LANDMARK_PARAMETERIZATION=WORLD_XYZ"
        if ($LASTEXITCODE -eq 0) {
            & cmake --build $buildPath --target VinsAnalysis -j $Jobs
        }
    }
    Pop-Location
}
