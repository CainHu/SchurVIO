param(
    [double]$Duration = 30,
    [int]$Features = 600,
    [string]$BuildDirectory = "cmake-build-release",
    [double]$TriangulationMinParallaxDeg = 8,
    [double]$LandmarkProcessNoiseDensity = 1e-3,
    [double]$AdaptiveInflationGain = 1,
    [string]$OnlyTag = "",
    [switch]$Quick
)

$ErrorActionPreference = "Stop"
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot ".."))
$buildPath = [IO.Path]::GetFullPath((Join-Path $projectRoot $BuildDirectory))
$analysis = Join-Path $buildPath "VinsAnalysis.exe"

if ($Quick) {
    $Duration = [Math]::Min($Duration, 8)
    $Features = [Math]::Min($Features, 250)
}

if (-not (Test-Path -LiteralPath $analysis)) {
    throw "Build VinsAnalysis first: cmake --build $BuildDirectory --target VinsAnalysis"
}

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

# The first three rows isolate covariance treatment while preserving the old
# independent-EKF point update. The last two compare the production map with a
# detached shadow post-processor that cannot feed covariance back into ESKF.
$configs = @(
    [pscustomobject]@{ Tag = "lmk_independent"; Mode = "independent";          Shadow = "0" },
    [pscustomobject]@{ Tag = "lmk_fixed_infl"; Mode = "independent_fixed";    Shadow = "0" },
    [pscustomobject]@{ Tag = "lmk_adapt_infl"; Mode = "independent_adaptive"; Shadow = "0" },
    [pscustomobject]@{ Tag = "lmk_retri";       Mode = "retriangulate";       Shadow = "0" },
    [pscustomobject]@{ Tag = "lmk_retri_shadow"; Mode = "retriangulate";      Shadow = "1" }
)
if ($OnlyTag) {
    $configs = @($configs | Where-Object Tag -eq $OnlyTag)
    if ($configs.Count -eq 0) {
        throw "Unknown config tag: $OnlyTag"
    }
}
$scenarios = @("circle_out", "circle_in", "helix_3d", "stop_go")

Push-Location $projectRoot
try {
    $runningAnalysis = Get-Process -Name "VinsAnalysis" -ErrorAction SilentlyContinue
    if ($runningAnalysis) {
        throw "Another VinsAnalysis process is still running (PID $($runningAnalysis.Id -join ', '))."
    }

    foreach ($config in $configs) {
        foreach ($scenario in $scenarios) {
            Write-Host "Landmark consistency $($config.Tag) / $scenario"
            & $analysis "0.01" $config.Tag "1.0" $scenario "$Duration" "$Features" `
                "tri" "1" "density" "1" "landmark_consistency" "1" "0" `
                $config.Mode "$TriangulationMinParallaxDeg" $config.Shadow `
                "$LandmarkProcessNoiseDensity" "$AdaptiveInflationGain"
            if ($LASTEXITCODE -ne 0) {
                throw "$($config.Tag) / $scenario failed with exit code $LASTEXITCODE"
            }
        }
    }

    Write-Host "Done: $projectRoot\out\landmark_consistency_summary.csv"
}
finally {
    Pop-Location
}
