param(
    [double]$Duration = 30,
    [int]$Features = 600,
    [string]$BuildDirectory = "cmake-build-release",
    [double]$TriangulationMinParallaxDeg = 8,
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

# The three requested candidates are Fixed, Retriangulate and SchurBackSubstitution.
# IndependentEkf is retained as the historical reference, not as a recommended policy.
$configs = @(
    [pscustomobject]@{ Tag = "lmk_fixed"; Mode = "fixed" },
    [pscustomobject]@{ Tag = "lmk_retri"; Mode = "retriangulate" },
    [pscustomobject]@{ Tag = "lmk_schur"; Mode = "schur" },
    [pscustomobject]@{ Tag = "lmk_legacy"; Mode = "independent" }
)
$scenarios = @("circle_out", "circle_in", "helix_3d", "stop_go")

Push-Location $projectRoot
try {
    $runningAnalysis = Get-Process -Name "VinsAnalysis" -ErrorAction SilentlyContinue
    if ($runningAnalysis) {
        throw "Another VinsAnalysis process is still running (PID $($runningAnalysis.Id -join ', '))."
    }

    foreach ($config in $configs) {
        foreach ($scenario in $scenarios) {
            Write-Host "Landmark $($config.Mode) / $scenario"
            & $analysis "0.01" $config.Tag "1.0" $scenario "$Duration" "$Features" `
                "tri" "1" "density" "1" "landmark" "1" "0" $config.Mode `
                "$TriangulationMinParallaxDeg"
            if ($LASTEXITCODE -ne 0) {
                throw "Landmark $($config.Mode) / $scenario failed with exit code $LASTEXITCODE"
            }
        }
    }

    Write-Host "Done: $projectRoot\out\landmark_strategy_summary.csv"
}
finally {
    Pop-Location
}
