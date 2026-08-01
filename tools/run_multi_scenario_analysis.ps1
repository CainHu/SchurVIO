param(
    [double]$Duration = 30,
    [int]$Features = 600,
    [string]$BuildDirectory = "cmake-build-release"
)

$ErrorActionPreference = "Stop"
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot ".."))
$buildPath = [IO.Path]::GetFullPath((Join-Path $projectRoot $BuildDirectory))
$analysis = Join-Path $buildPath "VinsAnalysis.exe"
$report = Join-Path $buildPath "VinsReport.exe"

if (-not (Test-Path -LiteralPath $analysis) -or -not (Test-Path -LiteralPath $report)) {
    throw "Build VinsAnalysis and VinsReport first: cmake --build $BuildDirectory --target VinsAnalysis VinsReport"
}

# Add the configured MinGW compiler directory to PATH so its runtime DLLs are found.
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

Push-Location $projectRoot
try {
    $scenarios = @("circle_out", "circle_in", "helix_3d", "stop_go")

    # circle_out/base must run first because it recreates summary.csv.
    foreach ($scenario in $scenarios) {
        & $analysis "0.01" "base" "1.0" $scenario "$Duration" "$Features"
        if ($LASTEXITCODE -ne 0) { throw "Scenario $scenario failed with exit code $LASTEXITCODE" }
    }

    # Keep parameter sweeps on circle_out; the four base rows measure cross-scenario robustness.
    foreach ($uv in @("0.0001", "0.001", "0.003", "0.01", "0.03", "0.1")) {
        $tag = "uv_" + $uv.Replace(".", "p")
        & $analysis $uv $tag "1.0" "circle_out" "$Duration" "$Features"
        if ($LASTEXITCODE -ne 0) { throw "uv_var=$uv failed with exit code $LASTEXITCODE" }
    }

    foreach ($scale in @(@("0.5", "proc_0p5"), @("2.0", "proc_2p0"))) {
        & $analysis "0.01" $scale[1] $scale[0] "circle_out" "$Duration" "$Features"
        if ($LASTEXITCODE -ne 0) { throw "process scale=$($scale[0]) failed with exit code $LASTEXITCODE" }
    }

    & $report
    if ($LASTEXITCODE -ne 0) { throw "Report generation failed with exit code $LASTEXITCODE" }
    Write-Host "Done: $projectRoot\out\report.html"
}
finally {
    Pop-Location
}
