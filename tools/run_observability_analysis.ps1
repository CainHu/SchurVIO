param(
    [double]$Duration = 30,
    [int]$Features = 600,
    [string]$BuildDirectory = "cmake-build-release"
)

$ErrorActionPreference = "Stop"
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot ".."))
$buildPath = [IO.Path]::GetFullPath((Join-Path $projectRoot $BuildDirectory))
$analysis = Join-Path $buildPath "VinsAnalysis.exe"

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

$configs = @(
    [pscustomobject]@{ Tag = "oc_on";  Enabled = "1" },
    [pscustomobject]@{ Tag = "oc_off"; Enabled = "0" }
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
            Write-Host "Observability $($config.Tag) / $scenario"
            & $analysis "0.01" $config.Tag "1.0" $scenario "$Duration" "$Features" `
                "tri" "1" "density" "1" "observability" $config.Enabled "0"
            if ($LASTEXITCODE -ne 0) {
                throw "Observability $($config.Tag) / $scenario failed with exit code $LASTEXITCODE"
            }
        }
    }

    Write-Host "Done: $projectRoot\out\observability_summary.csv"
}
finally {
    Pop-Location
}
