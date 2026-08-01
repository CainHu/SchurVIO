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
    foreach ($parallax in @(3, 5, 7, 8, 10)) {
        $tag = "tri_p$parallax"
        Write-Host "Circle-out triangulation / $parallax deg"
        & $analysis "0.01" $tag "1.0" "circle_out" "$Duration" "$Features" `
            "tri" "1" "density" "1" "triangulation" "1" "0" `
            "retriangulate" "$parallax"
        if ($LASTEXITCODE -ne 0) {
            throw "Triangulation threshold $parallax failed with exit code $LASTEXITCODE"
        }
    }
    Write-Host "Done: $projectRoot\out\triangulation_threshold_summary.csv"
}
finally {
    Pop-Location
}
