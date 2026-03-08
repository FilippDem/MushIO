$ErrorActionPreference = "Stop"

$env:PATH = "C:\Users\filip\scoop\apps\cmake\4.2.3\bin;" +
            "C:\Users\filip\scoop\apps\ninja\1.13.2;" +
            "C:\Users\filip\scoop\apps\gcc-arm-none-eabi\15.2.rel1\bin;" +
            "C:\Users\filip\scoop\apps\mingw\current\bin;" +
            $env:PATH

$env:PICO_SDK_PATH = "C:\pico\pico-sdk"

$build = "C:\Users\filip\OneDrive\Desktop\MushIV1p0\firmware_c\build"
Set-Location $build

Write-Host "--- Ninja incremental build ---" -ForegroundColor Cyan
ninja -j4
if ($LASTEXITCODE -ne 0) { throw "Build failed" }

$uf2 = "$build\mushio_c.uf2"
if (Test-Path $uf2) {
    $size = (Get-Item $uf2).Length
    Write-Host "SUCCESS  $uf2  ($([math]::Round($size/1KB)) KB)" -ForegroundColor Green
} else {
    throw "UF2 not found after build"
}
