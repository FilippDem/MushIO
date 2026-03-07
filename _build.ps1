$ErrorActionPreference = "Stop"

# Build toolchain — cross compiler for Pico, MinGW for native host tools
# (pioasm and picotool are host-native builds that need a native C++ compiler)
$env:PATH = "C:\Users\filip\scoop\apps\cmake\4.2.3\bin;" +
            "C:\Users\filip\scoop\apps\ninja\1.13.2;" +
            "C:\Users\filip\scoop\apps\gcc-arm-none-eabi\15.2.rel1\bin;" +
            "C:\Users\filip\scoop\apps\mingw\current\bin;" +
            $env:PATH

$env:PICO_SDK_PATH = "C:\pico\pico-sdk"

$src   = "C:\Users\filip\OneDrive\Desktop\MushIV1p0\firmware_c"
$build = "$src\build"

# Wipe and recreate build dir for a clean build
if (Test-Path $build) { Remove-Item $build -Recurse -Force }
New-Item -ItemType Directory -Force -Path $build | Out-Null
Set-Location $build

Write-Host "--- CMake configure (pico2_w, DEMO=1) ---" -ForegroundColor Cyan
cmake $src -G Ninja -DCMAKE_BUILD_TYPE=Release "-DMUSHIO_DEMO=1"
if ($LASTEXITCODE -ne 0) { throw "CMake configure failed" }

Write-Host "--- Ninja build ---" -ForegroundColor Cyan
ninja -j4
if ($LASTEXITCODE -ne 0) { throw "Build failed" }

$uf2 = "$build\mushio_c.uf2"
if (Test-Path $uf2) {
    $size = (Get-Item $uf2).Length
    Write-Host "SUCCESS  $uf2  ($([math]::Round($size/1KB)) KB)" -ForegroundColor Green
} else {
    throw "UF2 not found after build"
}
