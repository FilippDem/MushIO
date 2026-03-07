@echo off
setlocal

set "CMAKE_BIN=C:\Users\filip\scoop\apps\cmake\4.2.3\bin"
set "NINJA_BIN=C:\Users\filip\scoop\apps\ninja\1.13.2"
set "GCC_BIN=C:\Users\filip\scoop\apps\gcc-arm-none-eabi\15.2.rel1\bin"
set "PATH=%CMAKE_BIN%;%NINJA_BIN%;%GCC_BIN%;%PATH%"
set "PICO_SDK_PATH=C:\pico\pico-sdk"

set "BUILD=C:\Users\filip\OneDrive\Desktop\MushIV1p0\firmware_c\build"
mkdir "%BUILD%" 2>nul
cd /d "%BUILD%"

echo --- CMake configure ---
cmake "C:\Users\filip\OneDrive\Desktop\MushIV1p0\firmware_c" -G Ninja -DCMAKE_BUILD_TYPE=Release -DMUSHIO_DEMO=1
if errorlevel 1 exit /b 1

echo --- Ninja build ---
ninja -j4
