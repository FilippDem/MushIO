@echo off
setlocal EnableDelayedExpansion

:: ============================================================
::  MushIO V1.0  --  Build + Flash + GUI launcher
::
::  Toolchain discovery (checked in this order):
::    1. Environment variables: CMAKE_BIN, NINJA_BIN, GCC_ARM_BIN, PICO_SDK_PATH
::    2. Tools already on PATH (cmake, ninja, arm-none-eabi-gcc)
::
::  Set DEMO=1 before running for demo mode (synthetic data, no ADC hw).
:: ============================================================

set ROOT=%~dp0
set FW_DIR=%ROOT%firmware_c
set BUILD_DIR=%FW_DIR%\build

echo ============================================================
echo  MushIO V1.0  --  Build + Flash + GUI launcher
echo ============================================================

:: ---- Toolchain: add user-specified dirs to PATH if set --------
if defined CMAKE_BIN    set "PATH=%CMAKE_BIN%;%PATH%"
if defined NINJA_BIN    set "PATH=%NINJA_BIN%;%PATH%"
if defined GCC_ARM_BIN  set "PATH=%GCC_ARM_BIN%;%PATH%"
if defined MINGW_BIN    set "PATH=%MINGW_BIN%;%PATH%"

:: Verify required tools are available
where cmake >nul 2>&1
if errorlevel 1 (
    echo [ERROR] cmake not found on PATH.
    echo         Install CMake and ensure it is on your PATH,
    echo         or set CMAKE_BIN=^<path-to-cmake-bin-dir^>
    exit /b 1
)
where ninja >nul 2>&1
if errorlevel 1 (
    echo [ERROR] ninja not found on PATH.
    echo         Install Ninja and ensure it is on your PATH,
    echo         or set NINJA_BIN=^<path-to-ninja-dir^>
    exit /b 1
)
where arm-none-eabi-gcc >nul 2>&1
if errorlevel 1 (
    echo [ERROR] arm-none-eabi-gcc not found on PATH.
    echo         Install the ARM GCC toolchain and ensure it is on your PATH,
    echo         or set GCC_ARM_BIN=^<path-to-gcc-arm-bin-dir^>
    exit /b 1
)
if not defined PICO_SDK_PATH (
    echo [ERROR] PICO_SDK_PATH not set.
    echo         Clone the Pico SDK and set PICO_SDK_PATH to its location:
    echo           git clone https://github.com/raspberrypi/pico-sdk
    echo           set PICO_SDK_PATH=C:\path\to\pico-sdk
    exit /b 1
)

echo [OK] cmake:              found
echo [OK] ninja:              found
echo [OK] arm-none-eabi-gcc:  found
echo [OK] PICO_SDK_PATH:      %PICO_SDK_PATH%
echo.

:: ---- 1. CMake configure ------------------------------------------------
if not exist "%BUILD_DIR%\CMakeCache.txt" (
    echo [1/3] CMake configure...
    mkdir "%BUILD_DIR%" 2>nul
    pushd "%BUILD_DIR%"
    cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release -DMUSHIO_DEMO=%DEMO%
    if errorlevel 1 ( echo [ERROR] CMake configure failed & popd & exit /b 1 )
    popd
) else (
    echo [1/3] CMake already configured -- skipping
)

:: ---- 2. Ninja build ----------------------------------------------------
echo [2/3] Building firmware...
pushd "%BUILD_DIR%"
ninja -j4
if errorlevel 1 ( echo [ERROR] Build failed & popd & exit /b 1 )
popd

set UF2=%BUILD_DIR%\mushio_c.uf2
if not exist "%UF2%" (
    echo [ERROR] UF2 not found at %UF2%
    exit /b 1
)
echo [2/3] Built: %UF2%

:: ---- 3. Flash ----------------------------------------------------------
echo [3/3] Flashing...
:: Try OTA first (Pico already running firmware)
python "%ROOT%host\ota_client.py" "%UF2%" 2>nul
if errorlevel 1 (
    echo [INFO] OTA not available. Checking for RPI-RP2 BOOTSEL drive...
    :: Look for the drive
    for %%d in (D E F G H I J K L M N O P Q R S T U V W X Y Z) do (
        if exist "%%d:\INFO_UF2.TXT" (
            echo [INFO] Found BOOTSEL drive at %%d:
            copy /Y "%UF2%" "%%d:\" >nul
            echo [3/3] Flashed via USB copy.
            goto :flash_done
        )
    )
    echo [WARN] No BOOTSEL drive found. Put Pico in BOOTSEL mode and re-run,
    echo        OR connect a running Pico for OTA flash.
    goto :launch_gui
)
:flash_done
echo Waiting 5 s for Pico to reboot...
timeout /t 5 /nobreak >nul

:: ---- 4. Launch GUI -----------------------------------------------------
:launch_gui
echo.
echo Launching GUI...
start "" python "%ROOT%host\gui.py"
echo GUI launched. Done.
