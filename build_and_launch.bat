@echo off
setlocal

set CMAKE=C:\Users\filip\scoop\apps\cmake\4.2.3\bin
set NINJA=C:\Users\filip\scoop\apps\ninja\1.13.2
set GCC=C:\Users\filip\scoop\apps\gcc-arm-none-eabi\15.2.rel1\bin

:: Find mingw bin dir (for make/sh used by some cmake scripts)
for /d %%d in (C:\Users\filip\scoop\apps\mingw\*) do set MINGW=%%d\bin

set PATH=%CMAKE%;%NINJA%;%GCC%;%MINGW%;%PATH%
set PICO_SDK_PATH=C:\pico\pico-sdk

set ROOT=%~dp0
set FW_DIR=%ROOT%firmware_c
set BUILD_DIR=%FW_DIR%\build

echo ============================================================
echo  MushIO V1.0  --  Build + Flash + GUI launcher
echo ============================================================

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
:: Auto-detect Pico IP from active data-stream connection on port 9000
set PICO_IP=
for /f "tokens=3" %%i in ('netstat -an 2^>nul ^| findstr ":9000 " ^| findstr "ESTABLISHED"') do (
    if not defined PICO_IP (
        for /f "tokens=1 delims=:" %%a in ("%%i") do set PICO_IP=%%a
    )
)
:: Try OTA first (Pico already running firmware)
if defined PICO_IP (
    echo [OTA] Detected Pico at %PICO_IP% via port-9000 data stream.
    python "%ROOT%host\ota_client.py" "%UF2%" --host %PICO_IP% 2>nul
) else (
    python "%ROOT%host\ota_client.py" "%UF2%" 2>nul
)
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
