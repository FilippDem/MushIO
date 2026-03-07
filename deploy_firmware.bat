@echo off
:: MushIO V1.0 - Firmware Deploy Script
:: Copies all firmware files to the Pico 2 W over USB.
:: Run this any time you update the firmware.
::
:: Usage:
::   deploy_firmware.bat              (deploys Pico Demo firmware)
::   deploy_firmware.bat --real       (deploys real ADC firmware)

setlocal EnableDelayedExpansion
cd /d "%~dp0"

echo ============================================================
echo  MushIO V1.0  ^|  Firmware Deploy
echo ============================================================
echo.

:: ─────────────────────────────────────────────────────────────
:: 1.  Check mpremote is available
:: ─────────────────────────────────────────────────────────────
where mpremote >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] mpremote not found.
    echo         Install with:  pip install mpremote
    pause & exit /b 1
)

:: ─────────────────────────────────────────────────────────────
:: 2.  Find the Pico (filter by Raspberry Pi USB VID 0x2e8a)
:: ─────────────────────────────────────────────────────────────
echo [INFO]  Scanning for Pico 2 W...
set "PICO_PORT="
for /f "tokens=1,3" %%a in ('mpremote connect list 2^>^&1') do (
    echo %%b | findstr /i "2e8a" >nul 2>&1
    if !errorlevel! equ 0 (
        if not defined PICO_PORT set "PICO_PORT=%%a"
    )
)

if not defined PICO_PORT (
    echo [ERROR] No Pico 2 W found.
    echo         Make sure MicroPython is installed and the USB cable is plugged in.
    echo.
    echo         Available ports:
    mpremote connect list
    pause & exit /b 1
)
echo [OK]    Found Pico at %PICO_PORT%
echo.

:: ─────────────────────────────────────────────────────────────
:: 3.  Choose firmware mode
:: ─────────────────────────────────────────────────────────────
set "USE_STUB=1"
if /i "%1"=="--real" set "USE_STUB=0"

if "%USE_STUB%"=="1" (
    echo [INFO]  Mode: Pico Demo  ^(synthetic data, no ADC hardware required^)
    echo [INFO]  Uses: demo_adc_manager.py, main_test.py ^(deployed as main.py^)
) else (
    echo [INFO]  Mode: Real  ^(live ADS124S08 ADC hardware^)
    echo [INFO]  Uses: adc_manager.py, ads124s08.py, main.py
)
echo.

:: ─────────────────────────────────────────────────────────────
:: 4.  Copy shared files (same for both modes)
:: ─────────────────────────────────────────────────────────────
echo [COPY]  Shared files...

call :copy_file firmware\config.py        config.py
call :copy_file firmware\streamer.py      streamer.py
call :copy_file firmware\cmd_server.py    cmd_server.py

:: ─────────────────────────────────────────────────────────────
:: 5.  Copy mode-specific files
:: ─────────────────────────────────────────────────────────────
if "%USE_STUB%"=="1" (
    echo [COPY]  Pico Demo files...
    call :copy_file firmware\demo_adc_manager.py   demo_adc_manager.py
    call :copy_file firmware\main_test.py           main.py
) else (
    echo [COPY]  Real ADC files...
    call :copy_file firmware\ads124s08.py           ads124s08.py
    call :copy_file firmware\adc_manager.py         adc_manager.py
    call :copy_file firmware\main.py                main.py
)

:: ─────────────────────────────────────────────────────────────
:: 6.  Verify
:: ─────────────────────────────────────────────────────────────
echo.
echo [INFO]  Files on Pico:
mpremote connect %PICO_PORT% ls
echo.

:: ─────────────────────────────────────────────────────────────
:: 7.  Reboot into the new firmware
:: ─────────────────────────────────────────────────────────────
echo [INFO]  Rebooting Pico into new firmware...
mpremote connect %PICO_PORT% reset
echo.
echo [OK]    Deploy complete. Pico is starting up.
echo         Watch the LED: 5 rapid blinks = firmware running.
echo.
pause
exit /b 0

:: ─────────────────────────────────────────────────────────────
:: Helper: copy one file, print result
:: ─────────────────────────────────────────────────────────────
:copy_file
set "SRC=%~1"
set "DST=%~2"
if not exist "%SRC%" (
    echo [SKIP]    %SRC% not found - skipping
    goto :eof
)
mpremote connect %PICO_PORT% cp "%SRC%" :%DST% >nul 2>&1
if %errorlevel% equ 0 (
    echo [OK]      %SRC%  -^>  :%DST%
) else (
    echo [FAIL]    %SRC%  -^>  :%DST%
)
goto :eof
