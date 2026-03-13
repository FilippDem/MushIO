@echo off
:: MushIO V1.0 GUI Launcher
:: Checks Python, installs / updates packages, then launches the GUI.
:: Pass --demo to start in demo mode, or any other flags through to gui.py.
::
:: Usage:
::   launch_gui.bat            (connect to Pico hardware)
::   launch_gui.bat --demo     (simulated data, no hardware needed)

setlocal EnableDelayedExpansion
cd /d "%~dp0"

echo ============================================================
echo  MushIO V1.0  ^|  Real-Time Biopotential Monitor
echo ============================================================
echo.

:: ─────────────────────────────────────────────────────────────
::  1.  Locate Python
:: ─────────────────────────────────────────────────────────────
where python >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] Python not found.
    echo         Install Python 3.8+ from https://www.python.org/downloads/
    echo         Make sure to tick "Add Python to PATH" during install.
    echo.
    pause
    exit /b 1
)
for /f "tokens=*" %%v in ('python --version 2^>^&1') do set PYVER=%%v
echo [OK]    %PYVER%

:: ─────────────────────────────────────────────────────────────
::  2.  Ensure pip is available
:: ─────────────────────────────────────────────────────────────
python -m pip --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [INFO]  pip not found ^— bootstrapping with ensurepip...
    python -m ensurepip --upgrade --quiet
)

:: ─────────────────────────────────────────────────────────────
::  3.  Install required packages (silent if already present)
:: ─────────────────────────────────────────────────────────────
echo [INFO]  Checking required packages ^(matplotlib, numpy^)...
python -m pip install --quiet --exists-action i matplotlib numpy
if %errorlevel% neq 0 (
    echo [WARN]  Could not install core packages. GUI may not start.
) else (
    echo [OK]    Required packages present.
)

:: ─────────────────────────────────────────────────────────────
::  4.  Optional packages — offer to install if absent
:: ─────────────────────────────────────────────────────────────
python -c "import scipy" >nul 2>&1
if %errorlevel% neq 0 (
    echo.
    echo [OPT]   scipy is not installed ^(enables the bandpass ^/ notch filter^).
    set /p _INST_SCIPY="         Install scipy now? [Y/N]: "
    if /i "!_INST_SCIPY!"=="Y" (
        python -m pip install --quiet scipy
        if !errorlevel! equ 0 (
            echo [OK]    scipy installed.
        ) else (
            echo [WARN]  scipy install failed ^— bandpass filter will be disabled.
        )
    )
)

python -c "import cv2" >nul 2>&1
if %errorlevel% neq 0 (
    echo.
    echo [OPT]   opencv-python is not installed ^(enables the webcam overlay^).
    set /p _INST_CV="         Install opencv-python now? [Y/N]: "
    if /i "!_INST_CV!"=="Y" (
        python -m pip install --quiet opencv-python
        if !errorlevel! equ 0 (
            echo [OK]    opencv-python installed.
        ) else (
            echo [WARN]  opencv-python install failed ^— webcam overlay will be disabled.
        )
    )
)

:: ─────────────────────────────────────────────────────────────
::  5.  Check for outdated packages (skip if slow — 10 s cap)
:: ─────────────────────────────────────────────────────────────
echo.
echo [INFO]  Checking for package updates ^(10 s timeout^)...
set "_OUTDATED=%TEMP%\mushio_outdated.txt"

:: Write empty file first so FOR loop won't fail if Python times out
echo.>"%_OUTDATED%"
python -c "import subprocess,sys,os; pkgs={'matplotlib','numpy','scipy','opencv-python'}; r=subprocess.run([sys.executable,'-m','pip','list','--outdated','--format=columns'],capture_output=True,text=True,timeout=10); lines=[l for l in r.stdout.splitlines()[2:] if l.split() and l.split()[0].lower() in pkgs]; open(os.environ['_OUTDATED'],'w').write('\n'.join(lines))" 2>nul
if %errorlevel% neq 0 (
    echo [OK]    Skipped ^(network slow or unavailable^).
    goto :skip_updates
)

:: Check whether the file has any content (at least one line = at least one outdated pkg)
set "_HAS_UPDATES="
for /f "usebackq tokens=1" %%p in ("%_OUTDATED%") do set "_HAS_UPDATES=1"

if defined _HAS_UPDATES (
    echo.
    echo [UPDATE] Updates available for the following packages:
    echo.
    echo    Package             Installed    Latest
    echo    ------------------  -----------  -------
    type "%_OUTDATED%"
    echo.
    set /p _DO_UPD="         Upgrade all now? [Y/N]: "
    if /i "!_DO_UPD!"=="Y" (
        echo.
        for /f "usebackq tokens=1" %%p in ("%_OUTDATED%") do (
            echo [INFO]    Upgrading %%p ...
            python -m pip install --upgrade --quiet %%p
            if !errorlevel! equ 0 (
                echo [OK]      %%p updated.
            ) else (
                echo [WARN]    %%p update failed.
            )
        )
        echo.
        echo [OK]    All upgrades complete.
    ) else (
        echo [INFO]  Skipped. Run:  pip install --upgrade ^<package^>  to update manually.
    )
) else (
    echo [OK]    All packages are up to date.
)
:skip_updates
del "%_OUTDATED%" >nul 2>&1

:: ─────────────────────────────────────────────────────────────
::  6.  Launch the GUI
:: ─────────────────────────────────────────────────────────────
echo.
echo [INFO]  Launching MushIO V1.0 GUI...
echo.
python host\gui.py %*

:: Keep the window open on crash so the user can read the error
if %errorlevel% neq 0 (
    echo.
    echo [ERROR] GUI exited with error code %errorlevel%.
    echo         Check the output above for details.
    pause
)
endlocal
