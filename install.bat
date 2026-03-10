@echo off
echo Installing MushIO host dependencies...
pip install -r host\requirements.txt
if %ERRORLEVEL% neq 0 (
    echo.
    echo Install failed. Try: python -m pip install -r host\requirements.txt
    pause
    exit /b 1
)
echo.
echo All dependencies installed. Run with:
echo   python host\gui.py --demo
pause
