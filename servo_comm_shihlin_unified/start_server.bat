@echo off
setlocal
cd /d "%~dp0"

echo ============================================
echo  Servo Control Server (servo_comm_shihlin_unified)
echo ============================================
echo.

where python >nul 2>nul
if errorlevel 1 (
    echo ERROR: Python was not found on PATH.
    echo Install Python 3.9+ from https://www.python.org/downloads/
    echo ^(check "Add python.exe to PATH" during install^) and try again.
    pause
    exit /b 1
)

echo Checking/installing dependencies ^(pyserial, flask, python-osc^)...
python -m pip install -q -r requirements_pc.txt
if errorlevel 1 (
    echo ERROR: Failed to install dependencies. See the message above.
    pause
    exit /b 1
)

echo.
echo Make sure the RS-485 USB adapter is plugged in before continuing.
echo The server will auto-detect its COM port.
echo.
echo Opening http://localhost:5000 in your browser in a few seconds...
start "" cmd /c "timeout /t 3 >nul & start http://localhost:5000"

echo Starting the server. Press Ctrl+C in this window to stop it.
echo ============================================
echo.
python app.py

echo.
echo Server stopped.
pause
