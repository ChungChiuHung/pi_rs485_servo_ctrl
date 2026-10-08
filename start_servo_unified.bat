@echo off
rem Double-click launcher for the Shihlin servo web server
rem (servo_comm_shihlin_unified) on a Windows PC. Lives at the repository
rem root. Installs the Python packages from requirements_pc.txt only when one
rem is missing or too old, then opens the browser and starts the server.
rem Optional: set SERVO_WEB_PORT (default 5000) or SERVO_SERIAL_PORT first.
setlocal
set "UNIFIED_DIR=%~dp0servo_comm_shihlin_unified"

echo ============================================
echo  Servo Control Server (servo_comm_shihlin_unified)
echo ============================================
echo.

if not exist "%UNIFIED_DIR%\app.py" (
    echo ERROR: "%UNIFIED_DIR%\app.py" was not found.
    echo Keep this file in the repository root, next to the
    echo servo_comm_shihlin_unified folder.
    goto :fail
)
cd /d "%UNIFIED_DIR%"

rem Find a working Python. "python" can be the Microsoft Store placeholder
rem that only opens the Store, so each candidate is actually run.
set "PY="
python -c "import sys" >nul 2>nul && set "PY=python"
if not defined PY (
    py -3 -c "import sys" >nul 2>nul && set "PY=py -3"
)
if not defined PY (
    echo ERROR: Python was not found.
    echo Install Python 3.9+ from https://www.python.org/downloads/
    echo ^(tick "Add python.exe to PATH" during install^) and try again.
    goto :fail
)
for /f "delims=" %%v in ('%PY% --version 2^>^&1') do echo Using %%v

rem motor_profiles.json is tuned to the connected motor and is not in git
rem (see .gitignore), so a fresh clone or pull does not have it and app.py
rem stops with "FileNotFoundError: motor_profiles.json".
if not exist "motor_profiles.json" (
    echo.
    echo ERROR: motor_profiles.json is missing from
    echo   %UNIFIED_DIR%
    echo.
    echo It holds this rig's motor settings ^(gear ratio, baud rate, home^) and is
    echo not stored in git. Copy it from this rig's previous installation, or
    echo restore the last committed version with Git and check its values:
    echo   git show a64941d~1:servo_comm_shihlin_unified/motor_profiles.json ^> servo_comm_shihlin_unified\motor_profiles.json
    goto :fail
)

echo.
echo Checking Python packages ^(requirements_pc.txt^)...
%PY% check_requirements.py requirements_pc.txt
if errorlevel 1 (
    echo Installing the missing packages...
    %PY% -m pip install -r requirements_pc.txt
    if errorlevel 1 (
        echo ERROR: Failed to install the packages. See the message above
        echo ^(an internet connection is needed the first time^).
        goto :fail
    )
) else (
    echo All required packages are already installed.
)

set "WEB_PORT=%SERVO_WEB_PORT%"
if not defined WEB_PORT set "WEB_PORT=5000"

echo.
echo Make sure the RS-485 USB adapter is plugged in before continuing.
echo The server will auto-detect its COM port.
echo.
echo Opening http://localhost:%WEB_PORT% in your browser in a few seconds...
start "" cmd /c "timeout /t 3 >nul & start http://localhost:%WEB_PORT%"

echo Starting the server. Press Ctrl+C in this window to stop it.
echo ============================================
echo.
%PY% app.py

echo.
echo Server stopped.
pause
exit /b 0

:fail
echo.
pause
exit /b 1
