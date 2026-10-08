@echo off
rem Starts the Shihlin servo web server (servo_comm_shihlin_unified) on a
rem Windows PC that talks to the drive through a USB-RS485 adapter.
rem
rem   start_server.bat            check this PC, install missing packages, start
rem   start_server.bat --check    only show the setup report (installs and starts nothing)
rem   start_server.bat --install  reinstall the packages even if the check passes, then start
rem
rem Only the PC packages in requirements_pc.txt are used -- never RPi.GPIO,
rem which is for the Raspberry Pi. Optional before starting: set SERVO_WEB_PORT
rem (default 5000) or SERVO_SERIAL_PORT (e.g. COM7).
setlocal
cd /d "%~dp0"

set "MODE=start"
if /i "%~1"=="--check" set "MODE=check"
if /i "%~1"=="--install" set "MODE=install"
if /i "%~1"=="--help" goto :usage
if /i "%~1"=="/?" goto :usage
if not "%~1"=="" if "%MODE%"=="start" (
    echo Unknown option: %~1
    echo.
    set "USAGE_EXIT=1"
    goto :usage
)

echo ============================================
echo  Servo Control Server (servo_comm_shihlin_unified)
echo ============================================
echo.

rem --- Python ----------------------------------------------------------------
rem "python" can be the Microsoft Store placeholder that only opens the Store,
rem so each candidate is actually run.
set "PY="
python -c "import sys" >nul 2>nul && set "PY=python"
if not defined PY (
    py -3 -c "import sys" >nul 2>nul && set "PY=py -3"
)
if not defined PY (
    echo ERROR: Python was not found.
    echo Install Python 3.9 or newer from https://www.python.org/downloads/
    echo and tick "Add python.exe to PATH" in the installer, then run this again.
    goto :fail
)
%PY% -c "import sys; sys.exit(0 if sys.version_info >= (3, 8) else 1)"
if errorlevel 1 (
    for /f "delims=" %%v in ('%PY% --version 2^>^&1') do echo ERROR: %%v is too old.
    echo Install Python 3.9 or newer from https://www.python.org/downloads/
    goto :fail
)

rem --- Setup report ----------------------------------------------------------
%PY% check_pc_setup.py requirements_pc.txt
set "SETUP_RESULT=%errorlevel%"
if exist "motor_profiles.json" (
    echo Motor profile: motor_profiles.json found
) else (
    echo Motor profile: motor_profiles.json MISSING -- see below
)
echo.

if "%MODE%"=="check" goto :check_only
if "%MODE%"=="install" goto :install
if not "%SETUP_RESULT%"=="0" goto :install
echo Everything needed is installed -- not running pip.
goto :profile

rem --- Install the PC packages ----------------------------------------------
:install
%PY% -m pip --version >nul 2>nul
if errorlevel 1 (
    echo pip is not available -- setting it up with ensurepip...
    %PY% -m ensurepip --upgrade
    if errorlevel 1 (
        echo ERROR: Could not set up pip. Reinstall Python from https://www.python.org/downloads/
        goto :fail
    )
)
echo Installing the PC packages from requirements_pc.txt...
echo ^(details are also written to pip_install.log^)
%PY% -m pip install --disable-pip-version-check --log "%~dp0pip_install.log" -r requirements_pc.txt
if errorlevel 1 (
    echo.
    echo ERROR: Installing the packages failed -- see the message above and
    echo   %~dp0pip_install.log
    echo Common causes: no internet connection, or a proxy/firewall blocking
    echo pypi.org. Fix that and run start_server.bat again.
    goto :fail
)
echo.
echo Checking again after the install:
%PY% check_pc_setup.py --installed requirements_pc.txt
if errorlevel 1 (
    echo.
    echo ERROR: Some packages are still missing or too old after the install.
    echo Send the report above and pip_install.log to whoever maintains this PC.
    goto :fail
)
echo.

rem --- Motor profile --------------------------------------------------------
:profile
if not exist "motor_profiles.json" (
    call :profile_help
    goto :fail
)

rem --- Start ------------------------------------------------------------------
set "WEB_PORT=%SERVO_WEB_PORT%"
if not defined WEB_PORT set "WEB_PORT=5000"
echo Make sure the RS-485 USB adapter is plugged in before continuing.
echo The server will auto-detect its COM port.
echo.
echo Opening http://localhost:%WEB_PORT% in your browser in a few seconds...
start "" cmd /c "timeout /t 3 >nul & start http://localhost:%WEB_PORT%"
echo Starting the server. Press Ctrl+C in this window to stop it.
echo ============================================
echo.
%PY% app.py
set "APP_RESULT=%errorlevel%"
echo.
if "%APP_RESULT%"=="0" (
    echo Server stopped.
) else (
    echo The server stopped with an error ^(exit code %APP_RESULT%^) -- scroll up for the message.
    echo Run "start_server.bat --check" to see the setup report again.
)
pause
exit /b %APP_RESULT%

rem --- Helpers ---------------------------------------------------------------
:check_only
set "CHECK_RESULT=%SETUP_RESULT%"
if not exist "motor_profiles.json" (
    call :profile_help
    set "CHECK_RESULT=1"
)
if not "%SETUP_RESULT%"=="0" (
    echo Packages are missing or too old -- run start_server.bat to install them.
)
if "%CHECK_RESULT%"=="0" (
    echo Setup check passed. Run start_server.bat without --check to start.
) else (
    echo Setup check FAILED -- fix the items above before starting the server.
)
pause
exit /b %CHECK_RESULT%

:profile_help
echo ERROR: motor_profiles.json is missing from
echo   %CD%
echo It holds this rig's motor settings (gear ratio, baud rate, home) and is
echo not stored in git, so a fresh clone or pull does not have it. Copy it from
echo this rig's previous installation, or restore the last committed version
echo with Git ^(from the repository root^) and check its values:
echo   git show a64941d~1:servo_comm_shihlin_unified/motor_profiles.json ^> servo_comm_shihlin_unified\motor_profiles.json
echo.
exit /b 0

:usage
echo Usage: start_server.bat [--check ^| --install]
echo   (no option)  check this PC, install missing packages, start the server
echo   --check      only show the setup report; installs and starts nothing
echo   --install    reinstall the packages even if the check passes, then start
echo Optional: set SERVO_WEB_PORT=5001 or SERVO_SERIAL_PORT=COM7 before running.
pause
if defined USAGE_EXIT exit /b %USAGE_EXIT%
exit /b 0

:fail
echo.
pause
exit /b 1
