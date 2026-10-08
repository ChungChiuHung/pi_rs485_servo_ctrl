@echo off
rem Double-click launcher for the Shihlin servo web server
rem (servo_comm_shihlin_unified). Lives at the repository root so it can be
rem found without opening the folder; it only checks the motor profile file
rem and hands over to servo_comm_shihlin_unified\start_server.bat, which
rem installs the dependencies, opens the browser and runs the server.
setlocal
set "UNIFIED_DIR=%~dp0servo_comm_shihlin_unified"

if not exist "%UNIFIED_DIR%\start_server.bat" (
    echo ERROR: "%UNIFIED_DIR%\start_server.bat" was not found.
    echo Keep this file in the repository root, next to the
    echo servo_comm_shihlin_unified folder.
    pause
    exit /b 1
)

rem motor_profiles.json is tuned to the connected motor and is not in git
rem (see .gitignore), so a fresh clone or pull does not have it and app.py
rem stops with "FileNotFoundError: motor_profiles.json".
if not exist "%UNIFIED_DIR%\motor_profiles.json" (
    echo ERROR: motor_profiles.json is missing from
    echo   %UNIFIED_DIR%
    echo.
    echo It holds this rig's motor settings ^(gear ratio, baud rate, home^) and is
    echo not stored in git. Copy it from this rig's previous installation, or
    echo restore the last committed version with Git and check its values:
    echo   git show a64941d~1:servo_comm_shihlin_unified/motor_profiles.json ^> servo_comm_shihlin_unified\motor_profiles.json
    pause
    exit /b 1
)

call "%UNIFIED_DIR%\start_server.bat"
exit /b %errorlevel%
