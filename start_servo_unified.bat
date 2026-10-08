@echo off
rem Double-click launcher for the Shihlin servo web server
rem (servo_comm_shihlin_unified). Lives at the repository root so it can be
rem found without opening the folder. Everything -- the setup report,
rem installing missing PC packages, starting the server -- is done by
rem servo_comm_shihlin_unified\start_server.bat; options are passed through:
rem   start_servo_unified.bat --check    only show the setup report
rem   start_servo_unified.bat --install  reinstall the packages, then start
setlocal
set "SERVER_BAT=%~dp0servo_comm_shihlin_unified\start_server.bat"

if not exist "%SERVER_BAT%" (
    echo ERROR: "%SERVER_BAT%" was not found.
    echo Keep this file in the repository root, next to the
    echo servo_comm_shihlin_unified folder.
    pause
    exit /b 1
)

call "%SERVER_BAT%" %*
exit /b %errorlevel%
