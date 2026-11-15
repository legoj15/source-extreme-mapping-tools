@echo off
rem Double-click: lists the vvis comparison tests and asks which to run.
rem With arguments: passes them to run_vvis_tests.ps1 (-All, -TestNames a,b, -Group g).
setlocal
set RUNNER=%~dp0compare\run_vvis_tests.ps1
if not "%~1"=="" (
    pwsh.exe -ExecutionPolicy Bypass -File "%RUNNER%" %*
    exit /b %ERRORLEVEL%
)
pwsh.exe -ExecutionPolicy Bypass -File "%RUNNER%"
echo.
set /p NAMES=Tests to run (comma separated, or "all"): 
if /i "%NAMES%"=="all" (
    pwsh.exe -ExecutionPolicy Bypass -File "%RUNNER%" -All
) else (
    pwsh.exe -ExecutionPolicy Bypass -File "%RUNNER%" -TestNames %NAMES%
)
pause
