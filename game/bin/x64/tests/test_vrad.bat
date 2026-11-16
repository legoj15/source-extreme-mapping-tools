@echo off
rem Double-click: lists the vrad comparison tests and asks which to run.
rem With arguments: passes them to run_compare_tests.ps1 (-All, -TestNames a,b, -Group g).
setlocal
set RUNNER=%~dp0compare\run_compare_tests.ps1
if not "%~1"=="" (
    pwsh.exe -ExecutionPolicy Bypass -File "%RUNNER%" -Tool vrad %*
    exit /b %ERRORLEVEL%
)
pwsh.exe -ExecutionPolicy Bypass -File "%RUNNER%" -Tool vrad
echo.
set /p NAMES=Tests to run (comma separated, or "all"): 
if /i "%NAMES%"=="all" (
    pwsh.exe -ExecutionPolicy Bypass -File "%RUNNER%" -Tool vrad -All
) else (
    pwsh.exe -ExecutionPolicy Bypass -File "%RUNNER%" -Tool vrad -TestNames %NAMES%
)
pause
