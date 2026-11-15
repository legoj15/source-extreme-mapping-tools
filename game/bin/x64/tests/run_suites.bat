@echo off
rem Double-click: runs the quick regression suites (bash .agents/scripts/run_suites.sh) from the repo root.
rem With arguments: passes them on (--only a,b  or  --list).
setlocal
pushd "%~dp0..\..\..\.."
bash .agents/scripts/run_suites.sh %*
set RC=%ERRORLEVEL%
popd
pause
exit /b %RC%
