@echo off
rem Double-click: opens the lightmap optimizer window (pythonw when installed, so no console stays open).
setlocal
set GUI=%~dp0lmoptimizer_gui.py
where pythonw >nul 2>nul
if not errorlevel 1 (
    start "" pythonw "%GUI%"
) else (
    python "%GUI%"
)
