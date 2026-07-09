@echo off
pwsh.exe -ExecutionPolicy Bypass -File .\run_vvis_tests.ps1 -TestNames basic %*
