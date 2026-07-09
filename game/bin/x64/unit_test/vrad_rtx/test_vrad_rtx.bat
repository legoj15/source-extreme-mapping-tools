@echo off
pwsh.exe -ExecutionPolicy Bypass -File .\run_vrad_tests.ps1 -TestNames full %*
