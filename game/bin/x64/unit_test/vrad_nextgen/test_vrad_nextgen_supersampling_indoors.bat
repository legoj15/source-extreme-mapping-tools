@echo off
pwsh.exe -ExecutionPolicy Bypass -File .\run_vrad_nextgen_tests.ps1 -TestNames nextgen_supersampling_indoors %*
