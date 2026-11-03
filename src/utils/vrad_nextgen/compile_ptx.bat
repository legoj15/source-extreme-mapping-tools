@echo off
REM Compile OptiX .cu kernels to embedded PTX headers
REM Output: optix_kernels_ptx.h (C string containing the PTX)

setlocal

REM Setup Visual Studio environment for cl.exe
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul 2>&1

set CUDA_PATH=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v13.2
set OPTIX_PATH=C:\ProgramData\NVIDIA Corporation\OptiX SDK 9.1.0

set NVCC="%CUDA_PATH%\bin\nvcc.exe"
set SRC_DIR=%~dp0
set CU_FILE=%SRC_DIR%optix_kernels.cu
set PTX_FILE=%SRC_DIR%optix_kernels.ptx
set PTX_HEADER=%SRC_DIR%optix_kernels_ptx.h

echo [PTX] Compiling %CU_FILE%...

%NVCC% -ptx ^
    -I"%OPTIX_PATH%\include" ^
    -I"%CUDA_PATH%\include" ^
    --use_fast_math ^
    --allow-unsupported-compiler ^
    -arch=sm_86 ^
    -o "%PTX_FILE%" ^
    "%CU_FILE%"

if %ERRORLEVEL% neq 0 (
    echo [PTX] ERROR: nvcc compilation failed!
    exit /b 1
)

echo [PTX] Generating embedded header %PTX_HEADER%...

REM Convert PTX to a C-string embedded header using PowerShell
REM (more reliable than batch for/f with special characters)
powershell -NoProfile -Command ^
    "$ptx = Get-Content '%PTX_FILE%' -Raw; " ^
    "$escaped = $ptx -replace '\\', '\\\\' -replace '\"', '\\\"'; " ^
    "$lines = $escaped -split \"`n\"; " ^
    "$out = \"// Auto-generated - do not edit`r`n\"; " ^
    "$out += \"#pragma once`r`n`r`n\"; " ^
    "$out += \"static const char OPTIX_KERNELS_PTX[] =`r`n\"; " ^
    "foreach ($line in $lines) { " ^
    "  $line = $line.TrimEnd(\"`r\"); " ^
    "  if ($line.Length -gt 0) { $out += '\"' + $line + '\n\"' + \"`r`n\" } " ^
    "}; " ^
    "$out += \";`r`n\"; " ^
    "Set-Content '%PTX_HEADER%' $out -NoNewline"

if %ERRORLEVEL% neq 0 (
    echo [PTX] ERROR: Header generation failed!
    exit /b 1
)

echo [PTX] Done.
del "%PTX_FILE%" 2>nul
