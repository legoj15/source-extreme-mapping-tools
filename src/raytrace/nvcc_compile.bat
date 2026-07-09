@echo off
REM CUDA Compilation Script for VRAD RTX
REM Compiles raytrace_cuda.cu to object file

setlocal

REM Find CUDA installation
set CUDA_PATH=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v13.2
if not exist "%CUDA_PATH%\bin\nvcc.exe" (
    echo ERROR: CUDA 13.2 not found at %CUDA_PATH%
    echo Please install CUDA Toolkit 13.2 or update CUDA_PATH in this script.
    exit /b 1
)

REM Find Visual Studio using vswhere
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
    echo ERROR: vswhere.exe not found. Please install Visual Studio 2019 or later.
    exit /b 1
)

for /f "delims=" %%i in ('"%VSWHERE%" -latest -property installationPath 2^>nul') do set VS_PATH=%%i

if "%VS_PATH%"=="" (
    echo ERROR: Visual Studio not found
    exit /b 1
)


REM Find MSVC version
for /f "delims=" %%i in ('dir "%VS_PATH%\VC\Tools\MSVC" /b /ad /o-n') do (
    set MSVC_VER=%%i
    goto :found_msvc
)
:found_msvc

set MSVC_PATH=%VS_PATH%\VC\Tools\MSVC\%MSVC_VER%

echo.
echo ============================================================
echo Compiling CUDA Kernel: raytrace_cuda.cu
echo ============================================================
echo CUDA: %CUDA_PATH%
echo MSVC: %MSVC_VER%
echo.

REM Create output directory if needed
if not exist "Release\x64" mkdir Release\x64

REM Compile CUDA code
"%CUDA_PATH%\bin\nvcc.exe" ^
    -allow-unsupported-compiler ^
    -O3 ^
    -arch=sm_75 ^
    -gencode arch=compute_75,code=sm_75 ^
    -gencode arch=compute_86,code=sm_86 ^
    -gencode arch=compute_89,code=sm_89 ^
    -gencode arch=compute_90,code=sm_90 ^
    --compiler-bindir "%MSVC_PATH%\bin\Hostx64\x64" ^
    -I"%CUDA_PATH%\include" ^
    -I"..\..\public" ^
    -I"..\..\public\tier0" ^
    -I"..\..\public\tier1" ^
    -I"..\..\public\mathlib" ^
    -I"." ^
    -c raytrace_cuda.cu ^
    -o Release\x64\raytrace_cuda.obj ^
    > nvcc_stdout.txt 2> nvcc_stderr.txt

if ERRORLEVEL 1 (
    echo.
    echo *** CUDA COMPILATION FAILED ***
    echo.
    type nvcc_stderr.txt
    exit /b 1
)

echo.
echo CUDA compilation successful: Release\x64\raytrace_cuda.obj
echo.

exit /b 0
