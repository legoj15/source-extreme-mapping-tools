@echo off
REM OptiX PTX Compilation Script for VRAD RTX
REM Compiles optix_kernels.cu to PTX file for runtime loading

setlocal

REM CUDA: the first argument (the .vpc passes $(CUDA_PATH), the toolkit the host build
REM uses), else CUDA_PATH, else v13.4. v13.1's cudafe++ crashes under MSVC 14.51, so refuse it.
if "%~1"=="" if "%CUDA_PATH%"=="" (
    echo WARNING: CUDA_PATH is not set in the build environment. The project's CUDA include and
    echo          library paths come from CUDA_PATH, so the host build will not find the toolkit
    echo          this script falls back to. Set CUDA_PATH machine-wide ^(the CUDA installer does^).
)
if not "%~1"=="" set "CUDA_PATH=%~1"
if "%CUDA_PATH%"=="" set "CUDA_PATH=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v13.4"
if "%CUDA_PATH:~-1%"=="\" set "CUDA_PATH=%CUDA_PATH:~0,-1%"
if /I "%CUDA_PATH:~-5%"=="v13.1" (
    echo ERROR: CUDA 13.1 crashes under MSVC 14.51. Point CUDA_PATH at 13.2 or newer.
    exit /b 1
)
if not exist "%CUDA_PATH%\bin\nvcc.exe" (
    echo ERROR: nvcc not found at %CUDA_PATH%
    echo Install CUDA Toolkit 13.4 or set CUDA_PATH.
    exit /b 1
)

REM OptiX: the second argument (the .vpc's OPTIX_PATH macro, so the PTX and the host
REM build use one SDK), else the default install location for direct runs.
set "OPTIX_PATH=%~2"
if "%OPTIX_PATH%"=="" set "OPTIX_PATH=C:\ProgramData\NVIDIA Corporation\OptiX SDK 9.1.0"
if not exist "%OPTIX_PATH%\include\optix.h" (
    echo ERROR: OptiX SDK not found at %OPTIX_PATH%. Install OptiX SDK 9.1.0 or fix OPTIX_PATH in the .vpc.
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
echo Compiling OptiX PTX: optix_kernels.cu
echo ============================================================
echo CUDA: %CUDA_PATH%
echo OptiX: %OPTIX_PATH%
echo MSVC: %MSVC_VER%
echo.

REM Create output directory if needed - PTX goes to bin\x64 where vrad.exe runs
if not exist "..\..\game\bin\x64" mkdir "..\..\game\bin\x64"

REM Compile CUDA code to PTX for OptiX
REM OptiX requires PTX, not object files
"%CUDA_PATH%\bin\nvcc.exe" ^
    -allow-unsupported-compiler ^
    -O3 ^
    -ptx ^
    -arch=sm_75 ^
    -rdc=true ^
    --compiler-bindir "%MSVC_PATH%\bin\Hostx64\x64" ^
    -I"%CUDA_PATH%\include" ^
    -I"%OPTIX_PATH%\include" ^
    -I"." ^
    optix_kernels.cu ^
    -o "..\..\game\bin\x64\optix_kernels.ptx" ^
    > nvcc_optix_stdout.txt 2> nvcc_optix_stderr.txt

if ERRORLEVEL 1 (
    echo.
    echo *** OptiX PTX COMPILATION FAILED ***
    echo.
    type nvcc_optix_stderr.txt
    exit /b 1
)

echo.
echo OptiX PTX compilation successful: ..\..\game\bin\x64\optix_kernels.ptx
echo.

exit /b 0
