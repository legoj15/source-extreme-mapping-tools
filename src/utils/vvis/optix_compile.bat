@echo off
REM OptiX PTX Compilation Script for VVIS CUDA
REM Compiles vvis_optix.cu to PTX file for runtime loading

setlocal

REM Find CUDA installation
set CUDA_PATH=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v13.2
if not exist "%CUDA_PATH%\bin\nvcc.exe" (
    echo ERROR: CUDA 13.2 not found at %CUDA_PATH%
    echo Please install CUDA Toolkit 13.2 or update CUDA_PATH in this script.
    exit /b 1
)

REM Find OptiX installation
set OPTIX_PATH=C:\ProgramData\NVIDIA Corporation\OptiX SDK 9.1.0
if not exist "%OPTIX_PATH%\include\optix.h" (
    echo ERROR: OptiX SDK 9.1.0 not found at %OPTIX_PATH%
    echo Please install OptiX SDK 9.1.0 or update OPTIX_PATH in this script.
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
echo Compiling OptiX PTX: vvis_optix.cu
echo ============================================================
echo CUDA: %CUDA_PATH%
echo OptiX: %OPTIX_PATH%
echo MSVC: %MSVC_VER%
echo.

REM Create output directory if needed - PTX goes to bin\x64 where vvis.exe runs
if not exist "..\..\..\game\bin\x64" mkdir "..\..\..\game\bin\x64"

REM Compile CUDA code to PTX for OptiX
"%CUDA_PATH%\bin\nvcc.exe" ^
    -allow-unsupported-compiler ^
    -O3 ^
    -ptx ^
    -arch=sm_75 ^
    -rdc=true ^
    -DCOMPILER_MSVC ^
    -DCOMPILER_MSVC64 ^
    --compiler-bindir "%MSVC_PATH%\bin\Hostx64\x64" ^
    -I"%CUDA_PATH%\include" ^
    -I"%OPTIX_PATH%\include" ^
    -I"." -I"..\common" -I"..\..\public" -I"..\..\public\tier0" -I"..\..\public\tier1" -I"..\..\utils\common" ^
    vvis_optix.cu ^
    -o "..\..\..\game\bin\x64\vvis_optix.ptx" ^
    > nvcc_optix_stdout.txt 2> nvcc_optix_stderr.txt

if ERRORLEVEL 1 (
    echo.
    echo *** OptiX PTX COMPILATION FAILED ***
    echo.
    type nvcc_optix_stderr.txt
    exit /b 1
)

echo.
echo OptiX PTX compilation successful: ..\..\..\game\bin\x64\vvis_optix.ptx
echo.

exit /b 0
