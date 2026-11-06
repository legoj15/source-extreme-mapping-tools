@echo off
REM bspwalk: compiles bw_arcs.cu to game\bin\x64\bspwalk_arcs.ptx and bw_fan.cu to
REM game\bin\x64\bspwalk_fan.ptx (OptiX, loaded at run time from bspwalk.exe's own directory).
REM Run by the bspwalk PreBuildEvent from this directory.

setlocal

REM CUDA: the first argument (bspwalk.vpc passes its pinned CUDA_DIR, so the PTX and the
REM host build use one toolkit), else CUDA_PATH (set by the CUDA installer), else v13.4.
if not "%~1"=="" set "CUDA_PATH=%~1"
REM v13.1's cudafe++ crashes under MSVC 14.51, so refuse it.
if "%CUDA_PATH%"=="" set "CUDA_PATH=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v13.4"
if "%CUDA_PATH:~-1%"=="\" set "CUDA_PATH=%CUDA_PATH:~0,-1%"
if /I "%CUDA_PATH:~-5%"=="v13.1" (
    echo ERROR: CUDA 13.1 crashes under MSVC 14.51. Point CUDA_PATH at 13.2 or newer.
    exit /b 1
)
if not exist "%CUDA_PATH%\bin\nvcc.exe" (
    echo ERROR: nvcc not found at %CUDA_PATH%. Install CUDA 13.4 or set CUDA_PATH.
    exit /b 1
)

set "OPTIX_PATH=C:\ProgramData\NVIDIA Corporation\OptiX SDK 9.1.0"
if not exist "%OPTIX_PATH%\include\optix.h" (
    echo ERROR: OptiX SDK 9.1.0 not found at %OPTIX_PATH%
    exit /b 1
)

set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
    echo ERROR: vswhere.exe not found
    exit /b 1
)
for /f "delims=" %%i in ('"%VSWHERE%" -latest -property installationPath 2^>nul') do set "VS_PATH=%%i"
if "%VS_PATH%"=="" (
    echo ERROR: Visual Studio not found
    exit /b 1
)
for /f "delims=" %%i in ('dir "%VS_PATH%\VC\Tools\MSVC" /b /ad /o-n') do (
    set "MSVC_VER=%%i"
    goto :found_msvc
)
:found_msvc
set "MSVC_PATH=%VS_PATH%\VC\Tools\MSVC\%MSVC_VER%"

echo bspwalk: compiling bw_arcs.cu and bw_fan.cu to PTX (CUDA %CUDA_PATH%, MSVC %MSVC_VER%)
if not exist "..\..\..\game\bin\x64" mkdir "..\..\..\game\bin\x64"

call :ptx bw_arcs.cu bspwalk_arcs.ptx nvcc_arcs || exit /b 1
call :ptx bw_fan.cu bspwalk_fan.ptx nvcc_fan || exit /b 1
exit /b 0

REM :ptx compiles one kernel. Arguments: the source .cu, the output .ptx, and
REM the prefix of its two nvcc log files.
REM No --use_fast_math: the kernels' float arithmetic should stay close to the
REM CPU reference that -arccheck and -vischeck compare them with.
:ptx
"%CUDA_PATH%\bin\nvcc.exe" ^
    -allow-unsupported-compiler ^
    -O3 ^
    -ptx ^
    -arch=sm_75 ^
    --compiler-bindir "%MSVC_PATH%\bin\Hostx64\x64" ^
    -I"%CUDA_PATH%\include" ^
    -I"%OPTIX_PATH%\include" ^
    -I"." ^
    %1 ^
    -o "..\..\..\game\bin\x64\%2" ^
    > %3_stdout.txt 2> %3_stderr.txt

if ERRORLEVEL 1 (
    echo *** bspwalk PTX compilation of %1 FAILED ***
    type %3_stderr.txt
    exit /b 1
)
echo bspwalk: wrote ..\..\..\game\bin\x64\%2
exit /b 0
