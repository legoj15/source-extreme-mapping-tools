@echo off
REM bspwalk: compiles bw_arcs.cu to game\bin\x64\bspwalk_arcs.ptx and bw_fan.cu to
REM game\bin\x64\bspwalk_fan.ptx (OptiX, loaded at run time from bspwalk.exe's own directory).
REM Run by the bspwalk PreBuildEvent from this directory.

setlocal

REM CUDA: the first argument (bspwalk.vpc passes $(CUDA_PATH), the environment's value, so the
REM PTX and the host build use one toolkit), else CUDA_PATH (set by the CUDA installer), else
REM v13.4.
if "%~1"=="" if "%CUDA_PATH%"=="" (
    echo WARNING: CUDA_PATH is not set in the build environment. The project's CUDA include and
    echo          library paths come from CUDA_PATH, so the host build will not find the toolkit
    echo          this script falls back to. Set CUDA_PATH machine-wide ^(the CUDA installer does^).
)
if not "%~1"=="" set "CUDA_PATH=%~1"
if "%CUDA_PATH%"=="" set "CUDA_PATH=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v13.4"
if "%CUDA_PATH:~-1%"=="\" set "CUDA_PATH=%CUDA_PATH:~0,-1%"
if not exist "%CUDA_PATH%\bin\nvcc.exe" (
    echo ERROR: nvcc not found at %CUDA_PATH%. Install CUDA 13.2 or newer ^(13.4 is the default^) or set CUDA_PATH.
    exit /b 1
)
REM Floor: CUDA 13.2 or newer. v13.1's cudafe++ crashes under MSVC 14.51, and older toolkits
REM (13.0, 12.x) are not supported either. The version is read from nvcc itself, so it does not
REM depend on how CUDA_PATH is spelled.
REM (No pipe in the for /f command: cmd /c strips the quotes of a command with more than two, and
REM the quoted nvcc path then breaks at "C:\Program Files".)
set "NVCC_LINE="
for /f "delims=" %%v in ('"%CUDA_PATH%\bin\nvcc.exe" --version') do (
    echo %%v| findstr /C:"release" >nul && set "NVCC_LINE=%%v"
)
call :require_cuda_13_2 "%NVCC_LINE%" || exit /b 1

REM OptiX: the second argument (the .vpc's OPTIX_PATH macro, so the PTX and the host
REM build use one SDK), else the default install location for direct runs.
set "OPTIX_PATH=%~2"
if "%OPTIX_PATH%"=="" set "OPTIX_PATH=C:\ProgramData\NVIDIA Corporation\OptiX SDK 9.1.0"
if not exist "%OPTIX_PATH%\include\optix.h" (
    echo ERROR: OptiX SDK not found at %OPTIX_PATH%. Install OptiX SDK 9.1.0 or fix OPTIX_PATH in the .vpc.
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

REM :require_cuda_13_2 "<text>": <text> is nvcc's version line (Cuda compilation tools, release
REM 13.4, V13.4.92) or just "release 13.4". Exits 0 when major.minor is 13.2 or newer (13.4 and
REM 14.x pass), 1 with a message when it is older or cannot be read.
:require_cuda_13_2
set "CUDA_LINE=%~1"
set "CUDA_MAJOR="
set "CUDA_MINOR="
REM everything after the first "release ", then the numbers before its dots and commas
set "CUDA_TAIL=%CUDA_LINE:*release =%"
for /f "tokens=1,2 delims=.," %%a in ("%CUDA_TAIL%") do (
    set "CUDA_MAJOR=%%a"
    set "CUDA_MINOR=%%b"
)
echo %CUDA_MAJOR%.%CUDA_MINOR%| findstr /R /X "[0-9][0-9]*\.[0-9][0-9]*" >nul || goto :cuda_unreadable
if %CUDA_MAJOR% LSS 13 goto :cuda_too_old
if %CUDA_MAJOR% EQU 13 if %CUDA_MINOR% LSS 2 goto :cuda_too_old
exit /b 0
:cuda_too_old
echo ERROR: CUDA %CUDA_MAJOR%.%CUDA_MINOR% is below 13.2 (13.1 crashes under MSVC 14.51, older ones are unsupported). Point CUDA_PATH at 13.2 or newer.
exit /b 1
:cuda_unreadable
echo ERROR: cannot read the CUDA version from "%CUDA_PATH%\bin\nvcc.exe --version" (got "%CUDA_LINE%"). Point CUDA_PATH at 13.2 or newer.
exit /b 1
