@echo off
setlocal enabledelayedexpansion

rem Ensure basic system commands and PowerShell are in PATH
set "PATH=%SystemRoot%\System32;%SystemRoot%;%SystemRoot%\System32\Wbem;%SystemRoot%\System32\WindowsPowerShell\v1.0;%ProgramFiles%\PowerShell\7;%ProgramFiles%\PowerShell\6;%LOCALAPPDATA%\Microsoft\WindowsApps;!PATH!"

if "%CUDA_PATH%"=="" (
    if exist "C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v13.2" (
        set "CUDA_PATH=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v13.2"
        echo Info: CUDA_PATH not set, using default: !CUDA_PATH!
    ) else (
        echo Error: CUDA_PATH environment variable is not set and default v13.2 path not found.
        exit /b 1
    )
)

set "CUDA_BIN_PATH=%CUDA_PATH%\bin"

rem Use vswhere to find the latest Visual Studio installation
set "VSWHERE_DIR=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer"
set "VS_INSTALL_DIR="

if exist "%VSWHERE_DIR%\vswhere.exe" (
    pushd "%VSWHERE_DIR%"
    for /f "usebackq tokens=*" %%i in (`vswhere.exe -latest -version "[17.0,30.0)" -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do (
        set "VS_INSTALL_DIR=%%i"
    )
    popd
)


if "%VS_INSTALL_DIR%"=="" (
    echo Error: Visual Studio 2022/2026 with VC++ Tools not found.
    exit /b 1
)

echo Found Visual Studio at: %VS_INSTALL_DIR%

set "VCVARS_PATH=%VS_INSTALL_DIR%\VC\Auxiliary\Build\vcvars64.bat"

if not exist "%VCVARS_PATH%" (
    echo Error: vcvars64.bat not found at %VCVARS_PATH%
    exit /b 1
)

call "%VCVARS_PATH%"

rem Use cl.exe from path (set by vcvars)
set "CL_PATH="
for %%X in (cl.exe) do (set "CL_PATH=%%~$PATH:X")

if "!CL_PATH!"=="" (
    echo Error: cl.exe not found in PATH after vcvars64.bat
    exit /b 1
)

echo Found CL at: !CL_PATH!

echo DEBUG: Running NVCC...
set "CONFIG_CLEAN=%CONFIG: =%"
if /I "!CONFIG_CLEAN!"=="Debug" (
    echo [NVCC] Debug build detected.
    set "NVCC_FLAGS=-g -G -D_DEBUG -O0 -Xcompiler /MTd"
) else (
    echo [NVCC] Release build detected.
    set "NVCC_FLAGS=-O3 -Xcompiler /MT"
)

:: Suppress systemic warnings
set "NVCC_FLAGS=!NVCC_FLAGS! -diag-suppress 174,815,1394,997 -Xcompiler /wd4267"

rem Removed -I"..\thirdparty\optix\include" and changed -ptx to -c for object compilation
rem Quote all flags and capture all arguments robustly
"%CUDA_BIN_PATH%\nvcc.exe" -ccbin "!CL_PATH!" --use-local-env -m64 -D_WIN64 -D_WIN32 -v -c !NVCC_FLAGS! -arch=sm_75 -Wno-deprecated-gpu-targets -allow-unsupported-compiler --use_fast_math %*

if %ERRORLEVEL% neq 0 (
    echo Error: NVCC failed with error level %ERRORLEVEL%
    exit /b %ERRORLEVEL%
)

endlocal

