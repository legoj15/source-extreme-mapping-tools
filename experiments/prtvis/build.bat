@echo off
setlocal
rem Builds prtvis.exe (float, mirrors stock VVIS) and prtvis_d.exe (same source,
rem /DPRTVIS_DOUBLE: the clip math in double) with the newest MSVC that has the x64 tools.
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
  echo vswhere.exe not found at "%VSWHERE%"
  exit /b 1
)
set "VSROOT="
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSROOT=%%i"
if not defined VSROOT (
  echo vswhere found no Visual Studio with the x64 C++ tools
  exit /b 1
)
call "%VSROOT%\VC\Auxiliary\Build\vcvars64.bat" >nul 2>nul
if errorlevel 1 exit /b 1
cd /d "%~dp0"
cl /nologo /O2 /EHsc /std:c++17 /fp:precise .\prtvis.cpp /Fo:.\prtvis.obj /Fe:.\prtvis.exe
if errorlevel 1 exit /b 1
cl /nologo /O2 /EHsc /std:c++17 /fp:precise /DPRTVIS_DOUBLE .\prtvis.cpp /Fo:.\prtvis_d.obj /Fe:.\prtvis_d.exe
if errorlevel 1 exit /b 1
if exist .\prtvis.obj del .\prtvis.obj
if exist .\prtvis_d.obj del .\prtvis_d.obj
exit /b 0
