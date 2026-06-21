@echo off
setlocal EnableExtensions
chcp 65001 >nul
cd /d "%~dp0"

set "PYEXE="
where py >nul 2>nul
if not errorlevel 1 set "PYEXE=py -3"
if not defined PYEXE (
  where python >nul 2>nul
  if not errorlevel 1 set "PYEXE=python"
)
if not defined PYEXE (
  echo [ERROR] Python was not found. Please install Python 3 or add it to PATH.
  pause
  exit /b 1
)

echo [INFO] Installing dependencies...
%PYEXE% -m pip install --upgrade pip
%PYEXE% -m pip install numpy scipy matplotlib tifffile
if errorlevel 1 (
  echo [ERROR] Dependency installation failed.
  pause
  exit /b 1
)
echo [OK] Dependencies installed. Optional: %PYEXE% -m pip install rasterio
pause
