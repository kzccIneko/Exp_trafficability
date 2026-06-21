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

echo [INFO] Running v6.1 real DEM quick test...
if not exist "my_dem_path.txt" (
  echo [ERROR] my_dem_path.txt was not found.
  pause
  exit /b 1
)
%PYEXE% run_v6_experiments.py --dem-config my_dem_path.txt --max-pixels 500 --cell-size 30 --out outputs_v6_real_quick --roi-top-k 2 --roi-window 160 --roi-stride 80 --random-pairs-per-roi 0 --neighbors 4,8 --sensitivity-pairs 2
if errorlevel 1 (
  echo [ERROR] Real DEM quick test failed. Check DEM path in my_dem_path.txt and dependencies.
  pause
  exit /b 1
)
echo [OK] Finished. Please open outputs_v6_real_quick\00_实验说明_请先看.md
pause
