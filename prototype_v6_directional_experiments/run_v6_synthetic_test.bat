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

echo [INFO] Running v6.1 synthetic DEM test...
%PYEXE% run_v6_experiments.py --synthetic --synthetic-size 180 --out outputs_v6_synthetic_test --cell-size 30 --roi-top-k 2 --roi-window 100 --roi-stride 60 --random-pairs-per-roi 1 --neighbors 4,8,16 --sensitivity-pairs 3
if errorlevel 1 (
  echo [ERROR] Synthetic DEM test failed. See messages above.
  pause
  exit /b 1
)
echo [OK] Finished. Please open outputs_v6_synthetic_test\00_实验说明_请先看.md
pause
