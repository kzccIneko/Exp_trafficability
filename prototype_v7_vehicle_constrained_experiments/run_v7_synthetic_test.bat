@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"
call :findpy
%PYEXE% run_v7_vehicle_experiments.py --synthetic --synthetic-size 220 --cell-size 30 --out outputs_v7_synthetic_test --roi-top-k 2 --max-pairs 5 --sensitivity-pairs 2
pause
exit /b
:findpy
where py >nul 2>nul
if %errorlevel%==0 (
  set PYEXE=py -3
) else (
  set PYEXE=python
)
exit /b
