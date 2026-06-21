@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"
call :findpy
%PYEXE% run_v7_vehicle_experiments.py --dem-path-file my_dem_path.txt --cell-size 30 --max-pixels 1000 --out outputs_v7_real_full --roi-top-k 4 --max-pairs 12 --sensitivity-pairs 2
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
