@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"
call :find_python
if errorlevel 1 goto :end
%PYTHON_CMD% run_v10_experiments.py --dem-path-file my_dem_path.txt --max-pixels 600 --cell-size 30 --out outputs_v11_1_1_real_quick --n-rois 2 --n-pairs-per-scenario 1 --ablation-pairs 2 --scenario-pairs 2
:end
pause
exit /b
:find_python
py -3 --version >nul 2>nul
if not errorlevel 1 (
  set PYTHON_CMD=py -3
  exit /b 0
)
python --version >nul 2>nul
if not errorlevel 1 (
  set PYTHON_CMD=python
  exit /b 0
)
echo 未找到 Python，请先安装 Python。
exit /b 1
