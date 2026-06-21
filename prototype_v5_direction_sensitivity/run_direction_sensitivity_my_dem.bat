@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo 正在运行 DEM 方向敏感扩展实验 v5...
echo 如果第一次运行报缺库，请先运行 install_dependencies.bat
python run_sensitivity_suite.py --dem "D:\VSCode Program\通行能力分析_研\yajiang_gesigou_srtmgl1_30m_1000px_bbox.tif" --max-pixels 1000 --cell-size 30 --out outputs_direction_sensitivity
pause
