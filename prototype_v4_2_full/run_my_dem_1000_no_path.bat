@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo =========================================================
echo 1000x1000 DEM 代价场快速实验（不做 A* 路径规划）
echo 当前 Python:
python --version
echo =========================================================
python run_experiment.py --dem "D:\VSCode Program\通行能力分析_研\yajiang_gesigou_srtmgl1_30m_1000px_bbox.tif" --max-pixels 1000 --cell-size 30 --no-path --out outputs_real_1000_cost_only
echo.
echo 运行结束或已报错。成功时输出目录为: outputs_real_1000_cost_only
pause
