@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo =========================================================
echo 1000x1000 DEM 方向敏感越野通行能力实验
echo 当前目录: %cd%
echo 当前 Python:
python --version
echo =========================================================
echo.
echo 如果提示缺少 numpy/scipy/matplotlib/tifffile/rasterio，请先运行 install_dependencies.bat
echo.
python run_experiment.py --dem "D:\VSCode Program\通行能力分析_研\yajiang_gesigou_srtmgl1_30m_1000px_bbox.tif" --max-pixels 1000 --cell-size 30 --out outputs_real_1000
echo.
echo 运行结束或已报错。请查看上面的提示。
echo 成功时输出目录为: outputs_real_1000
pause
