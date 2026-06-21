@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo 正在运行快速测试：max-pixels=500, n-pairs=8, neighbors=4,8 ...
python run_sensitivity_suite.py --dem "D:\VSCode Program\通行能力分析_研\yajiang_gesigou_srtmgl1_30m_1000px_bbox.tif" --max-pixels 500 --cell-size 30 --n-pairs 8 --neighbors 4,8 --out outputs_quick_test
pause
