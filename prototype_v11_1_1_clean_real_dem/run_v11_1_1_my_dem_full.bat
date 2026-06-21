@echo off
chcp 65001 >nul
python run_v11_experiments.py --dem-path-file my_dem_path.txt --out outputs_v11_1_1_real_full --n-rois 4 --n-pairs-per-scenario 1 --neighbors 8 --hard-limits 1.0,1.2,1.5
pause
