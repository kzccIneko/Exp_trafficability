@echo off
chcp 65001 >nul
python run_v11_experiments.py --synthetic --out outputs_v11_1_1_synthetic_test --n-rois 1 --max-pairs 2 --neighbors 8 --hard-limits 1.0,1.2
pause
