@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo =========================================================
echo 安装实验代码依赖包
echo 当前 Python:
python --version
echo =========================================================
echo.
echo 1/2 安装基础依赖：numpy scipy matplotlib tifffile
python -m pip install --upgrade pip
python -m pip install numpy scipy matplotlib tifffile
if errorlevel 1 (
  echo.
  echo 基础依赖安装失败。请检查网络，或把错误截图发给我。
  pause
  exit /b 1
)
echo.
echo 2/2 尝试安装 rasterio（可选，但建议安装，能读取 GeoTIFF 坐标元数据）
python -m pip install rasterio
if errorlevel 1 (
  echo.
  echo rasterio 安装失败也可以先继续运行，v4.2 会自动退回 tifffile 读取 DEM。
  echo 如果之后仍有问题，请把这段错误截图发给我。
)
echo.
echo 依赖安装步骤结束。现在可以运行 run_my_dem_1000.bat。
pause
