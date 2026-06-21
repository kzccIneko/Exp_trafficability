"""
下载 ERA5-Land 土壤湿度数据
使用方法：
1. 设置环境变量 CDS_URL 和 CDS_KEY
2. 或直接修改下方配置
"""

import cdsapi
import os

# 配置 CDS API - 请填入你的 API key
CDS_URL = 'https://cds.climate.copernicus.eu/api'
CDS_KEY = 'your_cds_api_key_here'  # 替换为你的 API key

c = cdsapi.Client(url=CDS_URL, key=CDS_KEY)

print("="*60)
print("下载 ERA5-Land 土壤湿度数据")
print("="*60)
print("\n区域: 雅江格西沟")
print("范围: 100.846°E ~ 101.124°E, 29.876°N ~ 30.154°N")
print("时间: 2023年全年月平均")
print("变量: 4层土壤湿度")

try:
    c.retrieve(
        'reanalysis-era5-land-monthly-means',
        {
            'product_type': 'monthly_averaged_reanalysis',
            'variable': [
                'volumetric_soil_water_layer_1',  # 0-7cm
                'volumetric_soil_water_layer_2',  # 7-28cm
                'volumetric_soil_water_layer_3',  # 28-100cm
                'volumetric_soil_water_layer_4',  # 100-289cm
            ],
            'year': '2023',
            'month': ['01', '02', '03', '04', '05', '06',
                      '07', '08', '09', '10', '11', '12'],
            'time': '00:00',
            'area': [30.154, 100.846, 29.876, 101.124],  # North, West, South, East
            'data_format': 'netcdf',
        },
        'era5_land_soil_moisture_2023.nc')

    print("\n[OK] 下载完成!")
    print("文件: era5_land_soil_moisture_2023.nc")

except Exception as e:
    print(f"\n[ERROR] 下载失败: {e}")
    print("\n可能的原因:")
    print("1. API key 无效或过期")
    print("2. 网络连接问题")
    print("3. CDS 服务器繁忙")
