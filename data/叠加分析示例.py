"""
雅江格西沟通行能力分析 - 数据叠加分析示例
展示如何将土壤湿度、道路、水系等数据叠加到DEM上进行分析
"""

import rasterio
import geopandas as gpd
import numpy as np
from scipy.interpolate import griddata
from pathlib import Path
import matplotlib.pyplot as plt

# 数据目录
DATA_DIR = Path(".")

def load_dem():
    """加载12.5m DEM数据"""
    print("="*60)
    print("1. 加载DEM数据")
    print("="*60)

    dem_path = DATA_DIR / "dem_12m_resampled.tif"

    with rasterio.open(dem_path) as src:
        dem = src.read(1).astype(np.float32)
        transform = src.transform
        crs = src.crs
        bounds = src.bounds
        nodata = src.nodata

        # 创建坐标网格
        rows, cols = dem.shape
        col_coords = np.arange(cols) * transform.a + transform.c
        row_coords = np.arange(rows) * transform.e + transform.f
        lon_grid, lat_grid = np.meshgrid(col_coords, row_coords)

        print(f"  尺寸: {src.width} x {src.height}")
        print(f"  分辨率: {src.res}")
        print(f"  坐标系: {crs}")
        print(f"  边界: {bounds}")
        print(f"  高程范围: {np.nanmin(dem):.2f}m ~ {np.nanmax(dem):.2f}m")

    return dem, transform, crs, bounds, lon_grid, lat_grid

def load_soil_moisture():
    """加载土壤湿度数据"""
    print("\n" + "="*60)
    print("2. 加载土壤湿度数据")
    print("="*60)

    import xarray as xr

    sm_path = DATA_DIR / "soil_moisture_2023.nc"

    ds = xr.open_dataset(sm_path)

    # 获取表层土壤湿度（全年平均）
    sm_layer1 = ds['swvl1'].mean(dim='valid_time')  # 第一层，时间平均

    print(f"  变量: swvl1 (表层土壤湿度)")
    print(f"  单位: m³/m³")
    print(f"  空间范围:")
    print(f"    经度: {ds.longitude.values.min():.4f} ~ {ds.longitude.values.max():.4f}")
    print(f"    纬度: {ds.latitude.values.min():.4f} ~ {ds.latitude.values.max():.4f}")
    print(f"  时间平均: 2023年全年")
    print(f"  平均土壤湿度: {sm_layer1.values.mean():.4f} m³/m³")

    return ds, sm_layer1

def interpolate_soil_moisture(sm_data, lon_grid, lat_grid):
    """将土壤湿度插值到DEM网格"""
    print("\n" + "="*60)
    print("3. 土壤湿度插值到DEM网格")
    print("="*60)

    # 获取土壤湿度的坐标
    sm_lon = sm_data.longitude.values
    sm_lat = sm_data.latitude.values
    sm_values = sm_data.values

    # 创建土壤湿度的坐标网格
    sm_lon_grid, sm_lat_grid = np.meshgrid(sm_lon, sm_lat)

    # 展平用于插值
    points = np.column_stack([sm_lon_grid.flatten(), sm_lat_grid.flatten()])
    values = sm_values.flatten()

    # 插值到DEM网格
    sm_interpolated = griddata(
        points,
        values,
        (lon_grid, lat_grid),
        method='linear'
    )

    print(f"  原始土壤湿度形状: {sm_values.shape}")
    print(f"  插值后形状: {sm_interpolated.shape}")
    print(f"  插值后范围: {np.nanmin(sm_interpolated):.4f} ~ {np.nanmax(sm_interpolated):.4f} m³/m³")

    return sm_interpolated

def load_roads():
    """加载道路数据"""
    print("\n" + "="*60)
    print("4. 加载道路数据")
    print("="*60)

    roads_path = DATA_DIR / "osm_roads.shp"
    roads = gpd.read_file(roads_path)

    # 分类道路
    road_types = {
        'major': ['trunk', 'primary', 'secondary'],  # 主要道路
        'minor': ['tertiary', 'residential', 'unclassified'],  # 次要道路
        'track': ['track', 'service'],  # 小路/土路
        'path': ['footway', 'path', 'steps', 'pedestrian']  # 人行道/步道
    }

    print(f"  总道路数量: {len(roads)}")
    print(f"  道路类型分布:")
    for highway_type, count in roads['highway'].value_counts().items():
        print(f"    {highway_type}: {count}")

    return roads, road_types

def load_waterways():
    """加载水系数据"""
    print("\n" + "="*60)
    print("5. 加载水系数据")
    print("="*60)

    water_path = DATA_DIR / "osm_waterways.shp"
    waterways = gpd.read_file(water_path)

    print(f"  水系要素数量: {len(waterways)}")

    return waterways

def calculate_terrain_features(dem, transform):
    """计算地形特征（坡度、坡向）"""
    print("\n" + "="*60)
    print("6. 计算地形特征")
    print("="*60)

    from scipy.ndimage import sobel

    # 计算坡度（使用Sobel算子）
    dx = sobel(dem, axis=1) / (transform.a * 2)
    dy = sobel(dem, axis=0) / (transform.e * 2)
    slope = np.arctan(np.sqrt(dx**2 + dy**2)) * 180 / np.pi

    # 计算坡向
    aspect = np.arctan2(-dy, dx) * 180 / np.pi
    aspect = (aspect + 360) % 360  # 转换到0-360度

    print(f"  坡度范围: {np.nanmin(slope):.2f}° ~ {np.nanmax(slope):.2f}°")
    print(f"  坡向范围: {np.nanmin(aspect):.2f}° ~ {np.nanmax(aspect):.2f}°")

    return slope, aspect

def calculate_road_density(roads, bounds, grid_shape):
    """计算道路密度"""
    print("\n" + "="*60)
    print("7. 计算道路密度")
    print("="*60)

    # 创建道路密度网格
    road_density = np.zeros(grid_shape)

    # 统计每个网格内的道路数量
    for idx, road in roads.iterrows():
        geom = road.geometry
        if geom.geom_type == 'LineString':
            coords = np.array(geom.coords)
            for coord in coords:
                # 计算网格索引
                col = int((coord[0] - bounds.left) / (bounds.right - bounds.left) * grid_shape[1])
                row = int((bounds.top - coord[1]) / (bounds.top - bounds.bottom) * grid_shape[0])

                if 0 <= row < grid_shape[0] and 0 <= col < grid_shape[1]:
                    road_density[row, col] += 1

    print(f"  道路密度网格形状: {road_density.shape}")
    print(f"  最大道路密度: {road_density.max():.0f}")

    return road_density

def calculate_water_distance(waterways, bounds, grid_shape, transform):
    """计算到水系的距离"""
    print("\n" + "="*60)
    print("8. 计算到水系的距离")
    print("="*60)

    from scipy.ndimage import distance_transform_edt

    # 创建水系栅格
    water_raster = np.zeros(grid_shape)

    for idx, water in waterways.iterrows():
        geom = water.geometry
        if geom.geom_type == 'LineString':
            coords = np.array(geom.coords)
            for coord in coords:
                col = int((coord[0] - bounds.left) / transform.a)
                row = int((coord[1] - bounds.top) / transform.e)

                if 0 <= row < grid_shape[0] and 0 <= col < grid_shape[1]:
                    water_raster[row, col] = 1

    # 计算距离
    water_distance = distance_transform_edt(1 - water_raster)
    water_distance = water_distance * abs(transform.a)  # 转换为地理单位（度）

    print(f"  水系栅格形状: {water_raster.shape}")
    print(f"  最大距离: {water_distance.max():.4f} 度")

    return water_distance

def create_trafficability_model(dem, slope, sm_interpolated, road_density, water_distance):
    """创建通行能力模型"""
    print("\n" + "="*60)
    print("9. 创建通行能力模型")
    print("="*60)

    # 归一化各因素
    def normalize(data):
        return (data - np.nanmin(data)) / (np.nanmax(data) - np.nanmin(data))

    slope_norm = normalize(slope)
    sm_norm = normalize(sm_interpolated)
    road_norm = normalize(road_density)
    water_norm = normalize(water_distance)

    # 通行能力得分（0-1，越高越容易通行）
    # 权重分配
    weights = {
        'slope': 0.4,      # 坡度影响最大
        'soil_moisture': 0.3,  # 土壤湿度
        'road_density': 0.2,   # 道路密度
        'water_distance': 0.1  # 到水系距离
    }

    # 计算综合得分
    trafficability = (
        weights['slope'] * (1 - slope_norm) +  # 坡度越小越好
        weights['soil_moisture'] * (1 - sm_norm) +  # 土壤越干越好
        weights['road_density'] * road_norm +  # 道路密度越高越好
        weights['water_distance'] * water_norm  # 距离水系越远越好
    )

    print(f"  权重分配:")
    for factor, weight in weights.items():
        print(f"    {factor}: {weight}")

    print(f"\n  通行能力得分:")
    print(f"    范围: {np.nanmin(trafficability):.4f} ~ {np.nanmax(trafficability):.4f}")
    print(f"    平均: {np.nanmean(trafficability):.4f}")

    return trafficability

def save_results(dem, slope, aspect, sm_interpolated, trafficability, transform, crs):
    """保存结果"""
    print("\n" + "="*60)
    print("10. 保存结果")
    print("="*60)

    output_dir = DATA_DIR / "analysis_results"
    output_dir.mkdir(exist_ok=True)

    # 保存各图层
    datasets = {
        'dem_12m.tif': dem,
        'slope.tif': slope,
        'aspect.tif': aspect,
        'soil_moisture.tif': sm_interpolated,
        'trafficability.tif': trafficability
    }

    for filename, data in datasets.items():
        output_path = output_dir / filename

        with rasterio.open(
            output_path,
            'w',
            driver='GTiff',
            height=data.shape[0],
            width=data.shape[1],
            count=1,
            dtype=data.dtype,
            crs=crs,
            transform=transform,
            compress='lzw'
        ) as dst:
            dst.write(data, 1)

        print(f"  [OK] {filename}")

    print(f"\n  所有结果保存到: {output_dir}")

def create_visualization(dem, slope, sm_interpolated, trafficability):
    """创建可视化"""
    print("\n" + "="*60)
    print("11. 创建可视化")
    print("="*60)

    fig, axes = plt.subplots(2, 2, figsize=(14, 12))

    # DEM
    im1 = axes[0, 0].imshow(dem, cmap='terrain')
    axes[0, 0].set_title('DEM (m)')
    plt.colorbar(im1, ax=axes[0, 0])

    # 坡度
    im2 = axes[0, 1].imshow(slope, cmap='YlOrRd')
    axes[0, 1].set_title('Slope (degrees)')
    plt.colorbar(im2, ax=axes[0, 1])

    # 土壤湿度
    im3 = axes[1, 0].imshow(sm_interpolated, cmap='Blues')
    axes[1, 0].set_title('Soil Moisture (m³/m³)')
    plt.colorbar(im3, ax=axes[1, 0])

    # 通行能力
    im4 = axes[1, 1].imshow(trafficability, cmap='RdYlGn')
    axes[1, 1].set_title('Trafficability Score')
    plt.colorbar(im4, ax=axes[1, 1])

    plt.tight_layout()

    output_path = DATA_DIR / "analysis_results" / "visualization.png"
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"  [OK] 可视化保存到: {output_path}")

    plt.close()

def main():
    """主函数"""
    print("="*60)
    print("雅江格西沟通行能力分析 - 数据叠加分析")
    print("="*60)

    # 1. 加载DEM
    dem, transform, crs, bounds, lon_grid, lat_grid = load_dem()

    # 2. 加载土壤湿度
    ds, sm_layer1 = load_soil_moisture()

    # 3. 插值土壤湿度到DEM网格
    sm_interpolated = interpolate_soil_moisture(sm_layer1, lon_grid, lat_grid)

    # 4. 加载道路
    roads, road_types = load_roads()

    # 5. 加载水系
    waterways = load_waterways()

    # 6. 计算地形特征
    slope, aspect = calculate_terrain_features(dem, transform)

    # 7. 计算道路密度
    road_density = calculate_road_density(roads, bounds, dem.shape)

    # 8. 计算到水系的距离
    water_distance = calculate_water_distance(waterways, bounds, dem.shape, transform)

    # 9. 创建通行能力模型
    trafficability = create_trafficability_model(
        dem, slope, sm_interpolated, road_density, water_distance
    )

    # 10. 保存结果
    save_results(dem, slope, aspect, sm_interpolated, trafficability, transform, crs)

    # 11. 创建可视化
    create_visualization(dem, slope, sm_interpolated, trafficability)

    print("\n" + "="*60)
    print("分析完成!")
    print("="*60)
    print("\n生成的文件:")
    print("  - analysis_results/dem_12m.tif")
    print("  - analysis_results/slope.tif")
    print("  - analysis_results/aspect.tif")
    print("  - analysis_results/soil_moisture.tif")
    print("  - analysis_results/trafficability.tif")
    print("  - analysis_results/visualization.png")

if __name__ == "__main__":
    main()
