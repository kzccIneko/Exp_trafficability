"""
下载 ALOS PALSAR DEM 12.5m 数据
使用 NASA Earthdata Token 认证
"""

import requests
import asf_search as asf
from shapely.geometry import box
import os

# 分析区域
BBOX = {
    'left': 100.846,
    'right': 101.124,
    'bottom': 29.876,
    'top': 30.154
}

# NASA Earthdata Token - 从环境变量读取
EARTHDATA_TOKEN = os.environ.get('EARTHDATA_TOKEN', 'your_earthdata_token_here')

OUTPUT_DIR = "."

def search_alos_data():
    """搜索 ALOS PALSAR 数据"""
    print("="*60)
    print("搜索 ALOS PALSAR DEM 数据")
    print("="*60)

    print(f"\n搜索区域:")
    print(f"  经度: {BBOX['left']}° ~ {BBOX['right']}°")
    print(f"  纬度: {BBOX['bottom']}° ~ {BBOX['top']}°")

    # 创建 WKT 多边形
    bbox_geom = box(BBOX['left'], BBOX['bottom'], BBOX['right'], BBOX['top'])
    wkt = bbox_geom.wkt

    print(f"\n正在搜索 ASF 数据中心...")

    try:
        # 搜索 ALOS PALSAR RTC 数据
        results = asf.search(
            intersectsWith=wkt,
            dataset=asf.DATASET.ALOS_PALSAR,
            processingLevel='RTC_HI_RES',
            maxResults=10
        )

        print(f"\n找到 {len(results)} 个数据集")

        if len(results) == 0:
            print("\n尝试搜索其他 ALOS 数据类型...")
            results = asf.search(
                intersectsWith=wkt,
                dataset=asf.DATASET.ALOS_PALSAR,
                maxResults=10
            )
            print(f"找到 {len(results)} 个数据集")

        # 显示搜索结果
        for i, result in enumerate(results[:5]):
            props = result.properties
            print(f"\n数据集 {i+1}:")
            print(f"  文件名: {props.get('fileName', 'N/A')}")
            print(f"  日期: {props.get('startTime', 'N/A')[:10]}")
            print(f"  大小: {props.get('sizeMB', 'N/A')} MB")
            print(f"  极化: {props.get('polarization', 'N/A')}")

        return results

    except Exception as e:
        print(f"\n[ERROR] 搜索失败: {e}")
        import traceback
        traceback.print_exc()
        return []

def download_alos_dem():
    """下载 ALOS DEM 数据"""
    print("\n" + "="*60)
    print("下载 ALOS PALSAR DEM")
    print("="*60)

    results = search_alos_data()

    if not results:
        print("\n[INFO] 未找到合适的 ALOS 数据")
        print("\n替代方案:")
        print("1. 使用已下载的 Copernicus DEM 12.5m (重采样版)")
        print("2. 手动从 ASF 网站下载")
        print("   https://search.asf.alaska.edu/")
        return False

    # 选择第一个结果
    selected = results[0]
    download_url = selected.properties.get('url')
    filename = selected.properties.get('fileName', 'alos_dem')

    if not download_url:
        print("\n[ERROR] 无法获取下载链接")
        return False

    output_file = os.path.join(OUTPUT_DIR, f"alos_palsar_dem_{filename}")

    print(f"\n准备下载:")
    print(f"  文件: {filename}")
    print(f"  大小: {selected.properties.get('sizeMB', 'N/A')} MB")
    print(f"  保存到: {output_file}")

    try:
        # 使用 token 进行认证下载
        print("\n正在下载...")

        headers = {
            'Authorization': f'Bearer {EARTHDATA_TOKEN}',
            'User-Agent': 'GeoDataDownloader/1.0'
        }

        response = requests.get(download_url, headers=headers, stream=True, timeout=300)
        response.raise_for_status()

        with open(output_file, 'wb') as f:
            for chunk in response.iter_content(chunk_size=8192):
                f.write(chunk)

        print(f"\n[OK] 下载完成!")
        print(f"文件: {output_file}")

        return True

    except Exception as e:
        print(f"\n[ERROR] 下载失败: {e}")
        print("\n可能的原因:")
        print("1. Token 无效或过期")
        print("2. 需要先在 Earthdata 网站授权 ASF 应用")
        print("3. 网络连接问题")

        # 保存下载链接供手动下载
        urls_file = os.path.join(OUTPUT_DIR, "alos_download_urls.txt")
        with open(urls_file, 'w', encoding='utf-8') as f:
            f.write("ALOS PALSAR DEM 下载链接\n")
            f.write("="*60 + "\n\n")
            f.write("请手动访问以下链接下载:\n\n")
            for i, r in enumerate(results[:5]):
                f.write(f"数据集 {i+1}:\n")
                f.write(f"  文件: {r.properties.get('fileName')}\n")
                f.write(f"  URL: {r.properties.get('url')}\n")
                f.write(f"  大小: {r.properties.get('sizeMB')} MB\n\n")

        print(f"\n下载链接已保存到: {urls_file}")
        print("你可以手动复制链接到浏览器下载")

        return False

def main():
    print("="*60)
    print("ALOS PALSAR DEM 12.5m 下载工具")
    print("="*60)

    download_alos_dem()

    print("\n" + "="*60)
    print("完成!")
    print("="*60)

if __name__ == "__main__":
    main()
