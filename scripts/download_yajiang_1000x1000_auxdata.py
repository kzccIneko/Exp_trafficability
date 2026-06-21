import os
import zipfile
from pathlib import Path

import ee
import requests


# =====================
# 1. 初始化 Earth Engine
# =====================
project = os.environ.get("EE_PROJECT")

try:
    if project:
        ee.Initialize(project=project)
    else:
        ee.Initialize()
except Exception:
    ee.Authenticate()
    if project:
        ee.Initialize(project=project)
    else:
        ee.Initialize()


# =====================
# 2. 与 DEM 对应的 1000x1000 bbox
# =====================
WEST = 100.846111
EAST = 101.123889
SOUTH = 29.876111
NORTH = 30.153889

roi = ee.Geometry.Rectangle(
    [WEST, SOUTH, EAST, NORTH],
    proj="EPSG:4326",
    geodesic=False
)

OUT = Path("yajiang_auxdata_1000x1000")
OUT.mkdir(exist_ok=True)

DIMENSIONS = "1000x1000"


# =====================
# 3. 通用下载函数
# =====================
def download_ee_image(image, out_name):
    out_path = OUT / out_name

    params = {
        "region": roi,
        "dimensions": DIMENSIONS,
        "crs": "EPSG:4326",
        "format": "GEO_TIFF",
        "filePerBand": False,
        "name": out_path.stem,
    }

    print(f"\n生成下载链接：{out_name}")
    url = image.clip(roi).getDownloadURL(params)

    tmp_path = OUT / f"{out_path.stem}.download"

    print(f"下载：{out_name}")
    with requests.get(url, stream=True) as r:
        r.raise_for_status()
        with open(tmp_path, "wb") as f:
            for chunk in r.iter_content(chunk_size=1024 * 1024):
                if chunk:
                    f.write(chunk)

    if zipfile.is_zipfile(tmp_path):
        unzip_dir = OUT / f"{out_path.stem}_unzipped"
        unzip_dir.mkdir(exist_ok=True)

        with zipfile.ZipFile(tmp_path, "r") as z:
            z.extractall(unzip_dir)

        tif_files = list(unzip_dir.glob("*.tif"))

        if len(tif_files) == 1:
            tif_files[0].replace(out_path)
            tmp_path.unlink(missing_ok=True)
        else:
            print(f"返回多个 TIF，已解压到：{unzip_dir}")
            tmp_path.unlink(missing_ok=True)
            return
    else:
        tmp_path.replace(out_path)

    print(f"完成：{out_path}")


# =====================
# 4. 土地覆盖：GLC_FCS30D 2022，30m
# =====================
# b1-b23 对应 2000-2022，所以 b23 是 2022
landcover = (
    ee.ImageCollection("projects/sat-io/open-datasets/GLC-FCS30D/annual")
    .mosaic()
    .select("b23")
    .rename("landcover_glc_fcs30d_2022")
    .toUint16()
)

download_ee_image(
    landcover,
    "landcover_glc_fcs30d_2022_1000x1000.tif"
)


# =====================
# 5. 水系/水体：JRC Global Surface Water，30m
# =====================
water = (
    ee.Image("JRC/GSW1_4/GlobalSurfaceWater")
    .select([
        "occurrence",
        "seasonality",
        "recurrence",
        "transition",
        "max_extent"
    ])
    .rename([
        "water_occurrence_percent",
        "water_seasonality_months",
        "water_recurrence_percent",
        "water_transition_class",
        "water_max_extent"
    ])
    .toFloat()
)

download_ee_image(
    water,
    "water_jrc_gsw_1000x1000.tif"
)


# =====================
# 6. 植被：Landsat 8/9 NDVI，30m
# =====================
START_DATE = "2022-06-01"
END_DATE = "2022-10-01"


def landsat_l2_ndvi(img):
    qa = img.select("QA_PIXEL")

    # 去掉云、云阴影、雪
    mask = (
        qa.bitwiseAnd(1 << 1).eq(0)
        .And(qa.bitwiseAnd(1 << 3).eq(0))
        .And(qa.bitwiseAnd(1 << 4).eq(0))
        .And(qa.bitwiseAnd(1 << 5).eq(0))
    )

    red = img.select("SR_B4").multiply(0.0000275).add(-0.2)
    nir = img.select("SR_B5").multiply(0.0000275).add(-0.2)

    ndvi = nir.subtract(red).divide(nir.add(red)).rename("ndvi")
    return ndvi.updateMask(mask)


l8 = (
    ee.ImageCollection("LANDSAT/LC08/C02/T1_L2")
    .filterBounds(roi)
    .filterDate(START_DATE, END_DATE)
    .map(landsat_l2_ndvi)
)

l9 = (
    ee.ImageCollection("LANDSAT/LC09/C02/T1_L2")
    .filterBounds(roi)
    .filterDate(START_DATE, END_DATE)
    .map(landsat_l2_ndvi)
)

ndvi = (
    l8.merge(l9)
    .median()
    .rename("ndvi_landsat_2022")
    .multiply(10000)
    .toInt16()
)

download_ee_image(
    ndvi,
    "vegetation_ndvi_landsat_2022_1000x1000.tif"
)


# =====================
# 7. 土壤：OpenLandMap
# =====================
# 注意：这些土壤数据源数据不是30m，通常是250m。
# 这里会重采样为 1000x1000，用来和 DEM 对齐。

clay = (
    ee.Image("OpenLandMap/SOL/SOL_CLAY-WFRACTION_USDA-3A1A1A_M/v02")
    .select("b0")
    .rename("clay_percent_0cm")
)

sand = (
    ee.Image("OpenLandMap/SOL/SOL_SAND-WFRACTION_USDA-3A1A1A_M/v02")
    .select("b0")
    .rename("sand_percent_0cm")
)

silt = (
    ee.Image(100)
    .subtract(clay)
    .subtract(sand)
    .rename("silt_percent_0cm_estimated")
)

ph = (
    ee.Image("OpenLandMap/SOL/SOL_PH-H2O_USDA-4C1A2A_M/v02")
    .select("b0")
    .divide(10)
    .rename("ph_h2o_0cm")
)

bulk_density = (
    ee.Image("OpenLandMap/SOL/SOL_BULKDENS-FINEEARTH_USDA-4A1H_M/v02")
    .select("b0")
    .multiply(10)
    .rename("bulk_density_kg_m3_0cm")
)

organic_carbon = (
    ee.Image("OpenLandMap/SOL/SOL_ORGANIC-CARBON_USDA-6A1C_M/v02")
    .select("b0")
    .multiply(5)
    .rename("organic_carbon_g_kg_0cm")
)

soil_properties = (
    ee.Image.cat([
        clay,
        sand,
        silt,
        ph,
        bulk_density,
        organic_carbon
    ])
    .resample("bilinear")
    .toFloat()
)

download_ee_image(
    soil_properties,
    "soil_openlandmap_properties_1000x1000.tif"
)


soil_texture = (
    ee.Image("OpenLandMap/SOL/SOL_TEXTURE-CLASS_USDA-TT_M/v02")
    .select("b0")
    .rename("soil_texture_class_usda_0cm")
    .resample("nearest")
    .toUint8()
)

download_ee_image(
    soil_texture,
    "soil_openlandmap_texture_1000x1000.tif"
)


print("\n全部下载完成。输出文件夹：", OUT.resolve())