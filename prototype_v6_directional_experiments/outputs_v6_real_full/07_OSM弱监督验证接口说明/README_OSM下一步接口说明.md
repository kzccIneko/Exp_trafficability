# OSM 弱监督验证接口说明

本版 v6 是 DEM-only 实验包，不强行运行 OSM 验证，避免因为缺少道路矢量数据导致主实验无法闭环。

下一次可在 `osm_validation.py` 基础上实现：

1. 读取 OSM 道路矢量文件，例如 `.shp`、`.geojson`、`.gpkg`；
2. 筛选道路类型：`highway=track/path/unclassified/service/tertiary` 等；
3. 根据 DEM 栅格范围和分辨率栅格化道路；
4. 道路缓冲区作为正样本 `S+`；
5. 在相同坡度带、高程带、土地覆盖背景下抽取非道路负样本 `S-`；
6. 用模型的最小方向代价或方向平均代价评价道路与非道路样本；
7. 输出 `Delta C = mean(C-) - mean(C+)` 与 `AUC=P(Croad<Crandom)`。

注意：OSM 不是越野车辆真实通行能力的绝对真值，只是长期人类/车辆通行选择的弱证据。
