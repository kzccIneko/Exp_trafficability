# OSM 弱监督验证接口说明

本 v7 包先不自动下载 OSM。下一步需要用户提供研究区内道路矢量文件：shp/geojson/gpkg。

建议正样本：highway=track/path/unclassified/service/tertiary 等山区道路或林道。
建议负样本：在相同坡度带、相近高程带、相近土地覆盖背景下采样非道路栅格。

验证指标：

- Delta C = mean(C_nonroad)-mean(C_road)
- AUC = P(C_road < C_nonroad)
- Delta rho_roll, Delta rho_up, Delta rho_slide

OSM 只能作为弱监督证据，不是真实车辆通行能力真值。
