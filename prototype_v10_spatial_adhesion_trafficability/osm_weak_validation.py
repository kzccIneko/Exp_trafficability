"""
osm_weak_validation.py

OSM 弱监督验证接口。当前代码包不强制运行该模块。
OSM 道路只能作为已有路径选择的弱证据，不是通行能力真值。
"""
from __future__ import annotations


def describe_osm_validation_plan() -> str:
    return """OSM 弱监督验证计划：
1. 提取 track/path/unclassified/service 等道路作为正样本缓冲区；
2. 在相近坡度、高程、土地覆盖背景中抽取非道路负样本；
3. 比较道路样本与非道路样本的单位代价、侧滑附着利用率、侧翻稳定性利用率；
4. 输出 AUC、均值差异和箱线图；
5. 结论只表述为与已有道路选择的一致性，不表述为实车真值验证。
"""
