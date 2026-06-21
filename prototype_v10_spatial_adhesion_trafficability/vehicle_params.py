"""
vehicle_params.py

v10 示例车辆参数：Jeep Wrangler Rubicon 四门版。

公开参数只作为可查的车辆几何与质量参数；没有公开的质心高度、附着系数、最大可接受爬坡角不伪装为官方值，
而作为实验场景参数和敏感性分析参数。
"""
from __future__ import annotations

from dataclasses import dataclass, asdict, replace
import math
from pathlib import Path
import csv

INCH_TO_M = 0.0254
LB_TO_KG = 0.45359237


@dataclass(frozen=True)
class VehicleParams:
    # Public vehicle specifications
    name: str
    mass_kg: float
    wheelbase_m: float
    track_width_m: float
    vehicle_height_m: float
    ground_clearance_m: float
    approach_angle_deg: float
    departure_angle_deg: float
    breakover_angle_deg: float
    # Scenario / sensitivity parameters, not official vehicle values
    cg_height_ratio: float = 0.45       # h_c / vehicle_height_m
    max_grade_angle_deg: float = 30.0   # maximum acceptable climbing angle scenario
    mu_slide: float = 0.50              # lateral ground adhesion scenario
    mu_brake: float = 0.50              # downhill braking adhesion scenario

    @property
    def cg_height_m(self) -> float:
        return self.cg_height_ratio * self.vehicle_height_m

    @property
    def max_grade_angle_rad(self) -> float:
        return math.radians(self.max_grade_angle_deg)

    @property
    def static_stability_factor(self) -> float:
        return self.track_width_m / (2.0 * self.cg_height_m)

    @property
    def static_roll_limit_deg(self) -> float:
        return math.degrees(math.atan(self.static_stability_factor))

    @property
    def breakover_angle_rad(self) -> float:
        return math.radians(self.breakover_angle_deg)


def jeep_wrangler_rubicon_4door(**overrides) -> VehicleParams:
    """Jeep Wrangler Rubicon 四门版公开参数 + 明确标注的场景参数。"""
    base = VehicleParams(
        name="Jeep Wrangler Rubicon 4-door (public-spec example)",
        mass_kg=4320.0 * LB_TO_KG,
        wheelbase_m=118.4 * INCH_TO_M,
        track_width_m=62.9 * INCH_TO_M,
        vehicle_height_m=73.6 * INCH_TO_M,
        ground_clearance_m=10.8 * INCH_TO_M,
        approach_angle_deg=43.9,
        departure_angle_deg=37.0,
        breakover_angle_deg=22.6,
    )
    if overrides:
        base = replace(base, **overrides)
    return base


def vehicle_parameter_rows(v: VehicleParams) -> list[dict]:
    d = asdict(v)
    public_keys = {
        "mass_kg", "wheelbase_m", "track_width_m", "vehicle_height_m",
        "ground_clearance_m", "approach_angle_deg", "departure_angle_deg", "breakover_angle_deg",
    }
    labels = {
        "mass_kg": "整备质量",
        "wheelbase_m": "轴距",
        "track_width_m": "轮距",
        "vehicle_height_m": "车高",
        "ground_clearance_m": "运行离地间隙",
        "approach_angle_deg": "接近角",
        "departure_angle_deg": "离去角",
        "breakover_angle_deg": "通过角",
        "cg_height_ratio": "质心高度占车高比例 h_c/H",
        "max_grade_angle_deg": "最大可接受爬坡角场景",
        "mu_slide": "侧滑附着系数场景",
        "mu_brake": "下坡制动附着系数场景",
    }
    rows = []
    public_note = "公开车辆规格；用于示例车辆建模。"
    scenario_note = "实验场景/敏感性参数；不作为 Jeep 官方值。"
    for k, val in d.items():
        if k == "name":
            continue
        rows.append({
            "参数": k,
            "中文含义": labels.get(k, k),
            "数值": val,
            "单位": "m/kg/degree or dimensionless",
            "来源类型": "公开车辆参数" if k in public_keys else "场景/敏感性参数",
            "说明": public_note if k in public_keys else scenario_note,
        })
    rows.extend([
        {"参数": "cg_height_m", "中文含义": "质心高度 h_c", "数值": v.cg_height_m, "单位": "m", "来源类型": "由车高与 h_c/H 计算", "说明": "因为公开资料未给质心高度，所以由敏感性参数 h_c/H 计算。"},
        {"参数": "SSF", "中文含义": "静态稳定因子 B/(2h_c)", "数值": v.static_stability_factor, "单位": "dimensionless", "来源类型": "模型计算值", "说明": "用于侧翻稳定性利用率。"},
        {"参数": "static_roll_limit_deg", "中文含义": "简化静态侧翻参考角 atan(B/(2h_c))", "数值": v.static_roll_limit_deg, "单位": "degree", "来源类型": "模型计算值", "说明": "用于解释侧翻稳定性边界，不表示实车动态安全极限。"},
    ])
    return rows


def write_vehicle_params_csv(path: str | Path, v: VehicleParams) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = vehicle_parameter_rows(v)
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
