"""
vehicle_params.py

v7 使用一辆公开参数较完整的越野车作为示例车辆：
Jeep Wrangler Rubicon 四门版。公开参数来自 Stellantis Fleet 2026 Wrangler Buyer Guide
与 Jeep 官方 Wrangler capability 页面；非公开参数（质心高度、摩擦系数、最大可接受爬坡角）
不伪装为官方值，而作为敏感性分析参数。

单位统一为 SI：m, kg, rad, N。
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
    name: str
    mass_kg: float
    wheelbase_m: float
    track_width_m: float
    ground_clearance_m: float
    approach_angle_deg: float
    departure_angle_deg: float
    breakover_angle_deg: float
    # Assumption/sensitivity parameters
    cg_height_m: float = 0.85
    max_grade_angle_deg: float = 30.0
    rolling_resistance: float = 0.04
    mu_slide: float = 0.50
    mu_brake: float = 0.50
    eta_roll: float = 0.60
    eta_slide: float = 0.80
    eta_break: float = 0.70
    softplus_k: float = 8.0
    lambda_up: float = 1.0
    lambda_down: float = 1.0
    lambda_roll: float = 1.2
    lambda_slide: float = 1.0
    lambda_break: float = 0.35

    @property
    def breakover_angle_rad(self) -> float:
        return math.radians(self.breakover_angle_deg)

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
    def safe_roll_limit_deg(self) -> float:
        return math.degrees(math.atan(self.eta_roll * self.static_stability_factor))


def jeep_wrangler_rubicon_4door(**overrides) -> VehicleParams:
    """Jeep Wrangler Rubicon 四门版公开参数 + 明确标注的敏感性假设参数。"""
    base = VehicleParams(
        name="Jeep Wrangler Rubicon 4-door (public-spec example)",
        mass_kg=4320.0 * LB_TO_KG,
        wheelbase_m=118.4 * INCH_TO_M,
        track_width_m=62.9 * INCH_TO_M,
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
    rows = []
    public_keys = {
        "mass_kg", "wheelbase_m", "track_width_m", "ground_clearance_m",
        "approach_angle_deg", "departure_angle_deg", "breakover_angle_deg",
    }
    assumption_keys = set(d) - public_keys - {"name"}
    source_public = "Stellantis Fleet 2026 Wrangler Buyer Guide / Jeep official capability page; package docs include URLs."
    source_assumption = "Model assumption / sensitivity parameter; not claimed as an official Jeep value."
    labels = {
        "mass_kg": "整备质量",
        "wheelbase_m": "轴距",
        "track_width_m": "轮距",
        "ground_clearance_m": "运行离地间隙",
        "approach_angle_deg": "接近角",
        "departure_angle_deg": "离去角",
        "breakover_angle_deg": "通过角",
        "cg_height_m": "质心高度",
        "max_grade_angle_deg": "最大可接受爬坡角/牵引能力代理",
        "rolling_resistance": "滚动阻力系数",
        "mu_slide": "横向等效附着系数",
        "mu_brake": "下坡制动等效附着系数",
        "eta_roll": "横坡侧翻安全折减系数",
        "eta_slide": "侧滑安全折减系数",
        "eta_break": "几何通过安全折减系数",
        "softplus_k": "软屏障函数陡峭系数",
        "lambda_up": "上坡约束权重",
        "lambda_down": "下坡约束权重",
        "lambda_roll": "侧翻约束权重",
        "lambda_slide": "侧滑约束权重",
        "lambda_break": "几何通过约束权重",
    }
    for k, val in d.items():
        if k == "name":
            continue
        rows.append({
            "参数": k,
            "中文含义": labels.get(k, k),
            "数值": val,
            "单位": "SI or degree" if (k.endswith("_deg") or k.endswith("_m") or k.endswith("_kg")) else "dimensionless",
            "来源类型": "公开车辆参数" if k in public_keys else "假设/敏感性参数",
            "来源或处理说明": source_public if k in public_keys else source_assumption,
        })
    rows.append({
        "参数": "SSF",
        "中文含义": "静态稳定因子 B/(2h_c)",
        "数值": v.static_stability_factor,
        "单位": "dimensionless",
        "来源类型": "由公开轮距与假设质心高度计算",
        "来源或处理说明": "用于横坡侧翻能力利用率；质心高度需敏感性分析。",
    })
    rows.append({
        "参数": "safe_roll_limit_deg",
        "中文含义": "安全折减后的横坡参考角 atan(eta_roll*B/(2h_c))",
        "数值": v.safe_roll_limit_deg,
        "单位": "degree",
        "来源类型": "模型计算值",
        "来源或处理说明": "不是车辆官方极限，仅用于解释 eta_roll 折减后的模型边界。",
    })
    return rows


def write_vehicle_params_csv(path: str | Path, v: VehicleParams) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = vehicle_parameter_rows(v)
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
