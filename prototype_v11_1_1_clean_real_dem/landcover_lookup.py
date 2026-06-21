"""土地覆盖类别到基础附着能力的查表。"""
from __future__ import annotations
import csv
from pathlib import Path
import numpy as np


def read_lookup_csv(path: str | Path) -> dict[int, dict]:
    rows = {}
    with Path(path).open('r', encoding='utf-8-sig', newline='') as f:
        for r in csv.DictReader(f):
            cid = int(r['class_id'])
            out = dict(r)
            for k,v in list(out.items()):
                if k in ('class_id','class_name','notes'):
                    continue
                if k == 'hard_barrier':
                    out[k] = str(v).strip().lower() in ('1','true','yes','y')
                else:
                    try: out[k] = float(v)
                    except Exception: pass
            rows[cid] = out
    return rows


def map_landcover_to_mu(landcover: np.ndarray, lookup: dict[int, dict], scenario: str = 'normal') -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    scenario = scenario.lower()
    if scenario not in ('dry','normal','wet','post_rain'):
        scenario = 'normal'
    sc = 'wet' if scenario == 'post_rain' else scenario
    mu_s = np.full(landcover.shape, 0.5, dtype=float)
    mu_b = np.full(landcover.shape, 0.5, dtype=float)
    hard = np.zeros(landcover.shape, dtype=bool)
    for cid, row in lookup.items():
        mask = landcover == cid
        if not mask.any():
            continue
        mu_s[mask] = float(row.get(f'mu_slide_{sc}', row.get('mu_slide_normal', 0.5)))
        mu_b[mask] = float(row.get(f'mu_brake_{sc}', row.get('mu_brake_normal', 0.5)))
        hard[mask] = bool(row.get('hard_barrier', False))
    return mu_s, mu_b, hard
