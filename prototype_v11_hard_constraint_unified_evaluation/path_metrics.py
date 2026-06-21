"""路径约束利用率统计。"""
from __future__ import annotations
import numpy as np
from cost_model import nearest_direction_index


def edge_angle(r,c,nr,nc):
    return float(np.arctan2(nr-r, nc-c) % (2*np.pi))


def path_capability_profile(path, dem, cost_field, parts, directions, cell_size):
    if not path or len(path) < 2:
        return []
    rows = []
    cum = 0.0
    for k in range(len(path)-1):
        r,c = path[k]; nr,nc = path[k+1]
        theta = edge_angle(r,c,nr,nc)
        di = nearest_direction_index(theta, directions)
        length = float(np.hypot(nr-r, nc-c) * cell_size)
        rr = int(round((r+nr)/2)); cc = int(round((c+nc)/2))
        rr = max(0, min(dem.shape[0]-1, rr)); cc = max(0, min(dem.shape[1]-1, cc))
        cum += length
        dom = int(parts['dominant_limit'][rr,cc,di])
        rec = {
            'edge_id': k,
            'row': rr, 'col': cc,
            'theta_deg': np.degrees(theta),
            'edge_len_m': length,
            'cum_dist_m': cum,
            'elev_m': float(dem[rr,cc]),
            'unit_cost': float(cost_field[rr,cc,di]) if np.isfinite(cost_field[rr,cc,di]) else np.inf,
            'alpha_parallel_deg': float(np.degrees(parts['alpha_parallel_rad'][rr,cc,di])),
            'alpha_cross_deg': float(np.degrees(parts['alpha_cross_rad'][rr,cc,di])),
            'rho_up': float(parts['rho_up'][rr,cc,di]),
            'rho_down': float(parts['rho_down'][rr,cc,di]),
            'rho_roll': float(parts['rho_roll'][rr,cc,di]),
            'rho_slide': float(parts['rho_slide'][rr,cc,di]),
            'rho_max': float(parts['rho_max'][rr,cc,di]),
            'dominant_limit_code': dom,
            'dominant_limit_name': ['上坡牵引','下坡制动','侧翻稳定性','侧滑附着'][dom],
        }
        if 'mu_slide' in parts:
            rec['mu_slide'] = float(parts['mu_slide'][rr,cc])
        if 'mu_brake' in parts:
            rec['mu_brake'] = float(parts['mu_brake'][rr,cc])
        rows.append(rec)
    return rows


def summarize_profile(profile):
    if not profile:
        return {}
    L = sum(r['edge_len_m'] for r in profile)
    def wmean(key):
        return float(sum(r['edge_len_m'] * r[key] for r in profile) / max(L, 1e-9))
    def wfrac(key, th):
        return float(sum(r['edge_len_m'] for r in profile if r[key] > th) / max(L, 1e-9))
    def vmax(key):
        return float(max(r[key] for r in profile))
    dom_lengths = {}
    for r in profile:
        dom_lengths[r['dominant_limit_name']] = dom_lengths.get(r['dominant_limit_name'], 0.0) + r['edge_len_m']
    rec = {
        '评价路径长度_m': L,
        '平均上坡牵引利用率': wmean('rho_up'),
        '平均下坡制动利用率': wmean('rho_down'),
        '平均侧翻稳定性利用率': wmean('rho_roll'),
        '平均侧滑附着利用率': wmean('rho_slide'),
        '平均最大利用率': wmean('rho_max'),
        '最大上坡牵引利用率': vmax('rho_up'),
        '最大下坡制动利用率': vmax('rho_down'),
        '最大侧翻稳定性利用率': vmax('rho_roll'),
        '最大侧滑附着利用率': vmax('rho_slide'),
        '最大rho_max': vmax('rho_max'),
        'P_rho_roll_gt_0p7': wfrac('rho_roll', 0.7),
        'P_rho_slide_gt_0p7': wfrac('rho_slide', 0.7),
        'P_rho_up_gt_0p7': wfrac('rho_up', 0.7),
        'P_rho_down_gt_0p7': wfrac('rho_down', 0.7),
        'P_rho_max_gt_1p0': wfrac('rho_max', 1.0),
        'P_rho_max_gt_0p9': wfrac('rho_max', 0.9),
        '主导限制_上坡牵引_路径比例': dom_lengths.get('上坡牵引',0.0)/max(L,1e-9),
        '主导限制_下坡制动_路径比例': dom_lengths.get('下坡制动',0.0)/max(L,1e-9),
        '主导限制_侧翻稳定性_路径比例': dom_lengths.get('侧翻稳定性',0.0)/max(L,1e-9),
        '主导限制_侧滑附着_路径比例': dom_lengths.get('侧滑附着',0.0)/max(L,1e-9),
    }
    if 'mu_slide' in profile[0]: rec['平均侧滑附着能力_mu_s'] = wmean('mu_slide')
    if 'mu_brake' in profile[0]: rec['平均制动附着能力_mu_b'] = wmean('mu_brake')
    return rec


def indicator_explanations():
    return [
        {'指标':'平均上坡牵引利用率','解释':'整条路径平均用了多少比例的可接受爬坡能力。'},
        {'指标':'平均下坡制动利用率','解释':'整条路径平均用了多少比例的下坡制动/附着能力。'},
        {'指标':'平均侧翻稳定性利用率','解释':'整条路径平均用了多少比例的抗侧翻稳定能力。'},
        {'指标':'平均侧滑附着利用率','解释':'整条路径平均用了多少比例的横向附着能力。'},
        {'指标':'P_rho_max_gt_1p0','解释':'路径中超过至少一种车辆能力限制的长度比例。'},
        {'指标':'P_rho_slide_gt_0p7','解释':'路径中处于较高侧滑附着利用率状态的长度比例；0.7 是预警统计阈值，不是物理极限。'},
        {'指标':'P_rho_roll_gt_0p7','解释':'路径中处于较高侧翻稳定性利用率状态的长度比例；0.7 是预警统计阈值，不是物理极限。'},
    ]
