"""v10 绘图工具。"""
from __future__ import annotations
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def setup_font():
    import matplotlib.font_manager as fm
    for name in ['SimHei','Microsoft YaHei','SimSun','Noto Sans CJK SC','WenQuanYi Micro Hei']:
        try:
            p = fm.findfont(fm.FontProperties(family=name), fallback_to_default=False)
            if p:
                matplotlib.rcParams['font.sans-serif'] = [name] + matplotlib.rcParams.get('font.sans-serif', [])
                break
        except Exception:
            pass
    matplotlib.rcParams['axes.unicode_minus'] = False



def nan_reduce_2d(arr: np.ndarray, mode: str = "min") -> np.ndarray:
    """Reduce a 3D directional cost array without emitting All-NaN warnings.

    If all directions at a cell are invalid, the output cell remains NaN.
    """
    a = np.asarray(arr, dtype=float)
    if a.ndim != 3:
        return a
    finite = np.isfinite(a)
    out = np.full(a.shape[:2], np.nan, dtype=float)
    if mode == "max":
        tmp = np.where(finite, a, -np.inf)
        vals = np.max(tmp, axis=2)
        out[np.any(finite, axis=2)] = vals[np.any(finite, axis=2)]
    else:
        tmp = np.where(finite, a, np.inf)
        vals = np.min(tmp, axis=2)
        out[np.any(finite, axis=2)] = vals[np.any(finite, axis=2)]
    return out

def safe_limits(*arrs):
    vals=[]
    for a in arrs:
        aa=np.asarray(a); vals.append(aa[np.isfinite(aa)].ravel())
    vals=np.concatenate([v for v in vals if v.size]) if vals else np.array([0,1])
    lo,hi=np.percentile(vals,[2,98])
    if abs(hi-lo)<1e-9: hi=lo+1
    return lo,hi


def save_map_grid(path, maps, titles, cmap='viridis', suptitle=None):
    setup_font(); n=len(maps); fig, axes=plt.subplots(1,n,figsize=(5*n,4.5), squeeze=False)
    axes=axes.ravel()
    for ax, data, title in zip(axes,maps,titles):
        im=ax.imshow(data, origin='lower', cmap=cmap)
        ax.set_title(title); ax.set_xlabel('列号'); ax.set_ylabel('行号')
        plt.colorbar(im, ax=ax, shrink=0.8)
    if suptitle: fig.suptitle(suptitle)
    fig.tight_layout(); fig.savefig(path, dpi=180, bbox_inches='tight'); plt.close(fig)


def plot_surface_params(path, params):
    save_map_grid(path, [params['wetness_effective'], params['mu_slide'], params['mu_brake'], params['landcover']],
                  ['湿润倾向 W', '侧滑附着能力 μs(x,y)', '制动附着能力 μb(x,y)', '土地覆盖类别'], cmap='viridis', suptitle='PTF-Lite 地表附着参数空间化')


def plot_direction_cost_maps(path, fields, parts):
    setup_font(); C=fields['V10空间附着']; mn=nan_reduce_2d(C, 'min'); mx=nan_reduce_2d(C, 'max'); diff=mx-mn
    dom = np.nanmin(parts['dominant_limit'],axis=2) if parts['dominant_limit'].ndim==3 else parts['dominant_limit'][:,:,0]
    fig,axes=plt.subplots(1,4,figsize=(20,4.6))
    for ax,data,title,cmap in zip(axes,[mn,mx,diff,dom],['最小方向代价','最大方向代价','方向代价差异','主导限制类型示例'],['RdYlGn_r','RdYlGn_r','magma','tab10']):
        im=ax.imshow(data,origin='lower',cmap=cmap); ax.set_title(title); ax.set_xlabel('列号'); ax.set_ylabel('行号'); plt.colorbar(im,ax=ax,shrink=0.8)
    fig.tight_layout(); fig.savefig(path,dpi=180,bbox_inches='tight'); plt.close(fig)


def plot_paths(path, cost_map, paths, start, goal, title='路径对比'):
    setup_font(); fig,ax=plt.subplots(figsize=(7,6))
    base=nan_reduce_2d(cost_map, 'min') if cost_map.ndim==3 else cost_map
    im=ax.imshow(base, origin='lower', cmap='RdYlGn_r')
    plt.colorbar(im,ax=ax,shrink=0.8,label='单位方向代价最小值')
    for name,p in paths.items():
        if not p: continue
        rr=[x[0] for x in p]; cc=[x[1] for x in p]
        ax.plot(cc,rr,label=name,linewidth=2)
    ax.scatter([start[1]],[start[0]],marker='o',s=50,label='起点')
    ax.scatter([goal[1]],[goal[0]],marker='*',s=80,label='终点')
    ax.set_title(title); ax.set_xlabel('列号'); ax.set_ylabel('行号'); ax.legend(loc='best',fontsize=8)
    fig.tight_layout(); fig.savefig(path,dpi=180,bbox_inches='tight'); plt.close(fig)


def plot_profiles(path, profiles, title='沿途车辆能力利用率'):
    setup_font(); fig,axes=plt.subplots(4,1,figsize=(9,10),sharex=True)
    keys=['rho_up','rho_down','rho_roll','rho_slide']; names=['上坡牵引利用率','下坡制动利用率','侧翻稳定性利用率','侧滑附着利用率']
    for ax,key,name in zip(axes,keys,names):
        for model, prof in profiles.items():
            if not prof: continue
            x=[r['cum_dist_m'] for r in prof]; y=[r[key] for r in prof]
            ax.plot(x,y,label=model,linewidth=1.8)
        ax.axhline(1.0,linestyle='--',linewidth=1)
        ax.axhline(0.7,linestyle=':',linewidth=1)
        ax.set_ylabel(name); ax.grid(True,alpha=0.3)
    axes[-1].set_xlabel('沿路径距离 / m'); axes[0].legend(loc='best',fontsize=8)
    fig.suptitle(title); fig.tight_layout(); fig.savefig(path,dpi=180,bbox_inches='tight'); plt.close(fig)


def plot_bar_metrics(path, rows, title='路径高利用率比例'):
    setup_font(); labels=[r.get('模型代码',r.get('模型名称','model')) for r in rows]
    keys=['P_rho_roll_gt_0p7','P_rho_slide_gt_0p7','P_rho_max_gt_1p0']
    names=['高侧翻稳定性利用率比例','高侧滑附着利用率比例','约束超限路径比例']
    x=np.arange(len(labels)); width=0.25
    fig,ax=plt.subplots(figsize=(max(7,len(labels)*1.2),4.8))
    for i,(k,n) in enumerate(zip(keys,names)):
        vals=[float(r.get(k,0) or 0) for r in rows]
        ax.bar(x+(i-1)*width,vals,width,label=n)
    ax.set_xticks(x); ax.set_xticklabels(labels,rotation=25,ha='right'); ax.set_ylim(0,1); ax.set_ylabel('路径长度比例'); ax.set_title(title); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(path,dpi=180,bbox_inches='tight'); plt.close(fig)


def plot_ablation_summary(path, rows):
    setup_font(); labels=[r['版本'] for r in rows]
    vals=[float(r.get('P_rho_max_gt_1p0_mean',0)) for r in rows]
    fig,ax=plt.subplots(figsize=(8,4.6))
    ax.bar(labels, vals); ax.set_ylabel('平均约束超限路径比例'); ax.set_title('空间化参数消融实验'); ax.tick_params(axis='x', rotation=25)
    fig.tight_layout(); fig.savefig(path,dpi=180,bbox_inches='tight'); plt.close(fig)
