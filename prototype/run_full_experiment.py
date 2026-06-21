"""
完整实验脚本 — 运行全部实验并保存JSON数据
"""
import sys, os, json, time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from prototype.terrain_analysis import (calculate_gradient, calculate_slope,
    calculate_aspect, compute_longitudinal_cross_slope)
from prototype.cost_model import (compute_impedance_field, compute_scalar_cost_field,
    compute_longitudinal_only_cost_field, longitudinal_impedance, cross_slope_impedance,
    scalar_slope_impedance, edge_cost)
from prototype.path_planning import find_optimal_path
from prototype.utils import generate_synthetic_dem, load_real_dem

import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

# 中文字体
for _fn in ['SimHei', 'Microsoft YaHei']:
    try:
        _fp = fm.findfont(fm.FontProperties(family=_fn), fallback_to_default=False)
        if _fp and 'DejaVu' not in _fp:
            matplotlib.rcParams['font.sans-serif'] = [_fn]
            matplotlib.rcParams['font.family'] = 'sans-serif'
            matplotlib.rcParams['axes.unicode_minus'] = False
            break
    except: pass

OUT = os.path.dirname(os.path.abspath(__file__))
RESULTS = {}

def ai(cf, eps=1e-8):
    c = np.maximum(cf, 0)
    return (np.max(c,2)-np.min(c,2))/(np.max(c,2)+np.min(c,2)+eps)

def auc(rc, nc):
    c = sum(np.sum(rc[i] < nc) for i in range(len(rc)))
    return c/(len(rc)*len(nc))

def dc(pts, dirs, cf, thetas):
    vals = []
    for (r,c), th in zip(pts, dirs):
        r,c = int(r), int(c)
        opt = thetas[np.argmin(cf[r,c,:])]
        vals.append(abs(np.cos(th-opt)))
    return np.mean(vals)

def neg_samples(pts, dirs, shape, cf, thetas, cs, seed=42):
    rng = np.random.RandomState(seed)
    R, C = shape; costs, pts2 = [], []
    for rp, th in zip(pts, dirs):
        r, c = int(rp[0]), int(rp[1])
        for _ in range(50):
            dr, dc_ = rng.randint(-10,11), rng.randint(-10,11)
            nr, nc = r+dr, c+dc_
            if 0<=nr<R and 0<=nc<C and np.sqrt(dr**2+dc_**2)>=3:
                ad = np.abs(thetas-th)
                ad = np.minimum(ad, 2*np.pi-ad)
                costs.append(cf[nr,nc,np.argmin(ad)])
                pts2.append((nr,nc)); break
    return np.array(costs), np.array(pts2)

def main():
    print("="*60)
    print("完整实验 — 方向敏感越野通行能力建模")
    print("="*60)

    real = len(sys.argv)>1 and sys.argv[1]=='real'
    tif = os.path.join(os.path.dirname(OUT), 'yajiang_gesigou_srtmgl1_30m_1000px_bbox.tif')
    if real and os.path.exists(tif):
        dem, cs, meta = load_real_dem(tif, max_pixels=200)
        src = 'real'
    else:
        rows, cols = 80, 80; cs = 30.0
        dem = generate_synthetic_dem(rows, cols, cs, seed=42)
        src = 'synthetic'
    rows, cols = dem.shape
    print(f"DEM: {src}, {rows}x{cols}, 栅格{cs:.1f}m, 高差{dem.max()-dem.min():.1f}m")

    # 地形分析
    p, q = calculate_gradient(dem, cs)
    slope = calculate_slope(p, q)
    aspect = calculate_aspect(p, q)
    N = 8
    thetas = np.linspace(0, 2*np.pi, N, endpoint=False)

    # === 实验1: 三模型代价场 ===
    print("\n[实验1] 三模型代价场...")
    cB0 = compute_scalar_cost_field(p, q, cs, alpha_u=15.0)
    cB1 = compute_longitudinal_only_cost_field(p, q, thetas, cs, 15.0, 5.0, 15.0)
    imp, cO = compute_impedance_field(p, q, thetas, cs, 15.0, 5.0, 15.0, 10.0)

    aiB1, aiO = ai(cB1), ai(cO)
    RESULTS['exp1'] = {
        'shape':[rows,cols], 'cs':float(cs),
        'slope_range':[float(np.degrees(slope.min())), float(np.degrees(slope.max()))],
        'slope_mean':float(np.degrees(slope.mean())),
        'B0':[float(cB0.min()),float(cB0.max()),float(cB0.mean())],
        'B1':[float(cB1.min()),float(cB1.max()),float(cB1.mean())],
        'Ours':[float(cO.min()),float(cO.max()),float(cO.mean())],
        'AI_B1':[float(aiB1.min()),float(aiB1.max()),float(aiB1.mean())],
        'AI_Ours':[float(aiO.min()),float(aiO.max()),float(aiO.mean())],
        'n_dir':N
    }
    print(f"  B0: [{cB0.min():.4f},{cB0.max():.4f}], AI_B1={aiB1.mean():.4f}, AI_Ours={aiO.mean():.4f}")

    # === 实验2: 路径规划 ===
    print("\n[实验2] 路径规划...")
    m = max(5, int(rows*0.15))
    start, goal = (m,m), (rows-m-1, cols-m-1)
    # 边界惩罚
    bw = max(3, int(rows*0.05))
    for cf in [cO, cB1]:
        for i in range(bw):
            pen = 2.0*(1.0-i/bw)
            cf[i,:,:]+=pen; cf[-(i+1),:,:]+=pen
            cf[:,i,:]+=pen; cf[:,-(i+1),:]+=pen

    se = np.concatenate([cB0]*N, axis=2)
    pB0,tB0,aB0 = find_optimal_path(se, start, goal)
    pB1,tB1,aB1 = find_optimal_path(cB1, start, goal)
    pO,tO,aO = find_optimal_path(cO, start, goal)
    RESULTS['exp2'] = {
        'start':list(start),'goal':list(goal),
        'B0':[len(pB0),float(aB0),float(tB0)],
        'B1':[len(pB1),float(aB1),float(tB1)],
        'Ours':[len(pO),float(aO),float(tO)]
    }
    print(f"  B0:{len(pB0)}步/{aB0:.4f} B1:{len(pB1)}步/{aB1:.4f} Ours:{len(pO)}步/{aO:.4f}")

    # === 实验3: 流线验证 ===
    print("\n[实验3] 流线验证...")
    rng = np.random.RandomState(123)
    apts, adirs = [], []
    for _ in range(15):
        sr, sc = rng.randint(int(rows*.1),int(rows*.9)), rng.randint(int(cols*.1),int(cols*.9))
        for __ in range(40):
            if not(0<=sr<rows and 0<=sc<cols): break
            apts.append((sr,sc)); adirs.append(aspect[sr,sc])
            pl, ql = p[sr,sc], q[sr,sc]
            mg = np.sqrt(pl**2+ql**2)
            if mg<1e-8: break
            sr, sc = int(sr+ql/mg), int(sc+pl/mg)
    rpts, rdirs = np.array(apts), np.array(adirs)

    def dcost(cf, pt, th):
        ad = np.abs(thetas-th); ad = np.minimum(ad, 2*np.pi-ad)
        return cf[int(pt[0]),int(pt[1]),np.argmin(ad)]

    cO_r = np.array([dcost(cO,pt,th) for pt,th in zip(rpts,rdirs)])
    cB0_r = np.array([cB0[int(pt[0]),int(pt[1]),0] for pt in rpts])
    cB1_r = np.array([dcost(cB1,pt,th) for pt,th in zip(rpts,rdirs)])
    nO,_ = neg_samples(rpts,rdirs,dem.shape,cO,thetas,cs)
    nB0,_ = neg_samples(rpts,rdirs,dem.shape,se,thetas,cs)
    nB1,_ = neg_samples(rpts,rdirs,dem.shape,cB1,thetas,cs)

    auO, auB0, auB1 = auc(cO_r,nO), auc(cB0_r,nB0), auc(cB1_r,nB1)
    dcO = dc(rpts,rdirs,cO,thetas)
    dcB1 = dc(rpts,rdirs,cB1,thetas)
    n = min(len(cO_r),len(nO))
    delta = nO[:n]-cO_r[:n]

    try:
        from scipy.stats import wilcoxon
        _,wp = wilcoxon(delta, alternative='greater'); wp = float(wp)
    except: wp = None

    RESULTS['exp3'] = {
        'n_pts':int(len(rpts)),
        'auc':[float(auB0),float(auB1),float(auO)],
        'dc':[float(dcB1),float(dcO)],
        'delta_mean':float(np.mean(delta)),
        'delta_median':float(np.median(delta)),
        'wilcoxon_p':wp
    }
    print(f"  AUC: B0={auB0:.4f} B1={auB1:.4f} Ours={auO:.4f}")
    print(f"  DC: B1={dcB1:.4f} Ours={dcO:.4f} ΔC_mean={np.mean(delta):.4f}")

    # === 实验4: 参数敏感性 ===
    print("\n[实验4] 参数敏感性...")
    def sens(param, vals, fixed):
        res = {}
        for v in vals:
            kw = dict(fixed); kw[param] = v
            _,cf = compute_impedance_field(p,q,thetas,cs, **kw)
            a = ai(cf)
            res[str(v)] = {'mc':float(cf.mean()), 'mx':float(cf.max()), 'ai':float(a.mean())}
            print(f"  {param}={v}: mean_cost={cf.mean():.4f} AI={a.mean():.4f}")
        return res

    RESULTS['exp4'] = {
        'alpha_u': sens('alpha_u', [5,10,15,20,30], dict(alpha_m=5,alpha_d=15,alpha_r=10)),
        'alpha_m': sens('alpha_m', [0,3,5,8,12], dict(alpha_u=15,alpha_d=15,alpha_r=10)),
        'alpha_d': sens('alpha_d', [5,10,15,20,30], dict(alpha_u=15,alpha_m=5,alpha_r=10)),
        'alpha_r': sens('alpha_r', [5,8,10,15,20], dict(alpha_u=15,alpha_m=5,alpha_d=15)),
    }

    # === 可视化 ===
    print("\n[可视化] 生成图表...")
    suffix = '_real' if src=='real' else ''

    # 图1: 函数形态
    gr = np.linspace(-0.6,0.6,500)
    Rp = longitudinal_impedance(gr,15,5,15)
    Rc = cross_slope_impedance(np.abs(gr),10)
    Rs = scalar_slope_impedance(np.arctan(np.abs(gr)),15)
    fig,ax = plt.subplots(1,3,figsize=(18,5))
    ax[0].plot(np.degrees(np.arctan(gr)),Rp,'b-',lw=2)
    ax[0].axvline(15,color='r',ls=':',label='α_u=15°')
    ax[0].axvline(-5,color='g',ls=':',label='α_m=5°')
    ax[0].set_xlabel('纵坡角(°)'); ax[0].set_ylabel('R_∥')
    ax[0].set_title('纵坡阻碍度（三段式）'); ax[0].legend(); ax[0].grid(True,alpha=.3)
    ax[1].plot(np.degrees(np.arctan(np.linspace(0,.6,500))),'r-',lw=2)
    ax[1].plot(np.linspace(0,.6,500), Rc, 'r-', lw=2)
    ax[1].axvline(10,color='r',ls=':',label='α_r=10°')
    ax[1].set_xlabel('横坡角(°)'); ax[1].set_ylabel('R_⊥')
    ax[1].set_title('横坡阻碍度'); ax[1].legend(); ax[1].grid(True,alpha=.3)
    # 修复：重写横坡图
    ax[1].clear()
    gp = np.linspace(0,0.6,500)
    Rc2 = cross_slope_impedance(gp,10)
    ax[1].plot(np.degrees(np.arctan(gp)),Rc2,'r-',lw=2)
    ax[1].axvline(10,color='r',ls=':',label='α_r=10°')
    ax[1].axhline(0.632,color='gray',ls='--',alpha=.5,label='R=0.632(1-1/e)')
    ax[1].set_xlabel('横坡角(°)'); ax[1].set_ylabel('R_⊥')
    ax[1].set_title('横坡阻碍度'); ax[1].legend(); ax[1].grid(True,alpha=.3)
    sl = np.linspace(0,np.radians(45),500)
    ax[2].plot(np.degrees(sl),Rs,'g-',lw=2)
    ax[2].axvline(15,color='r',ls=':',label='α_u=15°')
    ax[2].set_xlabel('坡度角(°)'); ax[2].set_ylabel('R_scalar')
    ax[2].set_title('标量坡度阻碍度(B0)'); ax[2].legend(); ax[2].grid(True,alpha=.3)
    fig.suptitle('阻碍度函数设计',fontsize=15)
    fig.tight_layout(); fig.savefig(os.path.join(OUT,f'func_shapes{suffix}.png'),dpi=150,bbox_inches='tight')
    plt.close(fig); print("  func_shapes.png")

    # 图2: 非补偿合成
    fig,ax = plt.subplots(figsize=(8,6))
    rp = np.linspace(0,1,100); RP,RC = np.meshgrid(rp,rp)
    Rcomb = 1-(1-RP)*(1-RC)
    im=ax.contourf(RP,RC,Rcomb,levels=20,cmap='RdYlGn_r')
    plt.colorbar(im,label='R_slope')
    cs2=ax.contour(RP,RC,Rcomb,levels=[.1,.3,.5,.7,.9],colors='k',linewidths=.8)
    ax.clabel(cs2,fontsize=9)
    ax.set_xlabel('R_∥'); ax.set_ylabel('R_⊥')
    ax.set_title('非补偿合成: R=1-(1-R_∥)(1-R_⊥)')
    fig.tight_layout(); fig.savefig(os.path.join(OUT,f'noncomp{suffix}.png'),dpi=150,bbox_inches='tight')
    plt.close(fig); print("  noncomp.png")

    # 图3: 三模型路径
    minB0,minB1,minO = cB0[:,:,0], np.min(cB1,2), np.min(cO,2)
    fig,axes = plt.subplots(1,3,figsize=(18,5))
    for ax_,mat,t,path in zip(axes,[minB0,minB1,minO],
        ['B0:标量','B1:仅纵坡','Ours:纵坡+横坡'],[pB0,pB1,pO]):
        ax_.imshow(mat,cmap='RdYlGn_r',origin='lower')
        if len(path)>0:
            pa=np.array(path); ax_.plot(pa[:,1],pa[:,0],'b-',lw=2)
        ax_.plot(start[1],start[0],'go',ms=10); ax_.plot(goal[1],goal[0],'r*',ms=12)
        ax_.set_title(t)
    fig.suptitle('三模型路径对比',fontsize=14)
    fig.tight_layout(); fig.savefig(os.path.join(OUT,f'paths{suffix}.png'),dpi=150,bbox_inches='tight')
    plt.close(fig); print("  paths.png")

    # 图4: 代价玫瑰图
    fig,axes = plt.subplots(2,3,figsize=(16,10),subplot_kw={'projection':'polar'})
    ps = [(int(rows*.25),int(cols*.25)),(int(rows*.5),int(cols*.5)),(int(rows*.75),int(cols*.75))]
    ls = ['西南','中心','东北']
    tf = np.append(thetas,thetas[0])
    for i,(pos,label) in enumerate(zip(ps,ls)):
        r_,c_ = pos
        cb = cB1[r_,c_,:]; cf_= np.append(cb,cb[0])
        axes[0,i].fill(tf,cf_,alpha=.3,color='orange'); axes[0,i].plot(tf,cf_,'o-',color='orange',ms=3)
        axes[0,i].set_title(f'{label}\n(B1)',fontsize=10,pad=15)
        axes[0,i].set_theta_zero_location('N'); axes[0,i].set_theta_direction(-1)
        co_ = cO[r_,c_,:]; cf2=np.append(co_,co_[0])
        axes[1,i].fill(tf,cf2,alpha=.3,color='steelblue'); axes[1,i].plot(tf,cf2,'o-',color='steelblue',ms=3)
        axes[1,i].set_title(f'{label}\n(Ours)',fontsize=10,pad=15)
        axes[1,i].set_theta_zero_location('N'); axes[1,i].set_theta_direction(-1)
    fig.suptitle('代价玫瑰图对比',fontsize=14,y=1.02)
    fig.savefig(os.path.join(OUT,f'roses{suffix}.png'),dpi=150,bbox_inches='tight')
    plt.close(fig); print("  roses.png")

    # 图5: AI对比
    fig,axes = plt.subplots(1,2,figsize=(14,5))
    axes[0].imshow(aiB1,cmap='hot',origin='lower',vmin=0,vmax=1)
    axes[0].set_title('B1 AI'); plt.colorbar(axes[0].images[0],ax=axes[0],shrink=.8)
    axes[1].imshow(aiO,cmap='hot',origin='lower',vmin=0,vmax=1)
    axes[1].set_title('Ours AI'); plt.colorbar(axes[1].images[0],ax=axes[1],shrink=.8)
    fig.suptitle('各向异性指数对比',fontsize=14)
    fig.tight_layout(); fig.savefig(os.path.join(OUT,f'ai{suffix}.png'),dpi=150,bbox_inches='tight')
    plt.close(fig); print("  ai.png")

    # 图6: 纵坡/横坡分解
    fig,axes = plt.subplots(1,3,figsize=(18,5))
    th45 = np.pi/4
    gp45,gx45 = compute_longitudinal_cross_slope(p,q,th45)
    axes[0].imshow(np.degrees(slope),cmap='YlOrRd',origin='lower')
    axes[0].set_title('坡度(°)'); plt.colorbar(axes[0].images[0],ax=axes[0],shrink=.8)
    axes[1].imshow(np.degrees(gp45),cmap='RdBu_r',origin='lower')
    axes[1].set_title(f'纵坡g_∥(θ=45°)'); plt.colorbar(axes[1].images[0],ax=axes[1],shrink=.8)
    axes[2].imshow(np.degrees(np.abs(gx45)),cmap='YlOrRd',origin='lower')
    axes[2].set_title(f'|横坡g_⊥|(θ=45°)'); plt.colorbar(axes[2].images[0],ax=axes[2],shrink=.8)
    fig.suptitle('纵坡/横坡分解',fontsize=14)
    fig.tight_layout(); fig.savefig(os.path.join(OUT,f'decomp{suffix}.png'),dpi=150,bbox_inches='tight')
    plt.close(fig); print("  decomp.png")

    # 图7: 敏感性分析
    fig,axes = plt.subplots(2,2,figsize=(14,10))
    for ax_,param,label in zip(axes.flat,
        ['alpha_u','alpha_m','alpha_r','alpha_d'],
        ['α_u(上坡尺度角)','α_m(缓下坡容许角)','α_r(横坡尺度角)','α_d(陡下坡尺度角)']):
        data_ = RESULTS['exp4'][param]
        keys = sorted(data_.keys(), key=float)
        ax_.plot([float(k) for k in keys],[data_[k]['mc'] for k in keys],'bo-',lw=2,ms=6)
        ax_.set_xlabel(label); ax_.set_ylabel('平均代价')
        ax_.set_title(f'{label} 敏感性'); ax_.grid(True,alpha=.3)
        ax2_ = ax_.twinx()
        ax2_.plot([float(k) for k in keys],[data_[k]['ai'] for k in keys],'rs--',lw=2,ms=6)
        ax2_.set_ylabel('AI')
    fig.suptitle('参数敏感性分析',fontsize=15)
    fig.tight_layout(); fig.savefig(os.path.join(OUT,f'sensitivity{suffix}.png'),dpi=150,bbox_inches='tight')
    plt.close(fig); print("  sensitivity.png")

    # 保存JSON
    with open(os.path.join(OUT, 'experiment_results.json'), 'w', encoding='utf-8') as f:
        json.dump(RESULTS, f, ensure_ascii=False, indent=2)
    print("\n  已保存: experiment_results.json")
    print("="*60)
    print("全部实验完成！")

if __name__ == '__main__':
    main()
