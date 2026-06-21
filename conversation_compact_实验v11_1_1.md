# 会话压缩记录：越野通行能力建模 v10 → v11.1.1

生成时间：2026-06-20  
用途：保存本轮对话核心内容，便于后续新会话继续推进。  
当前主题：基于 DEM/GIS 数据的方向相关越野通行能力建模、实验设计、代码开发与论文级审阅。

---

## 1. 用户研究背景与固定约束

用户的博士研究方向被固定为：

> 基于地理信息数据的越野通行能力建模分析方法。核心是建立从地理环境要素到通行阻碍度，再到综合通行能力或通行代价图的量化建模框架。

具体研究内容包括：

1. 利用 DEM、土地覆盖分类数据、土壤属性数据、水系数据等地理信息数据；
2. 提取或构建坡度、地表粗糙度、地表覆盖类型、土壤承载力、水系障碍等通行影响因子；
3. 分别建立各要素对地面车辆通行能力的阻碍度量化模型；
4. 将多个单要素阻碍度综合为统一的栅格化通行能力指标；
5. 生成越野通行能力图或通行代价图；
6. 使用路径规划作为应用验证环节，验证通行能力图或代价图是否能支持合理越野路线选择。

实验条件约束：

- 只有一台电脑；
- 没有真实车辆实验数据；
- 数据源主要是 DEM、土地覆盖、土壤、水系等 GIS 数据；
- 教研室方向偏 GIS、地理/遥感数据处理；
- 导师更倾向“数学建模与工程应用”，而不是实车动力学实验。

导师提出过的重要想法：

- 坡度分析不能只看坡度大小；
- 应考虑等高线、山脊线、山谷线、曲率等地形结构；
- 等高线方向、山脊/山谷方向可能更容易形成自然通道；
- 通行能力图不应只是静态标量图，而应考虑行驶方向；
- 综合方法需要创新，不能只是简单加权；
- 评价方法需要客观验证，例如使用 OSM 山路/林道/track/path 数据弱监督验证。

---

## 2. 已上传和处理过的主要文件

本轮对话涉及以下文件或代码包：

1. `prototype_v10_spatial_adhesion_trafficability (2).zip`  
   - v10 实验代码包；
   - 包含真实 DEM 实验结果；
   - 包含 `outputs_v10_real_full/`；
   - 核心代码包括 `cost_model.py`、`vehicle_capability_model.py`、`spatial_surface_params.py`、`path_planning.py`、`path_metrics.py`、`run_v10_experiments.py`、`osm_validation.py`、`roi_selector.py` 等。

2. `v10实验_2026-06-19 18_20_53.md`  
   - 之前对话记录；
   - 包含研究内容、导师意见、实验设计讨论、文献整理提示词等内容。

3. `gemini01.txt`  
   - Gemini 专家对 v10 → v11 升级意见的审阅；
   - 重点支持“统一评价”和“硬约束可达性”两项升级；
   - 指出不同模型代价尺度不可直接比较。

4. `prototype_v11_hard_constraint_unified_evaluation.zip`  
   - v11 原型包；
   - 增加统一评价和硬约束 A* 的代码框架；
   - 但存在输出目录仍指向 `outputs_v10_real_full` 的工程问题。

5. `prototype_v11_1_1_clean_real_dem.zip` / `v11_1_1_clean_real_dem_download.zip`  
   - v11.1.1 清理版；
   - 目标是修复输出目录混乱、删除 synthetic demo 小方块、规范文件名、增强文档说明；
   - 下载时平台给文件名前自动加了很长前缀，但文件可下载。

---

## 3. v10 的核心算法思想

v10 的核心不是普通坡度代价图，而是建立方向相关车辆能力利用率代价场。

传统 GIS cost surface 通常写成：

\[
C(i)
\]

其中 \(i\) 是栅格位置。这隐含假设：同一个栅格无论从哪个方向通行，代价都一样。

v10 改为：

\[
C(i,d)
\]

其中：

- \(i\)：栅格位置；
- \(d\)：车辆行驶方向。

也就是说，同一格子从不同方向通过，代价不同。

v10 的核心代价函数为：

\[
C_{V10}(i,d)=1+\rho_{\max}(i,d)
\]

其中：

\[
\rho_{\max}(i,d)
=
\max
\left(
\rho_{\mathrm{up}},
\rho_{\mathrm{down}},
\rho_{\mathrm{roll}},
\rho_{\mathrm{slide}}
\right)
\]

四类车辆能力利用率分别是：

\[
\rho_{\mathrm{up}}
=
\frac{
\tan\left(\max(\alpha_{\parallel},0)\right)
}{
\tan(\alpha_{\mathrm{grade}})
}
\]

\[
\rho_{\mathrm{down}}
=
\frac{
\tan\left(\max(-\alpha_{\parallel},0)\right)
}{
\mu_b(i)
}
\]

\[
\rho_{\mathrm{roll}}
=
\frac{
\tan(\alpha_{\perp})
}{
B/(2h_c)
}
\]

\[
\rho_{\mathrm{slide}}
=
\frac{
\tan(\alpha_{\perp})
}{
\mu_s(i)
}
\]

变量含义：

- \(\alpha_{\parallel}\)：行驶方向纵坡；
- \(\alpha_{\perp}\)：车辆横坡；
- \(\alpha_{\mathrm{grade}}\)：车辆最大爬坡能力角；
- \(\mu_b(i)\)：空间化制动附着能力；
- \(\mu_s(i)\)：空间化侧滑附着能力；
- \(B\)：车辆轮距；
- \(h_c\)：车辆质心高度；
- \(B/(2h_c)\)：静态侧翻稳定能力；
- \(\rho_{\max}>1\)：至少一个车辆能力需求超过供给。

---

## 4. DEM 如何进入方向坡度模型

DEM 高程面为：

\[
z=z(x,y)
\]

DEM 梯度为：

\[
\nabla z_i=(p_i,q_i)
\]

车辆行驶方向单位向量为：

\[
\mathbf{u}_d=(\cos\theta_d,\sin\theta_d)
\]

横向单位向量为：

\[
\mathbf{n}_d=(-\sin\theta_d,\cos\theta_d)
\]

行驶方向纵坡正切为：

\[
g_{\parallel}(i,d)
=
\nabla z_i\cdot \mathbf{u}_d
=
p_i\cos\theta_d+q_i\sin\theta_d
\]

车辆横坡正切为：

\[
g_{\perp}(i,d)
=
\nabla z_i\cdot \mathbf{n}_d
=
-p_i\sin\theta_d+q_i\cos\theta_d
\]

转换为角度：

\[
\alpha_{\parallel}(i,d)
=
\arctan(g_{\parallel}(i,d))
\]

\[
\alpha_{\perp}(i,d)
=
\arctan(|g_{\perp}(i,d)|)
\]

设计原因：

1. 纵坡决定上坡牵引需求与下坡制动需求；
2. 横坡决定侧滑风险与侧翻风险；
3. 同一坡面沿等高线走时纵坡可能很小，但横坡可能很大；
4. 因此不能只用坡度大小 \(|\nabla z|\)，必须分解为方向纵坡和横坡。

---

## 5. B0、B1、V9、V10、V10-Hard 的含义

后续论文和图表中不能只写 B0/B1，必须写完整名称。

| 标识 | 完整名称 | 方法含义 | 实验作用 |
|---|---|---|---|
| B0 | 静态坡度代价模型 | 只使用坡度大小，不区分行驶方向 | 传统静态成本面基线 |
| B1 | 方向纵坡代价模型 | 只考虑行驶方向纵坡，不考虑横坡 | 验证只考虑方向纵坡是否会低估横坡风险 |
| V9 | 常数附着车辆能力模型 | 考虑上坡、下坡、侧翻、侧滑，但 \(\mu_s,\mu_b\) 为空间常数 | 车辆能力模型基线 |
| V10-Free | 空间附着车辆能力软约束模型 | \(\mu_s(i),\mu_b(i)\) 空间化，超限仍可通行但代价升高 | v10 主模型 |
| V10-Hard | 空间附着车辆能力硬约束模型 | 若 \(\rho_{\max}>\rho_{\lim}\)，该方向视为不可通行 | v11 可达性实验 |

理论递进关系：

\[
\text{B0 静态坡度}
\rightarrow
\text{B1 方向纵坡}
\rightarrow
\text{V9 车辆能力约束}
\rightarrow
\text{V10 空间附着}
\rightarrow
\text{V11 统一评价与硬约束}
\]

---

## 6. v10 已经证明了什么

v10 真实 DEM 实验结果中，方向代价统计显示：

\[
\overline{C}_{\min}=2.037
\]

\[
\overline{C}_{\max}=2.768
\]

\[
\overline{\Delta C}=0.730
\]

说明同一区域、同一栅格，因通行方向不同，平均代价差异约为 0.73。

这证明：

> 通行能力不是单一静态栅格属性，而是与车辆行驶方向相关的方向异质性场。

v10 路径实验中，B1 和 V10 的对比很关键：

\[
P_{B1}(\rho_{\max}>1)=0.824
\]

\[
P_{B1}(\rho_{\mathrm{slide}}>0.7)=0.861
\]

\[
\overline{\rho}_{\max,B1}=1.822
\]

而 V10 为：

\[
P_{V10}(\rho_{\max}>1)=0.555
\]

\[
P_{V10}(\rho_{\mathrm{slide}}>0.7)=0.475
\]

\[
\overline{\rho}_{\max,V10}=1.247
\]

这说明：

> 只考虑方向纵坡的 B1 会倾向于寻找纵坡小的路线，例如沿等高线方向通行，但这可能显著增加横坡侧滑风险。  
> V10 同时考虑纵坡、横坡、侧滑和制动附着，因此能更好揭示横坡风险。

v10 相比 V9 有一定改善：

\[
\overline{\rho}_{\max,V9}=1.315
\]

\[
\overline{\rho}_{\max,V10}=1.247
\]

\[
P_{V9}(\rho_{\max}>1)=0.585
\]

\[
P_{V10}(\rho_{\max}>1)=0.555
\]

说明空间化附着参数机制有效，但提升幅度不算压倒性。原因是当前真实 DEM 实验中，若没有真实土地覆盖、土壤、水系输入，代码会回退到 DEM/TWI 派生或合成参数。

严谨表述应为：

> V10 证明了空间化附着参数能够影响路径选择和车辆能力利用率，但在真实土地覆盖、土壤和水系完全接入前，还不能声称完整多源 GIS 融合已经被充分验证。

---

## 7. v11 相对 v10 的核心改动

v11 不是推翻 v10 公式，而是在 v10 基础上增加两个论文级关键实验机制。

### 7.1 统一评价 Unified Evaluation

问题：

B0/B1 的代价来自经验阻碍度：

\[
C_{B0/B1}=1-\ln(1-R)
\]

V9/V10 的代价来自车辆能力利用率：

\[
C_{V9/V10}=1+\rho_{\max}
\]

因此不同模型自己的累计规划代价不可直接比较：

\[
J_{B0} \not\sim J_{V10}
\]

v11 解决方法：

先让每个模型用自己的代价函数规划路径：

\[
P_m^*
=
\arg\min_P
\sum_{e_k\in P}
C_m(e_k)l(e_k)
\]

然后把所有路径统一放回 V10 车辆能力模型中重新评价：

\[
E_{V10}(P_m)
=
\left\{
\overline{\rho}_{\max},
P(\rho_{\max}>1),
P(\rho_{\mathrm{slide}}>0.7),
P(\rho_{\mathrm{down}}>0.7),
L,
D
\right\}
\]

意义：

> 不再比较不同模型自身代价大小，而是在同一车辆能力体系下比较路径质量。

---

### 7.2 硬约束可达性 Hard Constraint Reachability

v10 是软约束：

\[
C(i,d)=1+\rho_{\max}(i,d)
\]

即使：

\[
\rho_{\max}>1
\]

算法仍允许通过，只是代价升高。

v11 增加硬约束：

\[
C(i,d)=
\begin{cases}
1+\rho_{\max}(i,d), & \rho_{\max}(i,d)\leq \rho_{\lim} \\
+\infty, & \rho_{\max}(i,d)>\rho_{\lim}
\end{cases}
\]

其中：

- \(\rho_{\lim}=1.0\)：严格不允许超过车辆能力边界；
- \(\rho_{\lim}=1.2\)：允许轻微超限；
- \(\rho_{\lim}=1.5\)：允许更高风险压力。

意义：

> v10 回答“哪条路相对风险较小”，v11 进一步回答“在给定车辆能力阈值下是否存在可行路径”。

---

## 8. “路径约束利用率统计实验”是什么

`04_路径约束利用率统计实验` 是 v10/v11 中非常重要的路径实验模块。它不是简单画很多重复图，而是在做：

\[
\text{Path}
\rightarrow
\text{Vehicle capability utilization profile}
\rightarrow
\text{Risk statistics}
\]

每条路径沿途计算：

- \(\rho_{\mathrm{up}}\)：上坡牵引利用率；
- \(\rho_{\mathrm{down}}\)：下坡制动利用率；
- \(\rho_{\mathrm{roll}}\)：侧翻稳定性利用率；
- \(\rho_{\mathrm{slide}}\)：侧滑附着利用率；
- \(\rho_{\max}\)：主导能力利用率。

统计指标包括：

\[
\overline{\rho}_{\max}
\]

\[
P(\rho_{\max}>1)
\]

\[
P(\rho_{\mathrm{slide}}>0.7)
\]

\[
P(\rho_{\mathrm{down}}>0.7)
\]

\[
L(P)
\]

\[
\text{Detour}(P)
\]

之所以有 24 个 pair，是因为实验使用多个 ROI、多种起终点场景来避免单一路径偶然性。场景包括：

- `contour_cross_slope`：等高线/横坡穿越；
- `upslope_downslope`：上坡/下坡；
- `diagonal_ne_sw`：东北—西南斜向；
- `diagonal_nw_se`：西北—东南斜向；
- `random_far_1`、`random_far_2`：远距离随机起终点。

每组起终点跑 B0、B1、V9、V10 等模型。因此 24 组 pair 不是异常，而是为了稳健性验证。

建议后续改名为：

```text
04_unified_path_capability_evaluation
```

或：

```text
04_multi_pair_vehicle_capability_profiles
```

---

## 9. 图中“小白方块”的原因

用户发现真实 DEM 路径图中存在一个不和谐的小白方块。经检查，原因不是 DEM 地物，也不是路径规划结果，而是代码中的 synthetic landcover fallback 残留。

逻辑大致为：

```python
if landcover is None:
    generate_synthetic_landcover(...)
```

其中曾存在人为加入的 demo built-up patch / hard barrier patch：

```python
# small built-up patch as hard barrier demonstration
if rows > 80 and cols > 80:
    rr = slice(rows//2-8, rows//2+8)
    cc = slice(cols//2-8, cols//2+8)
    lc[rr, cc] = 50
```

类别 50 被配置为建成区或 hard barrier，绘图时无效/不可通行区域被渲染成白色，于是出现小白方块。

结论：

> 小白方块不是地理现象，而是合成土地覆盖测试残留。真实 DEM 实验中必须禁止这种 synthetic demo barrier。

v11.1.1 的修复目标：

1. 真实 DEM 模式下禁止自动注入 demo hard patch；
2. synthetic 测试和 real DEM 实验严格分离；
3. 输出图中不再出现人为小白块。

---

## 10. 文件乱码与链接问题

### 10.1 文件名乱码

用户截图中部分中文文件名乱码，例如：

```text
01_ - 默.md
02_ - O入订感_md
```

可能原因：

1. Windows 与 zip 文件名编码兼容问题；
2. 压缩包未正确写入 UTF-8 文件名标志；
3. 旧输出目录中本身已有 mojibake 文件名；
4. Python/系统环境对中文路径处理不一致。

后续规范：

- 代码输出目录和文件名全部使用英文安全命名；
- 中文解释写入文件内容、README、CSV 表头或图注；
- 打包时使用 UTF-8；
- 不再沿用旧乱码输出文件夹。

建议目录结构：

```text
00_docs/
01_vehicle_params/
02_surface_adhesion/
03_directional_cost/
04_unified_path_evaluation/
05_hard_constraint_reachability/
06_wetness_scenarios/
07_osm_weak_validation/
```

### 10.2 下载链接问题

之前尝试过 HTML 形式的链接：

```html
<a href="sandbox:/mnt/data/...">...</a>
```

但聊天界面不渲染 HTML，导致用户看到代码而不是链接。之后改为标准 Markdown 链接。

标准格式应为：

```markdown
[下载 v11_1_1_clean_real_dem_download.zip](sandbox:/mnt/data/v11_1_1_clean_real_dem_download.zip)
```

最后可下载文件为：

```text
v11_1_1_clean_real_dem_download.zip
```

下载时平台自动在文件名前加了很长前缀，例如：

```text
saasnexus-..._mnt_data_v11_1_1_clean_real_dem_download.zip
```

这是平台附件系统生成的前缀，不代表文件内容错误。

---

## 11. v11 / v11.1.1 需要修复的工程问题

用户明确指出：以后不能再“理论说完、代码没测好、输出路径没改”就发包。后续必须每一步检查、测试、返工后再发。

已确认或需修复的问题：

1. v11 输出目录仍使用 `outputs_v10_real_full`，导致 v10/v11 混淆；
2. 真实 DEM 图中存在 synthetic hard barrier 小白方块；
3. 文件名乱码；
4. 链接文本和正文不够明显；
5. 链接显示文件名和实际下载文件名不一致；
6. 部分 LaTeX 公式在对话中渲染不正确；
7. 文档中的公式还需全面检查；
8. 每个公式都需要说明：
   - 从什么问题来；
   - 为什么要这样设计；
   - 变量含义；
   - 参数实际解决什么问题；
   - 公式输出怎么解释；
   - 结果如何验证。

v11.1.1 的目标：

```text
prototype_v11_1_1_clean_real_dem/
```

应完成：

1. 输出目录版本锁死为 v11；
2. 禁止 real DEM 模式中的 synthetic demo barrier；
3. 文件名英文安全化；
4. 文档中说明 v10/v11/v11.1.1 差异；
5. 公式文档使用可读 LaTeX；
6. 统一评价和硬约束实验逻辑保留；
7. 下载包可正常解压。

---

## 12. 当前理论和实验要解决的现实问题

### 问题 A：传统静态坡度图不能表达方向

传统模型：

\[
C(i)
\]

v10 改为：

\[
C(i,d)
\]

解决：

> 同一地形单元在不同通行方向下车辆能力需求不同。

---

### 问题 B：只考虑纵坡会低估横坡风险

B1 只看：

\[
\alpha_{\parallel}
\]

但忽略：

\[
\alpha_{\perp}
\]

实验显示 B1 的侧滑高风险比例很高，说明：

> 沿等高线方向不一定安全，因为横坡可能导致侧滑或侧翻。

---

### 问题 C：地表类型和湿润条件不能只作为普通权重

V10 使用：

\[
\mu_s(i)
\]

\[
\mu_b(i)
\]

将土地覆盖、TWI、水系、土壤软弱倾向转化为空间附着能力，解决：

> GIS 地表因子如何进入车辆能力约束模型。

---

### 问题 D：不同模型规划代价不可直接比较

v11 通过统一评价解决：

\[
J_{B0}\not\sim J_{V10}
\]

改为比较：

\[
E_{V10}(P_{B0}),
E_{V10}(P_{B1}),
E_{V10}(P_{V9}),
E_{V10}(P_{V10})
\]

解决：

> 不同代价尺度下路径结果如何公平评价。

---

### 问题 E：通行能力图不仅要排序，还要判断能不能走

v11 增加硬约束：

\[
\rho_{\max}(i,d)\leq \rho_{\lim}
\]

解决：

> 在给定车辆能力边界下，是否存在可行路径。

---

## 13. 后续回答和开发的固定要求

用户明确要求：

1. 以后涉及理论、算法、实验设计、文献依据、审稿风险时，要联网查阅资料并精读引证；
2. 不能只凭已有对话内容推进；
3. 回答中公式必须尽量使用标准 LaTeX，不要写进 plain text；
4. 对话和文档中的每个公式都要解释来源、设计原因、参数意义和验证方式；
5. 代码包必须检查后再发：
   - 路径存在；
   - 文件大小合理；
   - 压缩包可打开；
   - 输出目录正确；
   - 不混用 v10/v11；
   - 不存在明显 synthetic artifact；
   - 下载链接可用；
   - 文件名版本号明确。
6. 下载链接要单独醒目展示，建议格式：

```markdown
**⬇️ [下载：v11_1_1_clean_real_dem_download.zip](sandbox:/mnt/data/v11_1_1_clean_real_dem_download.zip)**
```

7. 文档和文件名应明确版本号，例如：
   - `v11.1.1`
   - `v11_1_1_clean_real_dem_download.zip`

---

## 14. 下一步建议

建议下一步推进顺序：

### 第一步：彻底检查 v11.1.1 代码包

检查内容：

1. `run_v11_experiments.py` 是否默认输出到 `outputs_v11_real_full` 或等价 v11 目录；
2. 是否仍有 `outputs_v10_real_full` 硬编码；
3. `spatial_surface_params.py` 是否仍生成 demo built-up patch；
4. real DEM 模式是否禁用 synthetic hard barrier；
5. 文档公式是否渲染正常；
6. 输出文件名是否英文安全；
7. 压缩包是否可解压、可运行。

### 第二步：真实 DEM 全量重跑

输出目录建议：

```text
outputs_v11_1_1_real_full/
```

模型包括：

- B0 static slope；
- B1 directional longitudinal slope；
- V9 constant adhesion capability；
- V10 spatial adhesion free；
- V10 hard \(\rho_{\lim}=1.0\)；
- V10 hard \(\rho_{\lim}=1.2\)；
- V10 hard \(\rho_{\lim}=1.5\)。

### 第三步：补 OSM 弱监督验证

使用 OSM 山路/track/path 作为弱监督正样本，非道路背景样本作为负样本。

指标：

\[
AUC=P(C_{\mathrm{road}}<C_{\mathrm{nonroad}})
\]

\[
\Delta C=
\mathbb{E}[C_{\mathrm{nonroad}}]
-
\mathbb{E}[C_{\mathrm{road}}]
\]

结论边界：

> OSM 只能作为现实道路选线倾向的弱监督参照，不能作为绝对车辆通行真值。

### 第四步：整理论文方法章节

应按以下结构写：

1. 研究问题；
2. 现有方法不足；
3. DEM 方向坡度分解；
4. 车辆能力利用率模型；
5. 空间化附着参数；
6. 统一评价；
7. 硬约束可达性；
8. 实验设计；
9. 结果分析；
10. 结论边界。

---

## 15. 当前状态一句话总结

截至本轮对话末尾，整体状态为：

> v10 已经建立了方向相关车辆能力利用率代价场，并在真实 DEM 中初步证明方向异质性和横坡风险的重要性；v11 在此基础上加入统一评价和硬约束可达性，理论方向正确，但工程实现曾存在输出目录混用、synthetic 小白方块、文件名乱码、下载链接不稳定等问题；v11.1.1 的目标是清理这些工程问题，并为后续真实 DEM 全量重跑、OSM 弱监督验证和论文方法章节撰写做准备。

