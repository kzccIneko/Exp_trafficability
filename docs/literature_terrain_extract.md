# 越野通行能力建模文献综述 — 公式与方法提取

> 本文档系统提取了 `DEM 越野路径规划 通行能力分析\地貌分类、栅格代价\` 文件夹中所有PDF文献与"越野通行能力建模"相关的完整公式、推导过程、方法描述和关键数据。

---

## 第1章 Zevenbergen & Thorne (1987) — 地形表面定量分析

**文献**: Zevenbergen L.W., Thorne C.R. "Quantitative analysis of land surface topography" *Earth Surface Processes and Landforms*, 1987.

### 1.1 局部曲面拟合法

采用9个网格点（3×3窗口）的高程数据，通过**偏四次多项式**（partial quartic surface）拟合地表：

$$z = Ax^2y^2 + Bx^2y + Cxy^2 + Dx^2 + Ey^2 + Fxy + Gx + Hy + I$$

其中系数与3×3高程矩阵的关系（设网格间距为 $\Delta$）：

$$I = z_5$$

$$G = \frac{z_4 - z_6}{2\Delta}$$

$$H = \frac{z_2 - z_8}{2\Delta}$$

$$D = \frac{z_4 + z_6 - 2z_5}{2\Delta^2}$$

$$E = \frac{z_2 + z_8 - 2z_5}{2\Delta^2}$$

$$F = \frac{(z_1 - z_3 - z_7 + z_9)}{4\Delta^2}$$

$$A = \frac{(z_1 + z_3 + z_7 + z_9) - 2(z_2 + z_4 + z_6 + z_8) + 4z_5}{4\Delta^4}$$

$$B = \frac{(z_1 + z_3 - z_7 - z_9) - 2(z_2 - z_8)}{4\Delta^3}$$

$$C = \frac{(-z_1 + z_3 - z_7 + z_9) - 2(z_4 - z_6)}{4\Delta^3}$$

### 1.2 坡度（Slope）

$$\text{SLOPE} = -\sqrt{G^2 + H^2}$$

$$\text{SLOPE} = \arctan\sqrt{G^2 + H^2} \quad \text{（以度为单位时）}$$

### 1.3 坡向（Aspect）

$$\theta = \arctan\left(\frac{-H}{-G}\right)$$

### 1.4 曲率计算

**剖面曲率（Profile Curvature）**：沿最大坡度方向的曲率

$$\text{PROFC} = -2(D\cos^2\theta + E\sin^2\theta + F\cos\theta\sin\theta)$$

$$= \frac{-2(DG^2 + EH^2 + FGH)}{G^2 + H^2}$$

**平面曲率（Planform Curvature）**：垂直于最大坡度方向的曲率

$$\text{PLANC} = 2(D\sin^2\theta + E\cos^2\theta - F\sin\theta\cos\theta)$$

$$= \frac{2(DH^2 + EG^2 - FGH)}{G^2 + H^2}$$

**数学曲率（最大曲率方向）**：

$$K = \frac{\partial^2 z / \partial S^2}{\left[1 + (\partial z / \partial S)^2\right]^{3/2}}$$

**任意方向 $\phi$ 的曲率**：

$$\kappa(\phi) = \frac{2(D\cos^2\phi + E\sin^2\phi + F\sin\phi\cos\phi)}{G^2 + H^2}$$

### 1.5 方法特点
- 使用偏四次多项式而非二次多项式，可更好表示复杂地形
- 所有二阶导数系数（D, E, F）可通过3×3窗口直接计算
- 坡度、坡向、曲率均可用解析公式表达，无需迭代

---

## 第2章 Minař et al. (2020) — 地表曲率综合体系

**文献**: Minař J. et al. "Land surface curvatures as a key morphometric variable" *Earth-Science Reviews*, 2020.

### 2.1 不变曲率（与重力方向无关）

**最大主曲率** $k_{\max}$ 和 **最小主曲率** $k_{\min}$：

$$k_{\max} = k_1, \quad k_{\min} = k_2 \quad (k_1 \geq k_2)$$

**平均曲率**：

$$k_{\text{mean}} = \frac{k_{\max} + k_{\min}}{2}$$

**高斯曲率**：

$$K = k_{\max} \cdot k_{\min}$$

**Casorati曲率**：

$$k_C = \sqrt{\frac{k_{\max}^2 + k_{\min}^2}{2}}$$

**非球度（unsphericity）**：

$$k_u = k_{\max} - k_{\min}$$

### 2.2 与重力相关的曲率

**剖面曲率**（沿坡度方向）：$(k_n)_s$，由Zevenbergen & Thorne的PROFC定义。

**平面曲率**（垂直坡度方向）：$(k_n)_c$，由Zevenbergen & Thorne的PLANC定义。

**差值曲率**：

$$k_d = (k_n)_s - (k_n)_c$$

**总累积曲率**：

$$K_a = (k_n)_s + (k_n)_c$$

### 2.3 方向导数

Zevenbergen & Thorne的方向导数：

$$z_{ss} = 2(D\cos^2\theta + E\sin^2\theta + F\cos\theta\sin\theta)$$

$$z_{cc} = 2(D\sin^2\theta + E\cos^2\theta - F\sin\theta\cos\theta)$$

### 2.4 潜在能量面（PES）

$$\text{PES} = \rho \cdot g \cdot (k_n)_s$$

其中 $\rho$ 为物质密度，$g$ 为重力加速度。

### 2.5 GIS精度测试函数

用于评估GIS中曲率计算的精度：

$$z = \sin(X) \cdot \cos(Y)$$

该函数具有已知的解析曲率值，可用于验证数值计算的准确性。

---

## 第3章 Suvinen (2006) — 基于GIS的地形可操控性模拟模型

**文献**: Suvinen A. "A GIS-based simulation model for terrain tractability" *Journal of Terramechanics*, 2006.

### 3.1 车轮载荷计算

**前轴载荷**（每轮）：

$$W_f = \frac{0.6 \times M_T}{n_W} \times g$$

**后轴载荷**（每轮）：

$$W_r = \frac{(0.4 \times M_T + M_L + M_B)}{n_W} \times g$$

其中 $M_T$ 为整车质量，$M_L$ 为载荷质量，$M_B$ 为车身质量，$n_W$ 为车轮数量。

### 3.2 轮胎变形与接地压力

**轮胎变形**：

$$\delta = 0.365 + \frac{170 \times W_i}{p_i / 1000}$$

**轮胎接地压力**：

$$q = \frac{b^{0.8} \times W_i}{d^{0.8} \times d^{0.4}}$$

**虚拟轮半径**：

$$R = \frac{d^2 + \delta^2}{2\delta}$$

**接地面积**：

$$\text{AREA} = \frac{W_i}{q}$$

### 3.3 坡度阻力

**坡度阻力**：

$$F_G = \mu_G \times W$$

其中坡度阻力系数：

$$\mu_G = \sin\alpha$$

**最大侧倾坡度**：

$$\alpha_{l,\max} = \frac{8.5}{\cos(90° - \omega)}$$

### 3.4 障碍物阻力

**障碍物阻力系数**：

$$\mu_O = \frac{\sum(k \times H_i)}{D}$$

其中 $k$ 为障碍物类型系数，$H_i$ 为障碍物高度，$D$ 为行驶距离。

### 3.5 积雪阻力

**低密度积雪**：

$$F_S = 35 \times h_S$$

**高密度积雪**：

$$F_S = 60 \times h_S$$

### 3.6 承载能力

$$W_{i,\max} = \pi \times r_{\text{theor}}^2 \times (1.3 \times c \times N_c + 0.6 \times c \times r_{\text{theor}} \times N_c)$$

承载能力因子 $N_c$（内摩擦角 $\varphi$ 的函数）：

$$N_c = 0.0488\varphi^3 - 3.6055\varphi^2 + 90.9482\varphi - 760.7648$$

### 3.7 滚动阻力

**滚动阻力**：

$$F_{Ri} = \mu_R \times W_i$$

**滚动阻力系数**：

$$\mu_R = 0.04 + \sqrt{\frac{z}{2 \times R \times z}}$$

**沉陷量**：

$$z = \frac{q \times b}{E}$$

### 3.8 冻结深度（Stefan公式）

$$z_{\text{frost}} = \sqrt{s \times F^-}$$

**融化深度**：

$$z_{\text{thaw}} = 1.1 \times \sqrt{F^+}$$

### 3.9 路径距离代价（PATHDISTANCE）

$$C_{\text{total}} = \sum_{i=1}^{n} \left( \frac{F_{S_i} + F_{G_i} + F_{R_i} + F_{O_i}}{v_i} \right) \times d_i$$

---

## 第4章 RCI（重塑锥指数）与VCI（车辆锥指数）

### 4.1 Knight (1961) RCI估算公式

$$\text{RCI} = \exp\left[a' - b' \ln(\text{MC})\right]$$

其中 MC 为含水量，$a'$ 和 $b'$ 为土壤参数系数。

### 4.2 Mason & Baylot (2016) RCI公式

$$\text{RCI} = e^{\left[a - b \ln\left(\frac{V_w}{V_s} \cdot \frac{r_w}{r_s}\right)\right]}$$

其中 $V_w$ 为水体积，$r_w$ 为水容重，$V_s$ 为土体积，$r_s$ 为土容重。

### 4.3 USDA土壤分类参数系数

| USDA土壤类型 | $a'$ | $b'$ |
|---|---|---|
| SC（含细粒砂土） | 5.17 | 0.87 |
| SP（级配不良砂） | 5.32 | 0.78 |
| OL（有机低液限土） | 5.05 | 1.15 |

### 4.4 FM5-33标准RCI参考值

| 土壤类别 | 干燥 | 湿润 | 饱和 |
|---|---|---|---|
| SC | 126 | 86 | 46 |
| SP | 145 | 109 | 73 |
| OL | 111 | 57 | 3 |

### 4.5 VCI判定准则

$$\text{若 RCI} > \text{VCI}_1 \Rightarrow \text{可通过}$$
$$\text{若 RCI} < \text{VCI}_{50} \Rightarrow \text{不可通过}$$

其中 $\text{VCI}_1$ 为1次通过所需最小RCI，$\text{VCI}_{50}$ 为50次通过所需最小RCI。

### 4.6 VCI与MI的关系

**机动性指数（MI）**：

$$\text{MI} = \frac{p \times \sqrt{P/A} + q}{b}$$

其中 $p$ 为接地压力，$P$ 为功率，$A$ 为接地面积，$q$ 为轮胎参数，$b$ 为经验系数。

---

## 第5章 He et al. (2023) — 定量规则法评估越野通行性

**文献**: He K. et al. "An assessment on the off-road trafficability using a quantitative rule method" *Computers & Geosciences*, 2023.

### 5.1 AHP-WIC方法

**加权信息量计算**：

$$I_{x,y} = \beta_1 I_{1s} + \beta_2 I_{2s} + \cdots + \beta_{10} I_{10s}$$

其中 $\beta_1 \sim \beta_{10}$ 为10个影响因素的AHP权重。

### 5.2 AHP判断矩阵与权重

| 因素 | 权重 |
|---|---|
| 坡度（Slope） | 0.1401 |
| 土地覆盖（LC） | 0.1273 |
| 土壤（Soil） | 0.0808 |
| 地形湿度指数（TWI） | 0.1043 |
| RCI | 0.0836 |
| **地质灾害（GH）** | **0.2807** |
| 地形位置指数（TPI） | 0.0672 |
| 水流功率指数（SPI） | 0.0303 |
| 高程（Elevation） | 0.0295 |
| 岩石（Rock） | 0.0204 |
| 节理构造（JS） | 0.0360 |

$\lambda_{\max} = 12.18$，$CI = 0.118$，$RI = 1.52$，$CR = 0.077 < 0.1$（通过一致性检验）。

### 5.3 信息量公式

$$I = \sum_{i=1}^{n} W_i \cdot I_i$$

### 5.4 坡度分级信息量

| 坡度范围 | 信息量 |
|---|---|
| 0°~7° | 9 |
| 7°~14° | 8 |
| 14°~21° | 6 |
| 21°~28° | 5 |
| 28°~35° | 3 |
| >35° | 1 |

### 5.5 通行性四分类

| 类别 | 含义 |
|---|---|
| Good Going (I) | 良好通行 |
| Restricted Going (II) | 受限通行 |
| Restricted Going with Engineering (III) | 需工程措施 |
| Difficult Going (IV) | 难以通行 |

### 5.6 改进分水岭滤波方法

当"良好"像素被"困难"像素包围时，该像素降级为"困难"。滤波规则：
- II类可过滤I类
- III类可过滤I、II类
- IV类可过滤I、II、III类

---

## 第6章 Pundir & Garg (2021) — 基于规则的越野通行性评估

**文献**: Pundir S.K., Garg R.D. "Development of rule based approach for assessment of off-road trafficability" *Quaternary International*, 2021.

### 6.1 方法框架

采用四级评估：Good (I)、Restricted (II)、Restricted with Engineering (III)、Difficult (IV)。

核心四因素：**土地利用（LC）**、**土壤类型（Soil）**、**坡度（Slope）**、**含水量（Moisture）**。

### 6.2 履带车辆规则表（Table 7）

| 土壤类型 | 含水量 | 0°~7° | 8°~14° | 15°~21° | 22°~28° | 29°~35° | >35° |
|---|---|---|---|---|---|---|---|
| 砂土 | 干燥 | I | I | I | III | III | IV |
| 砂土 | 湿润 | I | I | II | III | III | IV |
| 砂土 | 饱和 | III | III | III | IV | IV | IV |
| 粘土 | 干燥 | I | II | III | III | IV | IV |
| 粘土 | 湿润 | II | II | III | IV | IV | IV |
| 粘土 | 饱和 | II | III | IV | IV | IV | IV |
| 粉土 | 干燥 | I | I | II | III | IV | IV |
| 粉土 | 湿润 | II | II | III | IV | IV | IV |
| 粉土 | 饱和 | II | III | IV | IV | IV | IV |

### 6.3 轮式车辆规则表（Table 8）

轮式车辆的通行性整体低于履带车辆，特别在以下条件下差异显著：
- 砂土+干燥+15°~21°：履带I → 轮式II
- 粘土+干燥+0°~7°：履带I → 轮式I（相似）
- 砂土+湿润+0°~7°：履带I → 轮式I

### 6.4 USDA土壤质地分类

| 质地类别 | 砂(%) | 粉(%) | 粘(%) |
|---|---|---|---|
| 砂土 | 85~100 | 0~14 | 0~10 |
| 砂壤土 | 50~70 | 0~50 | 0~20 |
| 壤质砂土 | 70~86 | 0~30 | 0~15 |
| 壤土 | 23~52 | 28~50 | 7~27 |
| 粉壤土 | 20~50 | 74~88 | 0~27 |
| 粘壤土 | 20~45 | 15~52 | 27~40 |
| 粘土 | 0~45 | 0~40 | 40~100 |

---

## 第7章 Liu et al. (2025) — 多因素动态通行性地图构建

**文献**: Liu Q. et al. "Construction of dynamic trafficability map for unmanned vehicles" *Scientific Reports*, 2025.

### 7.1 通行性指标体系

综合考虑**11个因素**：
- 地理要素：高程、坡度、TPI、地貌、土地覆盖
- 地质要素：土壤、地质灾害
- 气象要素：降雨、降雪、风力、水平能见度

### 7.2 高程通行性计算

$$\frac{V_{0\text{-}1000}}{T_{0\text{-}1000}} = \frac{V_{\text{elevation range}}}{T_{\text{elevation range}}}$$

| 高程范围 | 轮式通行性 | 履带通行性 |
|---|---|---|
| 0~1000m | 1 | 1 |
| 1000~2000m | 0.886 | 0.967 |
| 2000~3000m | 0.781 | 0.926 |
| 3000~4000m | 0.685 | 0.817 |
| 4000~5000m | 0.598 | 0.71 |

### 7.3 坡度通行性计算

$$\frac{V_{3°\text{-}6°}}{S_{3°\text{-}6°}} = \frac{V_{\text{slope range}}}{S_{\text{slope range}}}$$

| 坡度范围 | 轮式 | 履带 |
|---|---|---|
| 0°~3° | 1 | 1 |
| 3°~6° | 0.9 | 0.9 |
| 6°~10° | 0.675 | 0.72 |
| 10°~15° | 0.54 | 0.6 |
| 15°~20° | 0.36 | 0.36 |
| 20°~30° | 0.225 | 0.24 |
| 30°~35° | 0 | 0.12 |
| >35° | 0 | 0 |

### 7.4 TPI计算

$$\text{TPI} = Z - \bar{Z}$$

| 地形位置 | 轮式/履带通行性 |
|---|---|
| 山脊（ridge） | 0.2 |
| 上坡（upper slope） | 0.1 |
| 中坡（middle slope） | 0.2 |
| 平地（flats） | 0.9 |
| 下坡（downhill） | 0.3 |
| 山谷（valleys） | 0.7 |

### 7.5 RCI土壤强度公式

$$\text{RCI} = e^{\left[a - b \ln\left(\frac{V_w}{V_s} \cdot \frac{r_w}{r_s}\right)\right]}$$

判定准则：RCI > VCI → 通行性=1；否则通行性=0。

### 7.6 地质灾害通行性

$$L_p = 1 - L_d$$

其中 $L_d$ 为地质灾害风险等级值。

### 7.7 气象因素影响系数

| 降雨(mm/24h) | 影响系数 | 通行性 |
|---|---|---|
| <10 | 0 | 1 |
| 10~25 | 0.2 | 0.8 |
| 25~50 | 0.4 | 0.6 |
| 50~100 | 0.5 | 0.5 |

| 降雪(mm/12h) | 影响系数 | 通行性 |
|---|---|---|
| 0.1~0.25 | 0.1 | 0.9 |
| 0.25~3 | 0.4 | 0.6 |
| 3~5 | 0.7 | 0.3 |
| >5 | 0.9 | 0.1 |

### 7.8 AHP-RCTM综合评估模型

$$P_i = W_1 P_1 + W_2 P_2 + \cdots + W_{11} P_{11}$$

AHP权重（CR = 0.0009 < 0.1）：

| 因素 | 权重 |
|---|---|
| 坡度 | 0.193 |
| 土地覆盖 | 0.193 |
| 降雨 | 0.193 |
| 降雪 | 0.069 |
| 风力 | 0.069 |
| 能见度 | 0.069 |
| 地貌 | 0.069 |
| 高程 | 0.036 |
| 地质灾害 | 0.036 |
| TPI | 0.036 |

### 7.9 改进A*算法（IA*）

**传统A*代价函数**：

$$f(n) = g(n) + h(n)$$

**IA*代价函数**（以通行性为代价）：

$$g(n) = \sum_{i=1}^{n} p_i$$

$$f(n) = \sum_{i=1}^{n} p_i - p_{\min} \cdot l_n$$

其中 $p_{\min}$ 为未遍历网格中最小通行性值，$l_n$ 为当前节点到目标节点的欧氏距离。

---

## 第8章 Zhao (2025) — 融合复杂地形与降雨的越野路径规划

**文献**: 赵德全. "融合复杂地形与降雨天气因素的越野车辆通行路径规划模型研究" 山东农业大学硕士论文, 2025.

### 8.1 DEM坡度计算（Sobel算子）

基于3×3窗口的高程数据：

$$\frac{dz}{dx} = \frac{[(c + 2f + i) \times w_1] - [(a + 2d + g) \times w_2]}{8 \times \text{cellsize}}$$

$$\frac{dz}{dy} = \frac{[(g + 2h + i) \times w_3] - [(a + 2b + c) \times w_4]}{8 \times \text{cellsize}}$$

$$\text{Slope} = \arctan\sqrt{\left(\frac{dz}{dx}\right)^2 + \left(\frac{dz}{dy}\right)^2}$$

### 8.2 坡度分级权重

| 坡度范围 | 权重 | 等级 |
|---|---|---|
| 0°~5° | 1 | 1 |
| 5°~10° | 0.8 | 2 |
| 10°~15° | 0.6 | 3 |
| 15°~20° | 0.4 | 4 |
| 20°~25° | 0.2 | 5 |
| 25°~31° | 0.1 | 6 |
| >31° | 0 | 7 |

### 8.3 Green-Ampt入渗模型

**累积入渗量**：

$$F = (K_s - K_i) \cdot t = K_s(\theta_s - \theta_i)(H_f + L)$$

**入渗率**：

$$i(t) = K_s \left(\frac{\theta_s - \theta_i}{F(t)} + 1\right)$$

**湿润锋深度**：

$$L(t) = \frac{F(t)}{\theta_s - \theta_i}$$

### 8.4 PI参数系统

$$\text{PI} = \frac{F(t)}{\theta_s(t)}$$

用于量化降雨条件下车辆通过能力的定量评估。

### 8.5 地物速度衰减模型

$$\eta = \frac{v_{\text{terrain}}}{v_0} \times 100\%$$

| 地物类型 | 速度衰减率 | 权重系数 |
|---|---|---|
| 裸地 | 0 | 1 |
| 自然草地 | 0.228 | 0.8 |
| 盐碱地 | 0.337 | 0.7 |
| 沙地 | 0.565 | 0.4 |
| 裸岩碎石 | 0.699 | 0.3 |

### 8.6 高斯风险场

$$p(x, y) = \frac{1}{\sqrt{(2\pi)^2 |\Sigma|}} \exp\left[-\frac{1}{2}(X - \mu_0)^T \Sigma^{-1} (X - \mu_0)\right]$$

### 8.7 改进A*启发式函数

$$f(n) = \alpha \cdot g(n) + \beta \cdot h(n)$$

最优参数：$\alpha = 0.4$，$\beta = 0.6$。

---

## 第9章 基于相对特征的越野地形可通行性分析

**文献**: 刘华军等. "基于相对特征的越野地形可通行性分析" 数据采集与处理, 2006.

### 9.1 地形相对不变性概念

在高程图上定义地形的相对不变性，提取**坡度（slope）**、**横滚角（roll）**和**粗糙度（roughness）**三个相对特征。

### 9.2 坡度计算（曲面拟合法）

在3×3窗口内，使用最小二乘法拟合平面：

$$S_X = \frac{1}{8d}\left[f(i-1,j-1) + 2f(i-1,j) + f(i-1,j+1) - f(i+1,j-1) - 2f(i+1,j) - f(i+1,j+1)\right]$$

$$S_Y = \frac{1}{8d}\left[f(i-1,j-1) + 2f(i,j-1) + f(i+1,j-1) - f(i-1,j+1) - 2f(i,j+1) - f(i+1,j+1)\right]$$

$$S = \arctan\sqrt{S_X^2 + S_Y^2}$$

### 9.3 横滚角（Roll）计算

使用局部地形方差估计：

$$\text{Roll} = \text{var}(f_{ij}) = \frac{1}{N-1} \sum_{(x_m, y_n) \in N_{ij}} \left[f(x_m, y_n) - \bar{f}_{ij}\right]^2$$

### 9.4 粗糙度（Fractional Brownian Motion模型）

基于分形布朗运动（fBm）模型：

$$\log E\left[|f(x+\Delta x, y+\Delta y) - f(x,y)|\right] = H \log d + \log C$$

其中 $d = \sqrt{\Delta x^2 + \Delta y^2}$，$H$ 为Hurst指数，$D_f = n + 1 - H$ 为分形维数。

**Hurst指数计算**：

$$H = \frac{\log E[|\Delta f|] - \log C}{\log d}$$

- $H$ 越大 → 表面越平滑
- $H$ 越小 → 表面越粗糙

### 9.5 模糊规则融合

| 坡度 | 横滚 | 粗糙度 | 可通行性 |
|---|---|---|---|
| flat | even | smooth | high |
| flat | even | rough | high |
| sloped | even | smooth | high |
| sloped | even | rough | median |
| steep | - | - | low |
| - | uneven | - | low |
| - | - | bumpy | low~median |

---

## 第10章 Pundir & Garg (2020) — 越野通行性制图技术

**文献**: Pundir S.K., Garg R.D. "Development of mapping techniques for off road trafficability" *Spatial Information Research*, 2020.

### 10.1 植被指数

**归一化植被指数（NDVI）**：

$$\text{NDVI} = \frac{\text{NIR} - \text{RED}}{\text{NIR} + \text{RED}}$$

**增强NDVI（eNDVI）**：

$$\text{eNDVI} = \text{NDVI} \times (\text{NDVI}_v - \text{NDVI}_s)$$

**土壤调节植被指数（SAVI）**：

$$\text{SAVI} = \frac{(\text{NIR} - \text{RED})(1 + L)}{\text{NIR} + \text{RED} + L}$$

其中 $L = 0.465$。

**土壤含水量指数（SMI）**：

$$\text{SMI} = \frac{\text{SWIR1} - \text{TIR1}}{\text{SWIR1} + \text{TIR1}}$$

### 10.2 因子权重与综合评分体系

Pundir & Garg (2020) 采用**累积评分法**（0-30分制）对越野通行能力进行综合评估。各因子的权重分配如下：

| 因子 | 权重（满分） | 评分标准 |
|------|------------|---------|
| 坡度 | 10 | 0-7°=10, 8-14°=8, 15-21°=6, 22-28°=4, 29-35°=2, >35°=0 |
| 土地利用 | 8 | 开阔地=8, 稀疏植被=6, 农田=4, 密林=2, 水体=0 |
| 土壤类型 | 7 | 砂土=7, 壤土=5, 粉砂=3, 粘土=1 |
| 土壤湿度 | 5 | 干燥=5, 潮湿=3, 饱和=1, 积水=0 |

综合通行能力得分：

$$S = S_{\text{slope}} + S_{\text{LU}} + S_{\text{soil}} + S_{\text{moisture}}$$

通行能力等级划分：
- **Good (GO)**: $S \geq 24$
- **Restricted**: $15 \leq S < 24$
- **Difficult (No GO)**: $S < 15$

---

## 第11章 Pundir & Garg (2022) — 车辆特定越野通行能力评估技术

**文献**: Pundir S.K., Garg R.D. "Development of Technique for Vehicle Specific Off-Road Trafficability Assessment Using Soil Cone Index, Water Index, and Geospatial Data" *Photogrammetric Engineering & Remote Sensing*, 2022, 88(11): 689-697.

### 11.1 RCI计算公式（Knight 1961）

除砾石和泥炭外，RCI的通用计算公式为：

$$\text{RCI} = \exp[a' - b' \ln(\text{MC})]$$

其中 $a'$ 和 $b'$ 为各土壤类型的系数，MC为质量含水率。

**MC与体积含水率的关系**：

$$\text{MC}(\%) = M_V \cdot \frac{w}{s} \times 100\%$$

其中 $M_V$ 为体积含水率，$w$ 为水密度，$s$ 为土壤密度。

### 11.2 各土壤类型的RCI系数

| USDA分类 | USCS分类 | $a'$ | $b'$ | 干密度(lb/ft³) | 平均含水率(%) |
|----------|----------|------|------|---------------|-------------|
| Sand | SP | 3.987 | -0.815 | 93.6 | 34.70 |
| Loamy sand | SM | 12.542 | -2.955 | 93.7 | 40.80 |
| Sandy loam | SM | 12.542 | -2.955 | 93.7 | 40.80 |
| Loam | ML | 11.936 | -2.407 | 73.7 | 53.70 |
| Sandy clay loam | SC | 12.542 | -2.955 | 97.4 | 41.90 |
| Clay loam | CL | 15.506 | -3.530 | 86.8 | 46.90 |

### 11.3 NDWI水分指数

$$\text{NDWI} = \frac{\text{GREEN} - \text{NIR}}{\text{GREEN} + \text{NIR}}$$

NDWI > 0.3 时判定为水体或高度饱和区域，RCI取零值。

### 11.4 VCI与通行能力判定

车辆锥形指数（VCI）基于车辆特性固定。通行能力判定规则：

$$\text{RCI} > \text{VCI} \implies \text{可通行}$$

$$\text{RCI} \leq \text{VCI} \implies \text{不可通行}$$

---

## 第12章 Pundir & Garg (2021) — 修正RCI方程的综合方法

**文献**: Pundir S.K., Garg R.D. "A comprehensive approach for off-road trafficability evaluation and development of modified equation for estimation of RCI to assess regional soil variation using geospatial technology" *Quaternary Science Advances*, 2022, 5: 100042.

### 12.1 原始RCI方程

Knight (1961) 原始方程：

$$\text{RCI} = \exp[a' - b' \ln(\text{MC})]$$

### 12.2 修正RCI方程

基于区域土壤变异分析，提出修正方程：

$$\text{RCI} = a \ln(\text{MC}) + b + \exp[a' - b' \ln(\text{MC})]$$

其中 $a$ 和 $b$ 为基于实测数据拟合的常数，各土壤类型的值如下：

| 土壤类型 | $a$ | $b$ |
|----------|-----|-----|
| Loam | -6.33 | 19.08 |
| Sandy clay loam | -113.00 | 367.50 |
| Clay loam | -215.00 | 753.60 |
| Sand | 12.19 | -10.84 |
| Sandy loam | -68.40 | 218.80 |

### 12.3 残差RCI（RRCI）

$$\text{RRCI} = \text{RCI}_{\text{computed}} - \text{RCI}_{\text{observed}}$$

用于评估计算值与实测值的偏差，指导修正方程的建立。

### 12.4 精度评估

决定系数：

$$R^2 = 1 - \frac{\sum(X - Y)^2}{\sum(X - \bar{X})^2}$$

Jaisalmer地区 $R^2 = 0.7754$，Abohar地区 $R^2 = 0.4867$。

---

## 第13章 Salmivaara et al. (2024) — 森林滚动阻力与通行能力

**文献**: Salmivaara A. et al. "High-resolution harvester data for estimating rolling resistance and forest trafficability" *European Journal of Forest Research*, 2024, 143: 1641-1656.

### 13.1 传输功率模型

液压传输输出功率 $P_t$ 的计算：

$$P_t = \frac{P_{hdiff} \cdot (n_m \cdot 60 \cdot (I_m \cdot a_1 + b_1) \cdot m_{vol} \cdot 1000)}{1000}$$

其中 $P_{hdiff}$ 为液压差压(kPa)，$n_m$ 为液压马达转速(s⁻¹)，$I_m$ 为马达控制电流(mA)，$a_1 = (V_{mmax} - V_{mmin})/(I_{mmin} - I_{mmax})$，$m_{vol}$ 为马达容积效率系数。

### 13.2 分段传输功率函数

$$P_t(P_e) = \begin{cases}
A \cdot P_e + B \cdot (P_e - P_{brake})^3 + D, & P_e \leq P_{brake} \\
A \cdot P_e + C \cdot (P_e - P_{brake})^2 + D, & P_{brake} < P_e \leq P_{cut} \\
P_C + (E - P_C) \cdot (1 - \exp((P_{cut} - P_e) \cdot F)), & P_e > P_{cut}
\end{cases}$$

拟合参数：$A = 0.327$, $B = 0.005 \text{ kW}^{-2}$, $C = 0.004 \text{ kW}^{-1}$, $D = -12.662 \text{ kW}$, $E = 116.949 \text{ kW}$, $F = 138.331 \text{ kW}$, $P_{brake} = 21.921 \text{ kW}$, $P_{cut} = 105.847 \text{ kW}$。RMSE = 7.37 kW，$R^2 = 0.94$。

### 13.3 运动方程与滚动阻力系数

运动方程：

$$F_m + F_s + F_r = ma$$

假设匀速行驶（$a = 0$），滚动阻力系数：

$$R_R = \frac{F_r}{F_n}$$

其中法向力 $F_n = \|mg\cos\theta\|$，坡度力 $F_s = \|mg\sin\theta\|$。

**四种工况下的 $R_R$ 计算**：

| 工况 | 条件 | $R_R$ 公式 |
|------|------|-----------|
| 上坡驱动 | $\theta \geq 0, P_t \geq 0$ | $R_R = (F_m - F_s)/F_n$ |
| 下坡驱动 | $\theta < 0, P_t \geq 0$ | $R_R = (F_m + F_s)/F_n$ |
| 上坡制动 | $\theta \geq 0, P_t < 0$ | 匀速不可能 |
| 下坡制动 | $\theta < 0, P_t < 0$ | $R_R = (F_s - F_m)/F_n$ |

驱动力 $F_m = \|P_t/(v \cdot \eta)\|$（$\eta = 0.74$ 为机械传动效率）。

实测结果：林道 $R_R$ 均值 0.126，林外 $R_R$ 均值 0.163。站点间 $R_R$ 范围 0.14-0.19。

---

## 第14章 Borges et al. (2022) — 地形可通行性分析综述

**文献**: Borges P.V.K. et al. "A Survey on Terrain Traversability Analysis for Autonomous Ground Vehicles" *Field Robotics*, 2022, 2: 1567-1627.

### 14.1 可通行性定义体系

本文建立了完整的地形分析术语体系：

1. **障碍检测（Obstacle Detection）**：识别不可通行区域，生成二值占用图
2. **地形分类（Terrain Classification）**：语义标注地形类型（岩石、草地、水面等）
3. **可通行性分析（Traversability Analysis）**：结合地形特征与车辆动力学，生成代价/难度图

### 14.2 粗糙度度量方法

**最小二乘平面拟合法**：对车辆尺寸区域内的点云拟合平面，计算残差：

$$\text{roughness} \propto \sqrt{\frac{1}{n}\sum_{i=1}^{n} r_i^2}$$

其中 $r_i$ 为第 $i$ 个点到拟合平面的距离。

**主成分分析法（PCA）**：对地形点云的协方差矩阵进行特征值分解，特征值分布反映地形几何特征：
- 最大特征值 >> 其余特征值 → 平坦地面
- 特征值相近 → 粗糙/随机地形

### 14.3 可通行性指标分类

| 指标类型 | 依赖性 | 示例 |
|----------|--------|------|
| 纯地形指标 | 仅依赖地形 | 坡度、粗糙度、曲率 |
| 平台相关指标 | 依赖车辆特性 | 运动学位姿、动力学响应 |
| 概率指标 | 融合不确定性 | 翻车概率、碰撞风险、综合代价 |

---

## 第15章 Wallin et al. (2022) — 多目标粗糙地形可通行性学习

**文献**: Wallin E. et al. "Learning multiobjective rough terrain traversability" *arXiv:2203.16354*, 2022.

### 15.1 三个独立可通行性度量

**运动能力（Locomotion）** $L \in [0,1]$：

$$L = \exp\left(-\frac{1}{2\sigma^2}\left(\frac{d_\tau - \hat{d}}{\hat{d}}\right)^2\right)$$

其中 $d_\tau$ 为时间窗口 $\tau$ 内实际行驶距离，$\hat{d} = v\tau$ 为名义距离，$\sigma = 1/3$。当 $d_\tau < 0.2\hat{d}$ 连续5个观测时判定为卡住。

**能量消耗（Energy Consumption）** $E \in [0,1]$：

$$E = \frac{1}{d \cdot E_0} \int_t^{t+\tau} \sum_i P_i(t) dt$$

其中 $P_i(t) = \omega_i(t) M_i(t)$ 为第 $i$ 个电机的功率，$E_0 = 700 \text{ kJ/m}$ 为归一化系数。

**加速度（Acceleration）** $A$：

$$A = \frac{1}{\tau} \int_t^{t+\tau} |\mathbf{a}(t)| dt$$

其中 $\mathbf{a}(t)$ 为车辆后框架选定点的加速度。

### 15.2 深度神经网络模型

输入：目标速度 $v$ + 局部高程图 $h_{ij}$（64×32网格，覆盖10×5 m²区域）

输出：$L$, $E$, $A$ 三个可通行性度量

预测精度：90%，推理速度比仿真快3000倍。

---

## 第16章 Romo et al. (2022) — 各向异性代价表面与最小代价路径

**文献**: Romo C. et al. "A New Approach for Computing Anisotropic Cost Surfaces and Least-Cost Paths" *Journal of Geography and Earth Sciences*, 2022, 10(1): 1-15.

### 16.1 代价表面定义

代价表面函数：

$$C(Q) = \min_{P: S \to Q} \int_P f(x) \, dx$$

其中 $f(x)$ 为摩擦函数，$S$ 为起始区域，$Q$ 为工作区域中任意点。

### 16.2 各向异性摩擦模型

将摩擦分解为两个分量：

$$f(x, \vec{v}, \dot{\vec{v}}) = F_a(x, \vec{v}) + F_t(\dot{\vec{v}})$$

其中 $F_a$ 为各向异性摩擦（依赖位置和运动方向），$F_t$ 为转向摩擦（依赖运动方向变化）。

### 16.3 转向摩擦函数

$$F_t(\theta) = k_1 \cdot \left(\frac{\theta}{\pi/8}\right)^{k_2}$$

其中 $\theta$ 为入射方向与出射方向的夹角，$k_1$ 为转角22.5°时的代价增量，$k_2$ 为代价增长率（$k_2 = 1$ 表示线性增长）。

### 16.4 路径代价计算

路径上第 $i$ 个单元格的代价贡献：

$$c_i = F_a(i, S_i) + F_t(S_i, S_{i-1})$$

其中 $S_i$ 为进入当前单元格的方向，$S_{i-1}$ 为进入前一单元格的方向。

---

## 第17章 Potić et al. (2024) — 地理空间通行能力图的MCDM方法

**文献**: Potić I. et al. "Development of Geospatial Passability Maps: A Multi-Criteria Analysis Approach" *J. Geogr. Inst. Cvijic.*, 2024, 74(1): 29-45.

### 17.1 坡度计算（Horn公式）

$$\text{Slope} = \arctan\sqrt{(dz/dx)^2 + (dz/dy)^2}$$

### 17.2 地形粗糙度指数（TRI）

$$\text{TRI} = \sqrt{\frac{\sum(Z_{ij} - Z_{00})^2}{n}}$$

其中 $Z_{ij}$ 为邻域单元格高程，$Z_{00}$ 为中心单元格高程，$n$ 为邻域单元格数。

### 17.3 MCDM加权叠加分析

各因子的通行能力系数（0-100%）：

**河流网络**：1级=100%, 2级=80%, 3级=40%, 4级=10%, 5级=5%, 6级=2%, 7级=0%, 桥梁=100%

**道路网络**：铁路/高速/主路=100%, 次路/未分类=80%, 三级路=60%

**土地覆盖（CLC）**：耕地/牧地=100%, 灌丛/裸岩=60%, 森林/水体=10-30%

**土壤类型**（分干湿两季）：

| 土壤类型 | 干季系数 | 湿季系数 |
|----------|---------|---------|
| 砾石 | 100% | 80% |
| 砂土 | 90% | 70% |
| 粉砂 | 80% | 30% |
| 粘土 | 70% | 20% |
| 有机质 | 40% | 10% |

最终通行能力图通过加权叠加（WOA）生成。

---

## 第18章 Marková et al. (2025) — 气象数据驱动的土壤通行能力建模

**文献**: Marková L. et al. "Meteorological data-driven approach for soil passability modeling in GIS using machine learning" *Geofizika*, 2025, 42(1): 53-81.

### 18.1 锥形指数（CI）与通行能力

CI是评估土壤承载力的关键指标。RCI由CI和重塑指数（RI）计算：

$$\text{RCI} = \text{CI} \times \text{RI}$$

通行能力判定：$\text{RCI} > \text{VCI}$ 时车辆可通行。

### 18.2 气象预测因子

从GFS全球预报模型提取的预测因子：
1. 雪深
2. 土壤温度（0-10 cm层）
3. 土壤温度（10-30 cm层）
4. 体积土壤含水量（0-10 cm层）
5. 体积土壤含水量（10-30 cm层）
6. 24小时累积降水
7. 48小时累积降水

### 18.3 机器学习方法

采用留一交叉验证（LOOCV）评估7种回归方法：

$$z = \frac{x - u}{s}$$

（标准化公式，$u$ 为均值，$s$ 为标准差）

**评估指标**：

$$\text{MAE} = \frac{1}{n}\sum_{i=0}^{n-1} |y_i - \hat{y}_i|$$

$$\text{MaxE} = \max_i |y_i - \hat{y}_i|$$

**关键发现**：
- 随机森林（RF）方法表现最佳
- 体积土壤含水量（10-30 cm层）是最重要的预测因子
- 干燥站点的预测精度高于湿润站点
- CI6（15 cm深度）与预测因子的相关性最强（Kendall相关系数达-0.53）

---

## 第19章 NATO NRMM — 下一代北约参考机动性模型

**文献**: ET-148 "Next-Generation NATO Reference Mobility Model (NRMM)" NATO STO Technical Report.

### 19.1 NRMM概述

NATO参考机动性模型（NRMM是预测车辆在指定地形条件下移动能力的仿真工具，由美国陆军TARDEC和ERDC于1960-70年代开发。

**NRMM的固有局限性**：
- 基于经验观测，外推困难
- 仅支持二维分析
- 不考虑车辆动态效应
- 仅考虑稳态条件
- 依赖现场土壤测量

### 19.2 NG-NRMM增强能力

下一代NRMM（NG-NRMM）的目标：
- 支持基于物理的车辆-地形交互模型
- 引入随机分析框架
- 支持无人地面车辆（UGV）
- 与现代车辆动力学仿真工具集成
- 高性能计算（HPC）并行化

### 19.3 关键输入参数

- **锥形指数（CI）**：土壤贯入阻力
- **重塑锥形指数（RCI）**：考虑重塑效应的土壤强度
- **车辆锥形指数（VCI）**：车辆特定的最低土壤强度要求
- **平均最大压力（MMP）**：评估越野机动性

---

## 第20章 Huang et al. (2019) — 基于PCA和优化ELM的驾驶性评估模型

**文献**: Huang W. et al. "Drivability evaluation model using principal component analysis and optimized extreme learning machine" *Journal of Vibration and Control*, 2019, 25(16): 2274-2281.

### 20.1 PCA降维

对评估指标向量 $\mathbf{X} = [x_{i1}, x_{i2}, ..., x_{im}]^T$ 进行PCA分析：

相关系数矩阵 $\mathbf{R} = \frac{1}{m-1}\sum_{t=1}^{m} \mathbf{x}_t \cdot \mathbf{x}_t^T$

新评估指标（主成分）：

$$q_i = \sqrt{\lambda_i} a_{i1} x_1 + \sqrt{\lambda_i} a_{i2} x_2 + ... + \sqrt{\lambda_i} a_{in} x_n$$

当累积方差贡献率超过80%时，选取前 $k$ 个主成分。

### 20.2 极限学习机（ELM）模型

$$\text{Model} = \text{ELM}(\omega_i, \beta_i, \xi_i, f(\mathbf{x}_i), m)$$

输出权重求解：

$$\hat{\beta} = \mathbf{H}^T \mathbf{T}$$

其中 $\mathbf{H} = f(\omega_k \cdot \mathbf{x}_j + b_h k)$ 为隐层输出矩阵，$\mathbf{T}$ 为实际评分矩阵。

### 20.3 PSO优化适应度函数

$$f_i = \frac{1}{m}\sum_{i=1}^{m}(y_i - t_i)^2$$

其中 $y_i$ 为预测评分，$t_i$ 为真实评分。

**结果**：PCA+OELM模型的R相关系数达0.979，通过率95%。

---

## 第21章 综合公式汇总

### 21.1 坡度计算方法对比

| 方法 | 公式 | 适用场景 | 来源 |
|------|------|---------|------|
| Zevenbergen偏四次曲面 | $\text{SLOPE} = \sqrt{G^2 + H^2}$ | DEM 3×3窗口 | Zevenbergen & Thorne 1987 |
| Sobel算子 | $S = \sqrt{S_x^2 + S_y^2}$ | DEM栅格计算 | 赵某 2025 |
| Horn公式 | $\text{Slope} = \arctan\sqrt{(dz/dx)^2 + (dz/dy)^2}$ | GIS软件 | Potić et al. 2024 |
| 曲面拟合法 | 最小二乘拟合曲面 | 多点高程 | Liu et al. 2006 |

### 21.2 粗糙度指标汇总

| 指标 | 公式 | 来源 |
|------|------|------|
| TRI | $\text{TRI} = \sqrt{\sum(Z_{ij}-Z_{00})^2/n}$ | Riley et al. 1999; Potić 2024 |
| TPI | $\text{TPI} = Z_0 - \bar{Z}_{neighbor}$ | Weiss 2001; Liu 2025 |
| fBm Hurst指数 | $H$ 通过功率谱密度拟合 | Liu et al. 2006 |
| 剖面曲率 | $\kappa_v = -2(DG^2 + EH^2 + FGH)/(G^2+H^2)^{3/2}$ | Zevenbergen 1987 |
| 平面曲率 | $\kappa_h = -2(DG^2 + EH^2 - FGH)/(G^2+H^2)^{3/2}$ | Zevenbergen 1987 |

### 21.3 土壤强度指标汇总

| 指标 | 公式 | 来源 |
|------|------|------|
| RCI (原始) | $\text{RCI} = \exp[a' - b'\ln(\text{MC})]$ | Knight 1961 |
| RCI (修正) | $\text{RCI} = a\ln(\text{MC}) + b + \exp[a' - b'\ln(\text{MC})]$ | Pundir & Garg 2021 |
| VCI (单次) | $\text{VCI}_1 = 15.95 + 1.86 \cdot \text{MI}$ (细粒土) | Rula & Nuttal 1971 |
| VCI (50次) | $\text{VCI}_{50} = 26.16 + 2.38 \cdot \text{MI}$ (细粒土) | Rula & Nuttal 1971 |
| MI (履带) | $\text{MI} = \frac{p \cdot W}{b \cdot L \cdot h}$ | NRMM |
| MI (轮式) | $\text{MI} = \frac{p \cdot W}{n \cdot b \cdot d}$ | NRMM |

### 21.4 通行能力评估方法汇总

| 方法 | 类型 | 来源 |
|------|------|------|
| AHP权重叠加 | 多准则决策 | He et al. 2023; Potić 2024 |
| 规则库方法 | 土壤×坡度×湿度组合表 | Pundir & Garg 2020 |
| 累积评分法 | 0-30分制 | Pundir & Garg 2020 |
| 模糊综合评判 | 模糊逻辑 | 多篇文献 |
| 随机森林回归 | 机器学习 | Marková et al. 2025 |
| 深度神经网络 | CNN特征提取 | Wallin et al. 2022 |

---

*文档完成。共提取21章内容，覆盖文件夹中所有与越野通行能力建模相关的PDF文献。*