# v11.1.1 model formulas and experiment logic

## 1. Why the model is direction-dependent

Traditional raster cost surfaces assign one value to one cell. That form can be written as

$$
C(i).
$$

For off-road vehicle trafficability, the same terrain cell can be encountered as uphill, downhill, or side-slope depending on the travel direction. Therefore the model uses a directional edge cost:

$$
C(i,d),
$$

where $i$ is the grid cell and $d$ is the travel direction.

## 2. DEM gradient decomposition

Let the DEM be

$$
z=z(x,y).
$$

The local terrain gradient is

$$
\nabla z_i=(p_i,q_i)=\left(\frac{\partial z}{\partial x},\frac{\partial z}{\partial y}\right).
$$

For a travel direction angle $\theta_d$, the forward unit vector is

$$
\mathbf{u}_d=(\cos\theta_d,\sin\theta_d),
$$

and the lateral unit vector is

$$
\mathbf{n}_d=(-\sin\theta_d,\cos\theta_d).
$$

The longitudinal grade tangent is

$$
g_{\parallel}(i,d)=\nabla z_i\cdot\mathbf{u}_d.
$$

The lateral grade tangent is

$$
g_{\perp}(i,d)=\nabla z_i\cdot\mathbf{n}_d.
$$

The corresponding slope angles are

$$
\alpha_{\parallel}(i,d)=\arctan(g_{\parallel}(i,d)),
$$

and

$$
\alpha_{\perp}(i,d)=\arctan(|g_{\perp}(i,d)|).
$$

This decomposition is used because vehicle demand is not controlled by the scalar terrain slope alone. Uphill travel mainly tests traction, downhill travel mainly tests braking adhesion, and side-slope travel mainly tests sliding and rollover stability.

## 3. Vehicle capability utilization ratios

The uphill traction utilization is

$$
\rho_{\mathrm{up}}(i,d)=
\frac{\tan(\max(\alpha_{\parallel}(i,d),0))}{\tan(\alpha_{\mathrm{grade}})}.
$$

The numerator is the demanded uphill grade. The denominator is the assumed acceptable climbing capability. When the ratio approaches 1, the route segment approaches the modeled climbing boundary.

The downhill braking utilization is

$$
\rho_{\mathrm{down}}(i,d)=
\frac{\tan(\max(-\alpha_{\parallel}(i,d),0))}{\mu_b(i)}.
$$

The numerator is the demanded downhill grade. The denominator $\mu_b(i)$ is the spatial braking adhesion supply. Wet or weak surfaces reduce $\mu_b(i)$ and therefore increase the utilization ratio.

The rollover stability utilization is

$$
\rho_{\mathrm{roll}}(i,d)=
\frac{\tan(\alpha_{\perp}(i,d))}{B/(2h_c)}.
$$

Here $B$ is vehicle track width and $h_c$ is center-of-gravity height. The denominator $B/(2h_c)$ is a static stability factor. A higher center of gravity or narrower track width reduces rollover stability.

The side-slip utilization is

$$
\rho_{\mathrm{slide}}(i,d)=
\frac{\tan(\alpha_{\perp}(i,d))}{\mu_s(i)}.
$$

The numerator is the lateral grade demand, while $\mu_s(i)$ is the spatial lateral adhesion supply. This term is important because a route with small longitudinal grade may still be dangerous if it traverses a steep side-slope.

The integrated capability utilization is

$$
\rho_{\max}(i,d)=\max\left(\rho_{\mathrm{up}},\rho_{\mathrm{down}},\rho_{\mathrm{roll}},\rho_{\mathrm{slide}}\right).
$$

The maximum operator is used because off-road passability is governed by the most restrictive active constraint. A good value in one component cannot compensate for exceeding another physical capability boundary.

## 4. V10 soft cost

The v10 soft unit cost is

$$
C_{V10}(i,d)=1+\rho_{\max}(i,d).
$$

The constant 1 keeps distance as a baseline cost even on easy terrain. The second term adds vehicle capability pressure. This makes the path prefer both short distance and lower utilization.

The path planning objective is

$$
J_m^{\mathrm{plan}}(P)=\sum_{e_k\in P} C_m(e_k)\,l(e_k),
$$

where $l(e_k)$ is the edge length. $J_m^{\mathrm{plan}}$ is a relative accumulated path cost, not Joule energy.

## 5. V11 unified evaluation

Different planning models have different internal cost scales, so their planned costs are not directly comparable. V11 therefore evaluates every planned path under the same V10 capability model:

$$
E_{V10}(P_m)=\left\{\bar{\rho}_{\max}, P(\rho_{\max}>1), P(\rho_{\mathrm{slide}}>0.7), P(\rho_{\mathrm{down}}>0.7), L, D\right\}.
$$

This answers a fairer question: after each model chooses a path, how demanding is that path under one common vehicle-capability interpretation?

## 6. V11 hard constraint reachability

The hard-constraint cost is

$$
C(i,d)=
\begin{cases}
1+\rho_{\max}(i,d), & \rho_{\max}(i,d)\leq \rho_{\lim},\\
+\infty, & \rho_{\max}(i,d)>\rho_{\lim}.
\end{cases}
$$

The threshold $\rho_{\lim}$ is a risk tolerance parameter. For example, $\rho_{\lim}=1.0$ means no edge may exceed the modeled capability boundary, while $\rho_{\lim}=1.2$ allows limited overload pressure.

The constrained path is

$$
P^*=\arg\min_P\sum_{e_k\in P}C(e_k)l(e_k),
$$

subject to

$$
\rho_{\max}(e_k)\leq \rho_{\lim},\quad \forall e_k\in P.
$$

This experiment changes the model from a ranking tool into a reachability tool: it can answer whether a feasible route exists under a specified capability threshold.
