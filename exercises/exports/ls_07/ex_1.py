"""7.1  单输入系统的轨迹跟踪（课件 7.1，(7.10)）。

x' = [[0,1,0,0],[0,0,1,0],[0,0,0,1],[-a0,-a1,-a2,-a3]] x + [0,0,0,1]' u
y  = [b0, b1, b2, 0] x
beta2 s^2 + beta1 s + beta0 是 Hurwitz 多项式，期望输出 y* = sin(w t)。

传函 = (b2 s^2 + b1 s + b0)/(s^4 + a3 s^3 + a2 s^2 + a1 s + a0)，相对阶 2，
Gamma = c A B = beta2，所以

    u = K x + (1/beta2) [(gamma0 - w^2) sin(w t) + gamma1 w cos(w t)]

误差 e = [y - y*, y' - y*'] 满足 e' = [[0,1],[-gamma0,-gamma1]] e -> 0。
"""
import os

import matplotlib.pyplot as plt
import numpy as np
import sympy as sp
from scipy.integrate import solve_ivp

from linear_system_utils.cal_matrix_operation.cal_tracking_control.cal_tracking_control import (
    cal_coupling_matrix,
    cal_error_dynamics,
    cal_reference_vector,
    cal_tracking_control,
)

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

s, t, w = sp.symbols("s t omega")
a = sp.symbols("alpha0:4")
b = sp.symbols("beta0:3")
g1, g0 = sp.symbols("gamma1 gamma0")

A = sp.Matrix([[0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1],
               [-a[0], -a[1], -a[2], -a[3]]])
B = sp.Matrix([0, 0, 0, 1])
C = sp.Matrix([[b[0], b[1], b[2], 0]])
gamma = [s**2 + g1 * s + g0]

# ---- 第一问：相对阶、耦合矩阵、控制律 ----------------------------------
G_s = sp.simplify(C * (s * sp.eye(4) - A).inv() * B)[0]
print("传递函数 G(s) =", sp.factor(sp.cancel(G_s)))
r, Gamma = cal_coupling_matrix(A, B, C)
print("相对阶 r =", r, "   耦合矩阵 Gamma = c A B =", Gamma)
K, Gamma, r, info = cal_tracking_control(A, B, C, gamma)
print("K =", K)
vstar = cal_reference_vector(gamma, [sp.sin(w * t)], r, t)
print("v* =", sp.simplify(vstar))
A_e = cal_error_dynamics(gamma, r)
print("误差系统 A_e =", A_e, "  特征多项式 =", sp.factor(A_e.charpoly(s).as_expr()))

# ---- 数值例子：alpha = (1,4,6,4)（= (s+1)^4），beta = (2,3,1)，gamma = (s+2)^2
subs = {a[0]: 1, a[1]: 4, a[2]: 6, a[3]: 4, b[0]: 2, b[1]: 3, b[2]: 1,
        g0: 4, g1: 4, w: 1}
Kn = np.array(K.subs(subs).tolist(), dtype=float)
An = np.array(A.subs(subs).tolist(), dtype=float)
Bn = np.array(B.subs(subs).tolist(), dtype=float)
Cn = np.array(C.subs(subs).tolist(), dtype=float)
gam = float(Gamma.subs(subs)[0])
vn = sp.lambdify(t, vstar.subs(subs)[0], "numpy")
print("\n数值例：alpha=(1,4,6,4), beta=(2,3,1), gamma(s)=(s+2)^2, w=1")
print("K =", np.array2string(Kn, precision=4), "  Gamma =", gam)
print("误差系统极点 =", np.linalg.eigvals(np.array(A_e.subs(subs).tolist(), dtype=float)))


def rhs(tt, x):
    u = Kn @ x + vn(tt) / gam
    return An @ x + Bn.ravel() * u


T_END = 12.0
ts = np.linspace(0.0, T_END, 2401)
sol = solve_ivp(rhs, (0.0, T_END), np.zeros(4), t_eval=ts, rtol=1e-10, atol=1e-12)
y = (Cn @ sol.y).ravel()
ystar = np.sin(ts)
err = y - ystar
print("稳态 |y - y*| 最大值（最后 2 s）=", np.max(np.abs(err[ts > T_END - 2])))

fig, axes = plt.subplots(2, 1, figsize=(7.2, 5.2), sharex=True)
axes[0].plot(ts, ystar, "--", lw=2, label="期望输出 $y^*(t)=\\sin t$")
axes[0].plot(ts, y, lw=1.5, label="实际输出 $y(t)$")
axes[0].set_ylabel("输出")
axes[0].legend(loc="upper right")
axes[0].grid(alpha=0.3)
axes[1].plot(ts, err, lw=1.5, color="tab:red")
axes[1].set_ylabel("跟踪误差 $y-y^*$")
axes[1].set_xlabel("时间 $t$ / s")
axes[1].grid(alpha=0.3)
fig.suptitle("7.1  轨迹跟踪（$\\alpha=(1,4,6,4)$, $\\beta=(2,3,1)$, $\\gamma(s)=(s+2)^2$, $\\omega=1$）")
fig.tight_layout()
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fig", "ex_1_tracking.png")
os.makedirs(os.path.dirname(out), exist_ok=True)
fig.savefig(out, dpi=150)
print("图已保存：", out)
