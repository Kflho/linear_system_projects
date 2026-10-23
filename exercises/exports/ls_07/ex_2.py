"""7.2  最小相位 SISO 系统对周期期望输出的跟踪（课件 (7.50)）。

A = [[-1,1,2,-1],[0,-2,1,-1],[-1,0,-2,1],[1,0,1,0]]，B = [0,1,0,1]'，C = [1,0,1,0]，
y*(t) 是周期为 2 的信号：t∈[0,1] 时 sin^3(pi t)，t∈[1,2] 时 0。

G(s) = (s^2+2s+4)/(s^4+5s^3+10s^2+8s-4)，零点 -1±j√3（左半平面），相对阶 2，
Gamma = c A B = 1，取 gamma(s) = (s+2)(s+3)：

    u = K x + (y*'' + 5 y*' + 6 y*),  K = [2, -1, -3, -1]

误差 e = [y - y*, y' - y*'] 满足 e' = [[0,1],[-6,-5]] e -> 0。
"""
import os

import matplotlib.pyplot as plt
import numpy as np
import sympy as sp
from scipy.integrate import solve_ivp

from linear_system_utils.cal_matrix_operation.cal_tracking_control.cal_tracking_control import (
    cal_coupling_matrix,
    cal_error_dynamics,
    cal_tracking_control,
)

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

A = sp.Matrix([[-1, 1, 2, -1],
               [0, -2, 1, -1],
               [-1, 0, -2, 1],
               [1, 0, 1, 0]])
B = sp.Matrix([0, 1, 0, 1])
C = sp.Matrix([[1, 0, 1, 0]])
s, t = sp.symbols("s t")
gamma = [(s + 2) * (s + 3)]

# ---- 传函、零点、相对阶 -------------------------------------------------
G_s = sp.simplify(C * (s * sp.eye(4) - A).inv() * B)[0]
num, den = sp.fraction(sp.cancel(G_s))
print("G(s) =", sp.factor(num), "/", sp.factor(den))
print("零点 =", sp.solve(num, s))
print("极点 ≈", [sp.N(rt, 6) for rt in sp.nroots(den)])
r, Gamma = cal_coupling_matrix(A, B, C)
print("相对阶 r =", r, "   Gamma = c A B =", Gamma)

# ---- 控制律 ------------------------------------------------------------
K, Gamma, r, info = cal_tracking_control(A, B, C, gamma)
print("K =", K, "   Gamma =", Gamma)
A_e = cal_error_dynamics(gamma, r)
print("误差系统 A_e =", A_e, " 特征多项式 =", sp.factor(A_e.charpoly(s).as_expr()))

period = 2
t0 = sp.Symbol("t0")
y_first = sp.sin(sp.pi * t0) ** 3                      # t ∈ [0,1] 那一段
v_first = y_first.diff(t0, 2) + 5 * y_first.diff(t0) + 6 * y_first
reduced = sp.Rational(1, 4) * (
    (9 * sp.pi**2 - 6) * sp.sin(3 * sp.pi * t0)
    + (18 - 3 * sp.pi**2) * sp.sin(sp.pi * t0)
    + 15 * sp.pi * sp.cos(sp.pi * t0)
    - 15 * sp.pi * sp.cos(3 * sp.pi * t0)
)
print("v*（t∈[0,1]）= [(9π²-6) sin(3πt) + (18-3π²) sin(πt) + 15π cos(πt) - 15π cos(3πt)] / 4")
print("  与逐阶求导的结果一致：", sp.simplify(sp.expand_trig(v_first - reduced)) == 0)
print("v*（t∈[1,2]）= 0")

# ---- 仿真 --------------------------------------------------------------
An = np.array(A.tolist(), dtype=float)
Bn = np.array(B.tolist(), dtype=float)
Cn = np.array(C.tolist(), dtype=float)
Kn = np.array(K.tolist(), dtype=float)
gam = float(Gamma[0])


def ystar(tt):
    return np.sin(np.pi * tt) ** 3 if (tt % period) <= 1 else 0.0


def ref(tt):
    """v* = y*'' + 5 y*' + 6 y*（一个周期内分段给公式）"""
    tt = tt % period
    if tt <= 1:
        sin_, cos_ = np.sin(np.pi * tt), np.cos(np.pi * tt)
        y0 = sin_**3
        y1 = 3 * np.pi * sin_**2 * cos_
        y2 = 3 * np.pi**2 * (2 * sin_ * cos_**2 - sin_**3)
        return y2 + 5 * y1 + 6 * y0
    return 0.0


def rhs(tt, x):
    u = Kn @ x + ref(tt) / gam
    return An @ x + Bn.ravel() * u


T_END = 6.0
x0 = np.array([0.5, -1.0, 1.0, 0.0])       # 故意给个非零初值，看误差收敛
ts = np.linspace(0.0, T_END, 3001)
sol = solve_ivp(rhs, (0.0, T_END), x0, t_eval=ts, rtol=1e-10, atol=1e-12)
y = (Cn @ sol.y).ravel()
ys = np.array([ystar(tt) for tt in ts])
err = y - ys
print("\n数值例：gamma(s)=(s+2)(s+3)，x(0)=(0.5,-1,1,0)'，t∈[0,6]（3 个周期）")
print("K =", np.array2string(Kn, precision=4), "  Gamma =", gam)
print("稳态 |y - y*| 最大值（最后 2 s）=", np.max(np.abs(err[ts > T_END - 2])))
print("整段 |y - y*| 最大值 =", np.max(np.abs(err)), "（含启动段）")
print("状态范数 ||x(t)|| 的最大值 =", np.max(np.linalg.norm(sol.y, axis=0)),
      "（闭环有界，零动态极点是传函的零点 -1±j√3）")

fig, axes = plt.subplots(2, 1, figsize=(7.2, 5.2), sharex=True)
axes[0].plot(ts, ys, "--", lw=2, label="期望输出 $y^*(t)$")
axes[0].plot(ts, y, lw=1.5, label="实际输出 $y(t)$")
axes[0].set_ylabel("输出")
axes[0].legend(loc="upper right")
axes[0].grid(alpha=0.3)
axes[1].plot(ts, err, lw=1.5, color="tab:red")
axes[1].set_ylabel("跟踪误差 $y-y^*$")
axes[1].set_xlabel("时间 $t$ / s")
axes[1].grid(alpha=0.3)
fig.suptitle("7.2  跟踪周期期望输出（$\\gamma(s)=(s+2)(s+3)$）")
fig.tight_layout()
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fig", "ex_2_tracking.png")
os.makedirs(os.path.dirname(out), exist_ok=True)
fig.savefig(out, dpi=150)
print("图已保存：", out)
