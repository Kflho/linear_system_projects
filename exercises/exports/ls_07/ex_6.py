"""7.6  MIMO 系统的轨迹跟踪：耦合矩阵奇异 → 输出变换 → 跟踪。

A = [[0,1,0],[-1,0,1],[1,-2,0]]，B = [[1,0],[0,0],[0,1]]，
y = [[1,-1,1],[1,1,1]] x，y* = [sin t, cos 2t]'。

原输出：C B = [[1,1],[1,1]] 奇异（两行相同），耦合矩阵不满秩，静态状态反馈
无法让 y 的两个分量各自跟踪。做 ỹ = [[1,0],[-1,1]] y = [[1,-1,1],[0,2,0]] x 后：
r̃ = [1, 2]，Gamma~ = [[1,1],[-2,2]]，det = 4 != 0，可以对 ỹ 设计跟踪律，
再取 ỹ* = T y*，由 y = T^{-1} ỹ 得 y → y*。
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

s, t = sp.symbols("s t")
A = sp.Matrix([[0, 1, 0], [-1, 0, 1], [1, -2, 0]])
B = sp.Matrix([[1, 0], [0, 0], [0, 1]])
C = sp.Matrix([[1, -1, 1], [1, 1, 1]])
T_out = sp.Matrix([[1, 0], [-1, 1]])          # 题目给的输出变换

print("原输出 C =", C)
print("C B =", C * B, " det =", (C * B).det(), " rank =", (C * B).rank())

C_new = sp.simplify(T_out * C)
print("\n变换后 C~ = T C =", C_new)
r, Gamma = cal_coupling_matrix(A, B, C)
print("原耦合矩阵 Gamma =", Gamma, " r =", r, " det =", Gamma.det())
r_new, Gamma_new = cal_coupling_matrix(A, B, C_new)
print("新耦合矩阵 Gamma~ =", Gamma_new, " r~ =", r_new, " det =", Gamma_new.det())

# ---- 对 ỹ 设计跟踪律 ---------------------------------------------------
g1 = (s + 2)
g2 = (s + 2) * (s + 3)
K, Gamma_new, r_new, info = cal_tracking_control(A, B, C_new, [g1, g2])
print("\n取 gamma1(s)=s+2, gamma2(s)=(s+2)(s+3)：")
print("K =", K, "   Gamma~^{-1} =", sp.simplify(Gamma_new.inv()))
print("误差系统 A_e =", cal_error_dynamics([g1, g2], r_new))
print("  特征多项式 =", sp.factor(cal_error_dynamics([g1, g2], r_new).charpoly(s).as_expr()))

y_star = [sp.sin(t), sp.cos(2 * t)]
ytilde_star = sp.expand(T_out * sp.Matrix(y_star))
print("\ny* =", sp.Matrix(y_star).T, " →  ỹ* = T y* =", ytilde_star.T)
vstar = cal_reference_vector([g1, g2], list(ytilde_star), r_new, t)
print("v~* =", sp.expand(vstar).T)

# ---- 仿真 --------------------------------------------------------------
An = np.array(A.tolist(), dtype=float)
Bn = np.array(B.tolist(), dtype=float)
Cn = np.array(C.tolist(), dtype=float)
Kn = np.array(K.tolist(), dtype=float)
Ginv = np.array(Gamma_new.inv().tolist(), dtype=float)
vfun = sp.lambdify(t, list(sp.expand(vstar)), "numpy")


def rhs(tt, x):
    v = np.array(vfun(tt), dtype=float)
    u = Kn @ x + Ginv @ v
    return An @ x + Bn @ u


T_END = 12.0
ts = np.linspace(0.0, T_END, 4801)
sol = solve_ivp(rhs, (0.0, T_END), np.zeros(3), t_eval=ts, rtol=1e-10, atol=1e-12)
y = Cn @ sol.y
y_true = np.vstack([np.sin(ts), np.cos(2 * ts)])
err = y - y_true
print("\n数值例：x(0)=0，t∈[0,12]")
print("K =", np.array2string(Kn, precision=4))
print("稳态 max|y1-y1*|, max|y2-y2*|（最后 2 s）=",
      np.max(np.abs(err[0, ts > T_END - 2])), np.max(np.abs(err[1, ts > T_END - 2])))
print("整段 max|y - y*| =", np.max(np.abs(err)))

fig, axes = plt.subplots(2, 1, figsize=(7.4, 5.6), sharex=True)
axes[0].plot(ts, y_true[0], "--", lw=2, label="期望 $y_1^*=\\sin t$")
axes[0].plot(ts, y[0], lw=1.5, label="实际 $y_1$")
axes[0].set_ylabel("$y_1$")
axes[0].legend(loc="upper right")
axes[0].grid(alpha=0.3)
axes[1].plot(ts, y_true[1], "--", lw=2, label="期望 $y_2^*=\\cos 2t$")
axes[1].plot(ts, y[1], lw=1.5, label="实际 $y_2$")
axes[1].set_ylabel("$y_2$")
axes[1].set_xlabel("时间 $t$ / s")
axes[1].legend(loc="upper right")
axes[1].grid(alpha=0.3)
fig.suptitle("7.6  输出变换后跟踪 $y^*=[\\sin t,\\ \\cos 2t]^T$（$\\tilde{y}=Ty$）")
fig.tight_layout()
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fig", "ex_6_tracking.png")
os.makedirs(os.path.dirname(out), exist_ok=True)
fig.savefig(out, dpi=150)
print("图已保存：", out)
