"""9.6  全维、降维、q=1 函数观测器（课件 9.1-9.3）。

x' = [[-2,-2,0],[0,0,1],[0,-3,-4]] x + [0,1,-2]' u,  y = [[1,0,1],[0,1,0]] x

lambda(A) = {-1,-2,-3}，能观。观测器极点取 -4/-5/-6（全维）、-4（降维）、
-4（函数观测器，q = n-p = 1），都躲开 A 的特征值（函数观测器的 Sylvester 方程
要求 lambda(F) ∩ lambda(A) = 空）。
"""
import os

import matplotlib.pyplot as plt
import numpy as np
import sympy as sp
from scipy.integrate import solve_ivp

from linear_system_utils.cal_matrix_operation.cal_observer.cal_observer import (
    cal_function_observer,
    cal_full_order_observer,
    cal_observer_matrix,
    cal_reduced_order_observer,
)

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

A = np.array([[-2.0, -2.0, 0.0], [0.0, 0.0, 1.0], [0.0, -3.0, -4.0]])
B = np.array([[0.0], [1.0], [-2.0]])
C = np.array([[1.0, 0.0, 1.0], [0.0, 1.0, 0.0]])
C0 = np.array([[1.0], [0.0], [-1.0]])          # 满足 C C0 = 0，取整数解

As = sp.Matrix(A.tolist())
B_s = sp.Matrix(B.tolist())


def exact(M):
    """把 numpy 的小数矩阵显示成有理数（0.25 -> 1/4），方便和手算对照"""
    return sp.Matrix(np.atleast_2d(M).tolist()).applyfunc(
        lambda v: sp.nsimplify(round(float(v), 10), rational=True))


print("lambda(A) =", As.eigenvals())
print("能观性矩阵 O 的秩 =", np.linalg.matrix_rank(cal_observer_matrix(A, C)), "（n=3）")

# ---- 第一问：全维状态观测器（极点 -4,-5,-6）------------------------------
L, F_f, G_f, H_f, info_f = cal_full_order_observer(A, B, C, poles=[-4, -5, -6])
print("\n[全维] L =", exact(L), sep="\n")
print("F = A + L C =", exact(F_f), "  eig =",
      sp.Matrix(np.linalg.eigvals(F_f).real.round(10).tolist()), sep="\n")
print("G = -L =", exact(-L), " H = B =", exact(B), sep="")
print("观测器：x̂' = F x̂ + G y + H u")

# ---- 第二问：降维状态观测器（极点 -4）-----------------------------------
F_r, G_r, H_r, M_r, N_r, info_r = cal_reduced_order_observer(
    A, B, C, poles=[-4], C0=C0)
print("\n[降维] C0 =", C0.ravel(), "  T0 =", exact(info_r["T0"]), sep="\n")
print("分块：A11 =", exact(info_r["A11"]), " A12 =", C0 if False else info_r["A12"].ravel(),
      " A21 =", info_r["A21"].ravel(), " A22 =", info_r["A22"])
print("B1 =", info_r["B1"].ravel(), " B2 =", info_r["B2"].ravel())
print("L21 =", info_r["L21"].ravel(), " → F = A22 + L21 A12 =", F_r.ravel())
print("F =", F_r, " G =", exact(G_r), " H =", exact(H_r),
      " M =", M_r.ravel(), " N =", exact(N_r), sep="\n")
print("重构恒等式残差 M L21 + N - C^T(C C^T)^{-1} =", info_r["residual"])

# ---- 第三问：q = 1 的函数观测器（R = I3，极点 -4）-----------------------
# (F, G) 取 ([-4], [2, 0])：F Hurwitz、单输入必能控、lambda(F) 不在 lambda(A) 里
F_ob = np.array([[-4.0]])
G_ob = np.array([[2.0, 0.0]])
M_c, N_c, T_c, H_c, info_c = cal_function_observer(
    A, B, C, np.eye(3), F_ob, G_ob)
print("\n[函数观测器] F =", F_ob, " G =", G_ob)
print("T =", exact(T_c), " H = T B =", exact(H_c), sep="\n")
print("Phi = [T; C] 的秩 =", info_c["rank_Phi"], "（应为 3）")
print("M =", exact(M_c).T, " N =", exact(N_c), sep="\n")
print("x̂ = M z + N y；校验 ||M T + N C - I3|| =", info_c["residual_output"])

# ---- 仿真：三个观测器一起看误差收敛 -------------------------------------
q_f, q_c = F_r.shape[0], F_ob.shape[0]


def rhs(tt, state):
    x = state[:3]
    xh = state[3:6]
    zr = state[6:6 + q_f]
    zc = state[6 + q_f:6 + q_f + q_c]
    u = np.array([np.sin(tt)])            # 随便给个输入，观测器不挑输入
    y = C @ x
    dx = A @ x + B @ u
    dxh = F_f @ xh + G_f @ y + H_f @ u
    dzr = F_r @ zr + G_r @ y + H_r @ u
    dzc = F_ob @ zc + G_ob @ y + H_c @ u
    return np.concatenate([dx, dxh, dzr, dzc])


x0 = np.array([1.0, -2.0, 0.5])
z0 = np.zeros(3 + q_f + q_c)
T_END = 6.0
ts = np.linspace(0.0, T_END, 3001)
sol = solve_ivp(rhs, (0.0, T_END), np.concatenate([x0, z0]), t_eval=ts,
                rtol=1e-10, atol=1e-12)
X = sol.y[:3]
XH = sol.y[3:6]
XR = M_r @ sol.y[6:6 + q_f] + N_r @ (C @ X)
XC = M_c @ sol.y[6 + q_f:6 + q_f + q_c] + N_c @ (C @ X)
e_f = np.linalg.norm(X - XH, axis=0)
e_r = np.linalg.norm(X - XR, axis=0)
e_c = np.linalg.norm(X - XC, axis=0)
print("\n仿真：x(0) =", x0, "，u = sin t，三个观测器初值均为 0")
print("末端误差 ||x - x̂||：全维 %.3e  降维 %.3e  函数 %.3e"
      % (e_f[-1], e_r[-1], e_c[-1]))

fig, axes = plt.subplots(1, 2, figsize=(10.4, 4.2))
axes[0].semilogy(ts, np.maximum(e_f, 1e-16), label="全维观测器")
axes[0].semilogy(ts, np.maximum(e_r, 1e-16), label="降维观测器")
axes[0].semilogy(ts, np.maximum(e_c, 1e-16), label="函数观测器 $q=1$")
axes[0].set_xlabel("时间 $t$ / s")
axes[0].set_ylabel("$\\|x-\\hat{x}\\|$")
axes[0].set_title("三种观测器的重构误差")
axes[0].legend()
axes[0].grid(alpha=0.3, which="both")
axes[1].plot(ts, X[0], lw=1.2, label="$x_1$")
axes[1].plot(ts, X[1], lw=1.2, label="$x_2$")
axes[1].plot(ts, X[2], lw=1.2, label="$x_3$")
axes[1].set_xlabel("时间 $t$ / s")
axes[1].set_title("系统状态（$u=\\sin t$）")
axes[1].legend()
axes[1].grid(alpha=0.3)
fig.suptitle("9.6  全维 / 降维 / $q=1$ 函数观测器")
fig.tight_layout()
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fig", "ex_6_observers.png")
os.makedirs(os.path.dirname(out), exist_ok=True)
fig.savefig(out, dpi=150)
print("图已保存：", out)
