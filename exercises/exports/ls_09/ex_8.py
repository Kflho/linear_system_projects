"""9.8  吊车系统的降阶状态观测器：系统分解法 vs 函数观测器法（课件 9.2、9.3）。

x' = [[0,1,0],[-w0^2,0,w0^2],[0,0,0]] x + [0,0,1]' u,  y = [1,0,0] x

n=3、p=1 → 降阶观测器 2 维，误差极点 {-2w0, -3w0}。注意 lambda(A) = {0, ±jw0}，
观测器极点与之不相交（函数观测器解 Sylvester 方程需要这一点）。
"""
import os

import matplotlib.pyplot as plt
import numpy as np
import sympy as sp
from scipy.integrate import solve_ivp

from linear_system_utils.cal_matrix_operation.cal_observer.cal_observer_symbolic import (
    cal_function_observer_symbolic,
    cal_reduced_order_observer_symbolic,
)

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

w0 = sp.Symbol("w0", positive=True)
A = sp.Matrix([[0, 1, 0], [-w0**2, 0, w0**2], [0, 0, 0]])
B = sp.Matrix([0, 0, 1])
C = sp.Matrix([[1, 0, 0]])
poles = [-2 * w0, -3 * w0]

print("lambda(A) =", A.eigenvals(), "  期望误差极点 =", poles)

# ---- 方法一：基于系统分解（课件 9.2，(9.16)-(9.23)）---------------------
F1, G1, H1, M1, N1, info1 = cal_reduced_order_observer_symbolic(A, B, C, poles)
print("\n[方法一 系统分解]")
print("C0 =", info1["C0"], " T0 =", info1["T0"], sep="\n")
print("A11 =", info1["A11"], " A12 =", info1["A12"],
      " A21 =", info1["A21"], " A22 =", info1["A22"], sep="\n")
print("B1 =", info1["B1"], " B2 =", info1["B2"], sep="\n")
print("L21 =", info1["L21"].T, sep="\n")
print("F =", F1, " G =", G1, " H =", H1, sep="\n")
print("M =", M1, " N =", N1, sep="\n")
print("极点核对：", info1["poles_ok"], "  重构恒等式核对：", info1["recon_ok"])
print("观测器：z' = F z + G y + H u,  x̂ = M z + N y")

# ---- 方法二：函数观测器（课件 9.3，(9.31)-(9.34)）-----------------------
# 取 (F, G) 与法一相同：F Hurwitz、可控，且 lambda(F) ∩ lambda(A) = 空
T2, H2, M2, N2, info2 = cal_function_observer_symbolic(A, B, C, sp.eye(3), F1, G1)
print("\n[方法二 函数观测器] 取 F、G 与法一相同")
print("T =", T2, " H = T B =", H2, sep="\n")
print("Phi = [T; C] =", info2["Phi"], " det =", info2["Phi_det"], sep="\n")
print("M =", M2, " N =", N2, sep="\n")
print("F、G、H、M、N 与法一逐项相等：",
      all(sp.simplify(x - y) == sp.zeros(*x.shape)
          for x, y in [(H1, H2), (M1, M2), (N1, N2)]))

# ---- 换一组 (F, G)：同一系统的另一个等价降阶观测器 ----------------------
F3 = sp.Matrix([[0, 1], [-6 * w0**2, -5 * w0]])       # 仍以 -2w0、-3w0 为极点
G3 = sp.Matrix([0, 1])
T3, H3, M3, N3, info3 = cal_function_observer_symbolic(A, B, C, sp.eye(3), F3, G3)
print("\n[方法二·另一组 (F,G)]")
print("F =", F3, " G =", G3.tolist(), " det Phi =", info3["Phi_det"], sep="\n")
print("T =", T3, " H =", H3.T, sep="\n")
print("M =", M3, " N =", N3, sep="\n")

# ---- 数值验证：取 w0 = 1，看两个观测器的误差都收敛 ----------------------
w0v = 1.0
An = np.array(A.subs(w0, w0v).tolist(), dtype=float)
Bn = np.array(B.tolist(), dtype=float)
Cn = np.array(C.tolist(), dtype=float)
obs = {
    "分解法（同 (F,G) 的函数观测器）": (F1, G1, H1, M1, N1),
    "函数观测器·companion (F,G)": (F3, G3, H3, M3, N3),
}
errs = {}
ts = np.linspace(0.0, 6.0, 3001)
x0 = np.array([1.0, -2.0, 0.5])
for name, (F, G, H, M, N) in obs.items():
    Fn = np.array(F.subs(w0, w0v).tolist(), dtype=float)
    Gn = np.array(G.subs(w0, w0v).tolist(), dtype=float).reshape(2, 1)
    Hn = np.array(H.subs(w0, w0v).tolist(), dtype=float).reshape(2, 1)
    Mn = np.array(M.subs(w0, w0v).tolist(), dtype=float)
    Nn = np.array(N.subs(w0, w0v).tolist(), dtype=float)

    def rhs(tt, state, Fn=Fn, Gn=Gn, Hn=Hn):
        x, z = state[:3], state[3:]
        u = np.array([np.sin(2 * tt)])
        return np.concatenate([An @ x + Bn.ravel() * u[0],
                               Fn @ z + Gn.ravel() * (Cn @ x)[0] + Hn.ravel() * u[0]])

    sol = solve_ivp(rhs, (0.0, 6.0), np.concatenate([x0, np.zeros(2)]), t_eval=ts,
                    rtol=1e-10, atol=1e-12)
    X = sol.y[:3]
    XH = Mn @ sol.y[3:] + Nn @ (Cn @ X)
    errs[name] = np.linalg.norm(X - XH, axis=0)
    print("w0=1，%s：末端 ||x - x̂|| = %.2e" % (name, errs[name][-1]))

fig, ax = plt.subplots(figsize=(7.0, 4.0))
styles = ["-", "--"]
for (name, e), ls in zip(errs.items(), styles):
    ax.semilogy(ts, np.maximum(e, 1e-16), ls, lw=2.2, label=name)
ax.set_xlabel("时间 $t$ / s")
ax.set_ylabel("$\\|x-\\hat{x}\\|$")
ax.set_title("9.8  降阶观测器（$w_0=1$，误差极点 $-2w_0,-3w_0$；两条曲线完全重合）")
ax.legend()
ax.grid(alpha=0.3, which="both")
fig.tight_layout()
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fig", "ex_8_observers.png")
os.makedirs(os.path.dirname(out), exist_ok=True)
fig.savefig(out, dpi=150)
print("图已保存：", out)
