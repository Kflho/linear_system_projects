"""9.9  已知观测器的 F，求系数 G、H、M（课件 9.3，(9.32)-(9.34)）。

x' = [[-1,0,1],[2,0,2],[0,1,2]] x + [[0,1],[1,0],[0,1]] u,  y = [[1,0,0],[0,0,1]] x
给定的观测器：z' = F z + G y + H u，x̂ = M z，其中 F = [[-2,0,0],[0,-2,2],[0,-2,-2]]。

条件是（9.32)-(9.34) 里 N = 0 的版本：

    H = T B,   T A - F T = G C,   M T = I_n

未知的是 T (3x3)、G (3x2)、H (3x2)、M (3x3)，方程数少于未知数，解不唯一；
这里取 T 的前两行为 C 的两行（y 直接给出这两个状态），第三行待定，
由 (T A - F T) 落在 C 的行空间里解出，再回代求 G、H、M。
"""
import os

import matplotlib.pyplot as plt
import numpy as np
import sympy as sp
from scipy.integrate import solve_ivp

from linear_system_utils.cal_matrix_operation.cal_observer.cal_observer_symbolic import (
    cal_observer_from_T,
)

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

A = sp.Matrix([[-1, 0, 1], [2, 0, 2], [0, 1, 2]])
B = sp.Matrix([[0, 1], [1, 0], [0, 1]])
C = sp.Matrix([[1, 0, 0], [0, 0, 1]])
F = sp.Matrix([[-2, 0, 0], [0, -2, 2], [0, -2, -2]])
s = sp.Symbol("s")

print("lambda(A) =", [sp.N(r, 6) for r in sp.nroots(A.charpoly(s).as_expr())])
print("lambda(F) =", [sp.N(r, 6) for r in sp.nroots(F.charpoly(s).as_expr())])
print("两者不相交：", sp.simplify(sp.gcd(A.charpoly(s).as_expr(),
                                       F.charpoly(s).as_expr())) == 1)

# ---- 第一步：定 T。前两行取 C 的两行，第三行 [t, t2, t3] 待定 -----------
c1, t2, t3 = sp.symbols("t tau2 tau3")
T_gen = sp.Matrix([[C[0, 0], C[0, 1], C[0, 2]],
                   [C[1, 0], C[1, 1], C[1, 2]],
                   [c1, t2, t3]])
Delta = sp.expand(T_gen * A - F * T_gen)
nC = C.nullspace()[0]                      # C 的零空间，[1,0,-1]'
eqs = sp.simplify(Delta * nC)              # Delta 必须落在 C 的行空间 → Delta*nC = 0
print("\n(T A - F T) 作用在 null(C) 上：", eqs.T)
sol = sp.solve([sp.Eq(e, 0) for e in eqs], [t2, t3], dict=True)[0]
print("解得：", sol)
T = T_gen.subs({t2: sol[t2], t3: sol[t3], c1: 0})     # 自由参数 t 取 0（最简）
print("取 t = 0：T =", T, " det T =", T.det(), sep="\n")

# ---- 第二步：反求 G、H、M ----------------------------------------------
G, H, M, N, info = cal_observer_from_T(A, B, C, F, T, R=sp.eye(3), use_output=False)
print("\nG =", G, " H = T B =", H, " M = T^{-1} =", M, sep="\n")
print("校验：(T A - F T) - G C =", sp.simplify(info["delta"] - G * C))
print("校验：M T - I3 =", sp.simplify(M * T - sp.eye(3)))
print("校验：x̂ = M z 是 x 的估计（N =", N, "）")

# 自由参数 t 换个数就得到另一组等价解（解不唯一）
print("\n自由参数 t 的影响：")
for val in (0, 1):
    Tv = T_gen.subs({t2: sol[t2], t3: sol[t3], c1: val})
    Gv, Hv, Mv, Nv, infov = cal_observer_from_T(
        A, B, C, F, Tv, R=sp.eye(3), use_output=False)
    print("t =", val, "：T =", Tv.tolist(), " G =", Gv.tolist(),
          " M =", Mv.tolist(), " 残差 =", sp.simplify(infov["delta"] - Gv * C).tolist())

# ---- 第三步：数值验证 --------------------------------------------------
An = np.array(A.tolist(), dtype=float)
Bn = np.array(B.tolist(), dtype=float)
Cn = np.array(C.tolist(), dtype=float)
Fn = np.array(F.tolist(), dtype=float)
Gn = np.array(G.tolist(), dtype=float)
Hn = np.array(H.tolist(), dtype=float)
Mn = np.array(M.tolist(), dtype=float)


def rhs(tt, state):
    x, z = state[:3], state[3:]
    u = np.array([np.sin(tt), np.cos(2 * tt)])
    y = Cn @ x
    return np.concatenate([An @ x + Bn @ u, Fn @ z + Gn @ y + Hn @ u])


ts = np.linspace(0.0, 6.0, 3001)
x0 = np.array([1.0, -2.0, 0.5])
sol = solve_ivp(rhs, (0.0, 6.0), np.concatenate([x0, np.zeros(3)]), t_eval=ts,
                rtol=1e-10, atol=1e-12)
X = sol.y[:3]
XH = Mn @ sol.y[3:]
e = np.linalg.norm(X - XH, axis=0)
print("\n仿真：x(0) =", x0, "，z(0) = 0，u = [sin t, cos 2t]'")
print("末端 ||x - x̂|| =", e[-1], "（初值 %.3f）" % e[0])

fig, ax = plt.subplots(figsize=(7.0, 4.0))
ax.semilogy(ts, np.maximum(e, 1e-16))
ax.set_xlabel("时间 $t$ / s")
ax.set_ylabel("$\\|x-\\hat{x}\\|$")
ax.set_title("9.9  给定 $F$ 的观测器：重构误差（$\\lambda(F)=\\{-2,-2\\pm2j\\}$）")
ax.grid(alpha=0.3, which="both")
fig.tight_layout()
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fig", "ex_9_observer.png")
os.makedirs(os.path.dirname(out), exist_ok=True)
fig.savefig(out, dpi=150)
print("图已保存：", out)
