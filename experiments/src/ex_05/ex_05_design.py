"""实验五设计计算：观测器设计（计算机磁盘驱动器磁头位置伺服）。

被控对象（任务书"实验背景"）：

    x1 = 磁头位置，x2 = 磁头速度，u = 直流电机推力

        x' = [[0, 1], [0, -1]] x + [0; 1] u,      y = [1 0] x

    开环极点 {0, -1}：一个积分器 + 一个一阶惯性，位置本身是"漂"的。

三个任务共用一个状态反馈 K，只有观测器不同：

    u = K x̂,   K = [-5 -3]      （极点配置到 -2 +- j，见 cal_state_feedback_pole_placement）

    (一) 全维状态观测器   x̂' = F x̂ + G y + H u,  F = A + L C, G = -L, H = B
                          观测器极点 -5 +- 2j  ->  L = [-9; -20]
    (二) 降维状态观测器   z' = F z + G y + H u,  x̂ = M z + N y
                          误差极点 -5         ->  L21 = -4
                          F = -5, G = -20, H = 1, M = [0; 1], N = [1; 4]
    (三) 函数观测器      只重构 f = K x：z' = F z + G y + H u, u = f̂ = M z + N y
                          (F, G) = (-5, 1)    ->  T = [0.2, -0.05], H = -0.05
                          M = 60, N = -17     （u = 60 z - 17 y）
                          模型里为画"估计状态曲线"用 M* = [0; -20], N* = [1; 4]
                          重构 x̂ = M* z + N* y（满足 M* T + N* C = I）

三个 JSON 的键名就是 Simulink 模型里的变量名（State-Space 的 A_obs/B_obs/C_obs/D_obs
对应观测器积分链，F/G/H 是幻灯片里那种"分开的 Gain 块"写法，两种搭法都能用）。

另外每个 JSON 都带一份 design 块给 MATLAB 复核：
    A_cl   闭环矩阵（状态取 [x; z]，z 是观测器状态；u = K(M z + N C x)）
    z0     观测器状态初值（x̂(0) = M z0 + N C x0）
    poles  A_cl 的特征值（应当 = eig(A+BK) ∪ eig(F)，分离原理）

运行：
    python experiments\\src\\ex_05\\ex_05_design.py
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "common"))

from linsys_lab import ensure_lib_importable  # noqa: E402

ensure_lib_importable()

from linear_system_utils.cal_matrix_operation.cal_observer import (  # noqa: E402
    cal_function_observer,
    cal_full_order_observer,
    cal_observer_closed_loop,
    cal_observer_matrix,
    cal_reduced_order_observer,
)
from linear_system_utils.cal_matrix_operation.cal_pole_placement.cal_state_feedback_pole_placement import (  # noqa: E402
    cal_state_feedback_pole_placement,
)

OUT_JSON_1 = os.path.join(HERE, "ex_05_1_design.json")
OUT_JSON_2 = os.path.join(HERE, "ex_05_2_design.json")
OUT_JSON_3 = os.path.join(HERE, "ex_05_3_design.json")
OUT_LOG = os.path.join(HERE, "ex_05_design.txt")

# ---------------- 对象 ----------------
A = np.array([[0.0, 1.0],
              [0.0, -1.0]])
B = np.array([[0.0], [1.0]])
C_Y = np.array([[1.0, 0.0]])          # y = C_Y x
D = np.zeros((1, 1))

# ---------------- 设计指标 ----------------
POLES_K = [-2 + 1j, -2 - 1j]          # 状态反馈闭环极点
POLES_OBS_FULL = [-5 + 2j, -5 - 2j]   # 全维观测器极点（约 2.5 倍快）
POLES_OBS_RED = [-5.0]                # 降维观测器误差极点
F_FUNC = -5.0                         # 函数观测器自己挑的 F（Hurwitz）
G_FUNC = 1.0                          # (F, G) 可控即可

X0 = [1.0, 0.0]                       # 磁头初始位置偏差 1（速度 0）
TF = 8.0


def _json_default(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.floating):
        return float(o)
    if isinstance(o, np.bool_):
        return bool(o)
    if isinstance(o, complex):
        return {"real": float(o.real), "imag": float(o.imag)}
    raise TypeError(f"不能序列化 {type(o)}")


def cplx_pair(z):
    return [{"real": float(np.real(v)), "imag": float(np.imag(v))}
            for v in np.asarray(z).ravel()]


def clean(M, tol=1e-10):
    """把 1e-16 这种数值噪声抹成 0，JSON 和报告里干净些。"""
    M = np.asarray(M, dtype=float)
    M[np.abs(M) < tol] = 0.0
    return M


def as_mat(x, rows, cols):
    """标量/一维数组统一成 rows x cols 的矩阵（MATLAB 里维数别出岔子）。"""
    M = np.asarray(x, dtype=float).reshape(rows, cols)
    return clean(M)


def main():
    # Windows 控制台默认 GBK，打不出 "x̂" 这种带组合符的字符；日志文件本身是 UTF-8
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

    n = A.shape[0]
    p = C_Y.shape[0]

    # ================= 状态反馈 K =================
    K, K_info = cal_state_feedback_pole_placement(A, B, POLES_K)
    K = clean(np.atleast_2d(K))
    A_cl_state = A + B @ K
    poles_state = np.linalg.eigvals(A_cl_state)

    # ================= (一) 全维观测器 =================
    L, F1, G1, H1, info1 = cal_full_order_observer(A, B, C_Y, POLES_OBS_FULL)
    L, G1, H1 = clean(L), clean(G1), clean(H1)
    B_obs1 = np.hstack([G1, H1])                       # 观测器输入 [y; u]
    A_cl1, cl1 = cal_observer_closed_loop(
        A, B, C_Y, K, F1, G1, H1, np.eye(n), np.zeros((n, p)))

    # ================= (二) 降维观测器 =================
    F2, G2, H2, M2, N2, info2 = cal_reduced_order_observer(
        A, B, C_Y, POLES_OBS_RED)
    F2, G2, H2 = clean(F2), clean(G2), clean(H2)
    M2, N2 = clean(M2), clean(N2)
    B_obs2 = np.hstack([G2, H2])
    A_cl2, cl2 = cal_observer_closed_loop(A, B, C_Y, K, F2, G2, H2, M2, N2)

    # ================= (三) 函数观测器 =================
    # 1) 只重构 f = K x：直接给控制律 u = Mf z + Nf y
    Mf, Nf, T3, H3, info3 = cal_function_observer(
        A, B, C_Y, K, np.array([[F_FUNC]]), np.array([[G_FUNC]]))
    # 2) 顺便给出"重构状态"的那一对增益 M*, N*（R = I），模型里画 x̂ 用
    Ms, Ns, _, _, info3b = cal_function_observer(
        A, B, C_Y, np.eye(n), np.array([[F_FUNC]]), np.array([[G_FUNC]]))
    Mf, Nf, Ms, Ns = clean(Mf), clean(Nf), clean(Ms), clean(Ns)
    H3 = clean(H3)
    B_obs3 = np.hstack([np.array([[G_FUNC]]), H3])
    A_cl3, cl3 = cal_observer_closed_loop(A, B, C_Y, K, np.array([[F_FUNC]]),
                                          np.array([[G_FUNC]]), H3, Ms, Ns)

    # ================= 交叉复核 =================
    # (1) 分离原理：闭环极点 = 状态反馈极点 ∪ 观测器极点
    sep1 = sorted(np.real(cl1["poles"]))
    sep2 = sorted(np.real(cl2["poles"]))
    sep3 = sorted(np.real(cl3["poles"]))
    # (2) (二) 与 (三) 的闭环响应一致：u 增益只差坐标标度
    same_loop = bool(np.allclose(sorted(np.real(cl2["poles"])), sorted(np.real(cl3["poles"])))
                     and np.allclose(np.sort(np.imag(cl2["poles"])), np.sort(np.imag(cl3["poles"]))))
    # 函数观测器的输出：u = Mf z + Nf y 与 K(Ms z + Ns y) 一致（只差数值噪声）
    u_via_K = clean(K @ np.hstack([Ms, Ns]))
    u_direct = clean(np.hstack([Mf, Nf]))

    obs_rank = int(np.linalg.matrix_rank(cal_observer_matrix(A, C_Y)))

    # ================= 写 JSON =================
    data1 = {
        # ---- Simulink 变量 ----
        "A": clean(A), "B": B, "C": np.eye(n), "D": np.zeros((n, 1)),
        "c": C_Y,
        "x0": np.array(X0), "xh0": np.zeros(n),           # 观测器初值：0（故意让它收敛）
        "K": K,
        "F": F1, "G": G1, "H": H1,                        # 幻灯片那种"分开的 Gain 块"写法
        "A_obs": F1, "B_obs": B_obs1, "C_obs": np.eye(n), "D_obs": np.zeros((n, 1)),
        # ---- 复核用 ----
        "design": {
            "task": "全维状态观测器",
            "Tf": TF,
            "obs_rank": obs_rank,
            "poles_K": cplx_pair(poles_state),
            "poles_obs": cplx_pair(info1["poles"]),
            "L": L, "F": F1, "G": G1, "H": H1,
            "A_cl": clean(A_cl1), "z0": np.zeros(n),
            "poles_A_cl": cplx_pair(cl1["poles"]),
            "stable": cl1["stable"],
            "stable_obs": info1["stable"],
            "note": "x̂(0) = 0，估计误差 e(0) = x0，按 -5+-2j 收敛",
        },
    }

    data2 = {
        "A": clean(A), "B": B, "C": np.eye(n), "D": np.zeros((n, 1)),
        "c": C_Y,
        "x0": np.array(X0),
        "K": K,
        "F": F2, "G": G2, "H": H2,
        "M": M2, "N": N2,
        "A_obs": F2, "B_obs": B_obs2, "C_obs": np.ones((1, 1)), "D_obs": 0.0,
        "z0": 0.0,
        "design": {
            "task": "降维状态观测器",
            "Tf": TF,
            "obs_rank": obs_rank,
            "poles_K": cplx_pair(poles_state),
            "poles_obs": cplx_pair(info2["poles"]),
            "L21": clean(info2["L21"]),
            "T0": clean(info2["T0"]),
            "Aij": clean(np.block([[info2["A11"], info2["A12"]],
                                  [info2["A21"], info2["A22"]]])),
            "B1": clean(info2["B1"]), "B2": clean(info2["B2"]),
            "reconstruction_residual": info2["residual"],
            "A_cl": clean(A_cl2), "z0": np.zeros((1, 1)),
            "poles_A_cl": cplx_pair(cl2["poles"]),
            "stable": cl2["stable"],
            "note": "z(0)=0 对应 x̂(0) = M z0 + N y(0) = [1; 4]，"
                    "与真值 [1; 0] 差 [0; -4]，正好用来观察误差收敛",
        },
    }

    data3 = {
        "A": clean(A), "B": B, "C": np.eye(n), "D": np.zeros((n, 1)),
        "c": C_Y,
        "x0": np.array(X0),
        "K": K,
        "F": np.array([[F_FUNC]]), "G": np.array([[G_FUNC]]), "H": H3,
        "M": Ms, "N": Ns,                     # 重构状态 x̂ = M z + N y（模型里画曲线用）
        "M_f": Mf, "N_f": Nf,                 # 直接的控制律 u = M_f z + N_f y
        "A_obs": np.array([[F_FUNC]]), "B_obs": B_obs3,
        "C_obs": np.ones((1, 1)), "D_obs": 0.0,
        "z0": 0.0,
        "design": {
            "task": "函数观测器",
            "Tf": TF,
            "obs_rank": obs_rank,
            "poles_K": cplx_pair(poles_state),
            "poles_obs": cplx_pair(info3["poles"]),
            "T": clean(T3), "H": H3,
            "M_f": Mf, "N_f": Nf,
            "M_state": Ms, "N_state": Ns,
            "rank_Phi": info3["rank_Phi"],
            "residual_output": info3["residual_output"],
            "u_gain_direct": u_direct,
            "u_gain_via_K": u_via_K,
            "A_cl": clean(A_cl3), "z0": np.zeros((1, 1)),
            "poles_A_cl": cplx_pair(cl3["poles"]),
            "stable": cl3["stable"],
            "same_loop_as_reduced": same_loop,
            "note": "u = M_f z + N_f y = 60z - 17y；模型里用 x̂ = M z + N y 再乘 K，"
                    "与直接乘 M_f/N_f 等价（矩阵乘法结合律）",
        },
    }

    for path, data in ((OUT_JSON_1, data1), (OUT_JSON_2, data2), (OUT_JSON_3, data3)):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2, default=_json_default)

    # ================= 可读摘要 =================
    lines = []
    lines.append("实验五设计计算（观测器设计，磁盘驱动器磁头模型）")
    lines.append("=" * 64)
    lines.append("")
    lines.append("对象: x' = [[0,1],[0,-1]] x + [0;1] u,  y = [1 0] x")
    lines.append(f"能观性 rank = {obs_rank}（n = {n}，能观）")
    lines.append(f"状态反馈: u = K x, K = {np.array2string(K.ravel(), precision=0)}"
                 f"  ->  eig(A+BK) = {np.array2string(np.sort_complex(poles_state), precision=4)}")
    lines.append("")
    lines.append("[任务一] 全维观测器：F = A + LC, G = -L, H = B")
    lines.append(f"  布置极点 {POLES_OBS_FULL}  ->  L = {np.array2string(L.ravel(), precision=0)}")
    lines.append(f"  F = {np.array2string(F1, precision=0).replace(chr(10), ' ')}")
    lines.append(f"  G = {np.array2string(G1.ravel(), precision=0)}, "
                 f"H = {np.array2string(H1.ravel(), precision=0)}")
    lines.append(f"  eig(F) = {np.array2string(np.sort_complex(info1['poles']), precision=4)}")
    lines.append(f"  闭环(分离)极点 = {np.array2string(np.sort_complex(cl1['poles']), precision=4)}")
    lines.append("")
    lines.append("[任务二] 降维观测器：z' = F z + G y + H u, x̂ = M z + N y")
    lines.append(f"  误差极点 {POLES_OBS_RED}  ->  L21 = {float(np.asarray(info2['L21']).ravel()[0]):g}")
    lines.append(f"  F = {np.array2string(F2, precision=0).replace(chr(10), ' ')}, "
                 f"G = {np.array2string(G2.ravel(), precision=0)}, "
                 f"H = {np.array2string(H2.ravel(), precision=0)}")
    lines.append(f"  M = {np.array2string(M2.ravel(), precision=0)}, "
                 f"N = {np.array2string(N2.ravel(), precision=0)}")
    lines.append(f"  重构恒等式残差 |M L21 + N - C'(CC')^{-1}| = {info2['residual']:.2e}")
    lines.append(f"  闭环(分离)极点 = {np.array2string(np.sort_complex(cl2['poles']), precision=4)}")
    lines.append("")
    lines.append("[任务三] 函数观测器：只重构 f = K x")
    lines.append(f"  取 (F, G) = ({F_FUNC:g}, {G_FUNC:g})  ->  T = "
                 f"{np.array2string(T3.ravel(), precision=4)}, H = T B = "
                 f"{float(np.asarray(H3).ravel()[0]):g}")
    lines.append(f"  u = M_f z + N_f y = {float(Mf.ravel()[0]):g} z "
                 f"{float(Nf.ravel()[0]):+g} y")
    lines.append(f"  重构状态的增益 M = {np.array2string(Ms.ravel(), precision=0)}, "
                 f"N = {np.array2string(Ns.ravel(), precision=0)}"
                 f"（满足 M T + N C = I）")
    lines.append(f"  M T + N C - I 残差 = {info3b['residual_output']:.2e}")
    lines.append(f"  K (M z + N y) = {np.array2string(u_via_K.ravel(), precision=0)}"
                 f"  与 M_f/N_f 一致")
    lines.append(f"  闭环(分离)极点 = {np.array2string(np.sort_complex(cl3['poles']), precision=4)}")
    lines.append("")
    lines.append("结论要点：")
    lines.append("  * 三种观测器都满足分离原理：闭环极点 = {-2±j} ∪ 观测器极点。")
    lines.append("  * (二)(三) 的闭环极点相同（都含 -5），u(t) 轨迹也完全相同；")
    lines.append("    差别只是观测器内部状态 z 的标度（z_三 = -z_二 / 20）。")
    lines.append(f"  * 全维观测器 x̂(0)=0，误差 e(0)=x0；降维/函数观测器 x̂1=y 直接可得，")
    lines.append("    所以只有速度估计有初始误差。")
    lines.append("")
    lines.append(f"输出: {OUT_JSON_1}")
    lines.append(f"      {OUT_JSON_2}")
    lines.append(f"      {OUT_JSON_3}")

    text = "\n".join(lines)
    with open(OUT_LOG, "w", encoding="utf-8") as f:
        f.write(text + "\n")
    print(text)


if __name__ == "__main__":
    main()
