"""实验三设计计算：倒立摆输出调节（静态状态反馈），跟踪 r(t) = 0.1 cos 2t。

被控对象（实验指导书）：

    x' = A x + B u,    y = [1 0 0 0] x

        [0    1    0      0]        [ 0    ]
    A = [0    0  -3.267   0]    B = [ 0.444]
        [0    0    0      1]        [ 0    ]
        [0    0   19.6    0]        [-0.667]

参考输入由外系统产生，取 w(0) = [1, 0]'：

    w' = A_v w,  A_v = [0 2; -2 0]   ->  w = [cos 2t; -sin 2t]
    r  = [0.1 0] w = 0.1 cos 2t

设计（静态状态反馈输出调节）三步，算法全在 Python 仓的 linear_system_utils：

    1) 极点配置定 K（u = K x，闭环 A + B K 极点为 P）：
       cal_matrix_operation.cal_pole_placement.cal_state_feedback_pole_placement
    2) 解调节器方程           Pi A_v = A Pi + B Gamma,  C Pi = Q   （C = [1 0 0 0], Q = [0.1 0]）
    3) 取 K_v = Gamma - K Pi，控制律 u = K x + K_v w：
       cal_matrix_operation.cal_output_regulation

Simulink 模型（ex_03_sim.slx）用的变量名就是本脚本写出去的键：
    A, B, C, D   State-Space1（C 取 eye(4)，输出全部状态，C=In）
    c            y = c x
    A_v ... Initial_v   参考发生器 State-Space
    r            r = r * w
    k_x, k_v     两个增益
模型里参考发生器的输入口是空的（外系统自治），ex_03_run_sim.m 会在内存里补一个
Constant 0 再仿真，磁盘上的 .slx 不动。
"""
import json
import os

import numpy as np

from linear_system_utils.cal_matrix_operation.cal_output_regulation import (
    cal_output_regulation_gain,
    cal_regulation_solvability,
)
from linear_system_utils.cal_matrix_operation.cal_pole_placement.cal_state_feedback_pole_placement import (
    cal_state_feedback_pole_placement,
)

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_JSON = os.path.join(HERE, "ex_03_design.json")
OUT_LOG = os.path.join(HERE, "ex_03_design.txt")

# 闭环极点：-5 +- 3j、-5 +- 2j（实部 -5 左右，阻尼 0.86 / 0.93，比开环快得多又不过冲）
POLES = [-5 + 3j, -5 - 3j, -5 + 2j, -5 - 2j]

# 课件 lab34 第 6 页图上标的 kx，只拿来做对照（我们按 P 自己算）
KX_HANDOUT = np.array([[150.9184, 96.4286, 374.5122, 94.2857]])
KV_HANDOUT = np.array([[-3.6786, -13.2353]])


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


def cplx_pairs(z):
    """复数数组 -> [{'real':.., 'imag':..}, ...]，JSON 用"""
    return [{"real": float(np.real(v)), "imag": float(np.imag(v))} for v in np.asarray(z).ravel()]


def main():
    # ---------------- 模型 ----------------
    A = np.array([[0, 1, 0, 0],
                  [0, 0, -3.267, 0],
                  [0, 0, 0, 1],
                  [0, 0, 19.6, 0]], dtype=float)
    B = np.array([[0], [0.444], [0], [-0.667]], dtype=float)
    C_y = np.array([[1, 0, 0, 0]], dtype=float)          # 真正的输出 y = C_y x
    D_y = np.zeros((1, 1))
    n = A.shape[0]

    # State-Space1 的 C/D：取 eye(4) 让它输出全部状态（课件" C = In "），y 用 c 提取
    C_model = np.eye(n)
    D_model = np.zeros((n, 1))

    # 外系统：w = [cos2t; -sin2t]
    A_v = np.array([[0.0, 2.0], [-2.0, 0.0]])
    B_v = np.zeros((2, 1))
    C_v = np.eye(2)
    D_v = np.zeros((2, 1))
    INIT_V = np.array([1.0, 0.0])
    R_GAIN = np.array([[0.1, 0.0]])                       # r = 0.1 w_1

    Q = R_GAIN @ C_v                                      # 参考输出系数，r = Q w

    # ---------------- 1) 极点配置定 K ----------------
    K, info_pp = cal_state_feedback_pole_placement(A, B, POLES)

    # ---------------- 2)3) 调节器方程 + 前馈增益 ----------------
    Kv, Pi, Gamma, info_or = cal_output_regulation_gain(A, B, C_y, A_v, K, Q=Q)
    solv = cal_regulation_solvability(A, B, C_y, A_v)

    # 稳态量的量级（报告里用来解释"为什么 u 的稳态幅值只有 1.1 左右"）
    x_ss_amp = float(np.max(np.abs(Pi @ np.array([[1.0], [0.0]]))) / 1.0)   # w 幅值 1
    u_ss_amp = float(np.max(np.abs(Gamma @ np.array([[1.0], [0.0]]))))

    cls_poles = np.linalg.eigvals(A + B @ K)

    data = {
        # ---- Simulink 变量（模型直接用这些名字）----
        "A": A, "B": B, "C": C_model, "D": D_model,
        "c": C_y,
        "A_v": A_v, "B_v": B_v, "C_v": C_v, "D_v": D_v, "Initial_v": INIT_V,
        "r": R_GAIN,
        "k_x": K, "k_v": Kv,
        # ---- 设计/复核 ----
        "design": {
            "poles": cplx_pairs(POLES),
            "closed_loop_poles": cplx_pairs(cls_poles),
            "nu": info_pp["nu"],
            "Pi": Pi, "Gamma": Gamma, "Q": Q,
            "k_x_handout": KX_HANDOUT, "k_v_handout": KV_HANDOUT,
            "residual_regulator": info_or["residual"],
            "residual_closed_loop": info_or["residual_closed_loop"],
            "residual_output": info_or["residual_output"],
            "stable": info_or["stable"],
            "solvable": solv["ok"],
            "solvability_rank": solv["rank"],
            "solvability_need": solv["need"],
            "x_ss_amp": x_ss_amp,
            "u_ss_amp": u_ss_amp,
            "exo_freq": 2.0,
            "amp_r": 0.1,
        },
    }

    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2, default=_json_default)

    # ---------------- 可读摘要 ----------------
    lines = []
    lines.append("实验三设计计算（倒立摆输出调节 · 静态状态反馈）")
    lines.append("=" * 60)
    lines.append("")
    lines.append("[对象] x' = A x + B u,  y = [1 0 0 0] x")
    lines.append("A =")
    for row in A:
        lines.append("    " + "  ".join(f"{v:9.4g}" for v in row))
    lines.append("B =")
    for row in B:
        lines.append("    " + "  ".join(f"{v:9.4g}" for v in row))
    lines.append("")
    lines.append("[参考] w' = A_v w, w(0) = [1 0]'  ->  w = [cos2t; -sin2t],  r = 0.1 w_1 = 0.1 cos2t")
    lines.append("A_v =")
    for row in A_v:
        lines.append("    " + "  ".join(f"{v:9.4g}" for v in row))
    lines.append("")
    lines.append("[1] 极点配置（课件 4.4 公式）")
    lines.append(f"  期望极点 P = {np.array2string(np.array(POLES), precision=4)}")
    lines.append(f"  能控性指标 nu = {info_pp['nu']}")
    lines.append("  K =")
    for row in K:
        lines.append("      " + "  ".join(f"{v:11.4f}" for v in row))
    lines.append(f"  闭环极点 = {np.array2string(np.sort_complex(cls_poles), precision=4)}")
    lines.append(f"  课件 kx  = {np.array2string(KX_HANDOUT.ravel(), precision=4)}")
    lines.append(f"  与课件 kx 相对差 = {np.linalg.norm(K - KX_HANDOUT) / np.linalg.norm(KX_HANDOUT):.2%}"
                 "（课件极点在 -4.73+-2.84j、-5.31+-2.05j，与本设计几乎相同）")
    lines.append("")
    lines.append("[2] 调节器方程  Pi A_v = A Pi + B Gamma,  C Pi = Q")
    lines.append("  Pi =")
    for row in Pi:
        lines.append("      " + "  ".join(f"{v:11.6f}" for v in row))
    lines.append("  Gamma = " + "  ".join(f"{v:11.6f}" for v in Gamma.ravel()))
    lines.append(f"  残差: 方程 {info_or['residual']:.2e}, 闭环形式 {info_or['residual_closed_loop']:.2e},"
                 f" C Pi - Q {info_or['residual_output']:.2e}")
    lines.append(f"  能调节性判据 rank([A-lambda I, B; C, 0]) @ lambda=+-2j:"
                 f" {solv['rank']} (需要 {solv['need']})  ->  {'通过' if solv['ok'] else '不通过'}")
    lines.append("")
    lines.append("[3] 静态输出调节增益  K_v = Gamma - K Pi")
    lines.append("  K_v = " + "  ".join(f"{v:11.4f}" for v in Kv.ravel()))
    lines.append(f"  课件图上 kv = {np.array2string(KV_HANDOUT.ravel(), precision=4)}"
                 "（不满足调节器方程，见 bin/references/ex_03.md）")
    lines.append(f"  稳定: A+B K 全是左半平面 -> {info_or['stable']}")
    lines.append("")
    lines.append("[4] 量级")
    lines.append(f"  稳态状态幅值 max|Pi w| = {x_ss_amp:.4g}, 稳态控制幅值 max|Gamma w| = {u_ss_amp:.4g}"
                 f"（w 幅值 1）")
    lines.append("")
    lines.append(f"输出: {OUT_JSON}")

    text = "\n".join(lines)
    with open(OUT_LOG, "w", encoding="utf-8") as f:
        f.write(text + "\n")
    print(text)


if __name__ == "__main__":
    main()
