"""实验四设计计算：内模原理。

两个模型（都是手搭的积分链实现，状态顺序由接线决定）：

  ex_04_1_sim.slx —— 倒立摆的输出调节（内模原理），跟踪 r(t) = 0.1cos2t
      对象 x' = A x + B u，y = [1 0 0 0] x
      内模：a' = e - 4b, b' = a           （1/(s^2+4)，e = y - r）
      控制：u = k_x x + kyita2 a + kyita1 b

  ex_04_2_sim.slx —— 渐近跟踪 + 干扰抑制（内模原理）
      对象 x' = [[0,1],[-2,2]] x + [0;1] u + E d,  y = [1 0] x,  d = 1(t), r = 0.1cos2t
      模型里对象是积分链实现的：x1' = d + x2, x2' = 2d + u - 2x1 + 2x2   （E = [1;2]）
      内模：p' = e - 4q, q' = p, s' = q  （1/(s(s^2+4))，同时含 1/s 和 1/(s^2+4)）
      控制：u = kyita3 p + kyita2 q + kyita1 s + k_x1 x1 + k_x2 x2

两个模型现在都自带 Constant 0（外系统自治输入）和变量初值 v_start，可以直接跑；
本脚本输出的键名就是模型里的变量名。

设计流程（算法都在 linear_system_utils.cal_matrix_operation.cal_output_regulation）：
    1) cal_internal_model：按模型实际的 Ac/Bc 拼增广系统 [x; eta]
    2) cal_state_feedback_pole_placement：对增广系统极点配置得 K_aug = [K_x, K_eta]
    3) cal_internal_model_closed_loop：复核闭环极点、稳定性

本脚本输出两个 JSON（键名就是两个 Simulink 模型里的变量名），run_sim 直接灌。
"""
import json
import os

import numpy as np

from linear_system_utils.cal_matrix_operation.cal_output_regulation import (
    cal_internal_model,
    cal_internal_model_closed_loop,
)
from linear_system_utils.cal_matrix_operation.cal_pole_placement.cal_state_feedback_pole_placement import (
    cal_state_feedback_pole_placement,
)

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_JSON_1 = os.path.join(HERE, "ex_04_1_design.json")
OUT_JSON_2 = os.path.join(HERE, "ex_04_2_design.json")
OUT_LOG = os.path.join(HERE, "ex_04_design.txt")

# 闭环极点（都取实数，方便看"最慢的模态有多快"；比参考频率 omega=2 快 1 倍以上）
POLES_1 = [-2, -3, -4, -5, -6, -7]
POLES_2 = [-2, -3, -4, -5, -6]

# ---------------- 公共：外系统（r = 0.1 cos 2t） ----------------
A_V = np.array([[0.0, 2.0], [-2.0, 0.0]])   # w = [cos2t; -sin2t]
B_V = np.zeros((2, 1))
C_V = np.eye(2)
D_V = np.zeros((2, 1))
INIT_V = np.array([1.0, 0.0])
R_GAIN = np.array([[0.1, 0.0]])

# ---------------- 模型 1：倒立摆 + 1/(s^2+4) ----------------
A1 = np.array([[0, 1, 0, 0],
               [0, 0, -3.267, 0],
               [0, 0, 0, 1],
               [0, 0, 19.6, 0]], dtype=float)
B1 = np.array([[0], [0.444], [0], [-0.667]], dtype=float)
C_Y1 = np.array([[1, 0, 0, 0]], dtype=float)
AC1 = np.array([[0.0, -4.0], [1.0, 0.0]])       # 模型接线：a' = e - 4b, b' = a
BC1 = np.array([[1.0], [0.0]])
W1 = -4.0                                        # 图上那个 -4 的增益

# ---------------- 模型 2：2 阶对象 + 干扰 + 1/(s(s^2+4)) ----------------
A2 = np.array([[0.0, 1.0], [-2.0, 2.0]])
B2 = np.array([[0.0], [1.0]])
C_Y2 = np.array([[1.0, 0.0]])
AC2 = np.array([[0.0, -4.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0]])                 # p' = e - 4q, q' = p, s' = q
BC2 = np.array([[1.0], [0.0], [0.0]])
# 模型里对象是用积分链搭的：x1' = d + x2, x2' = 2d + u - 2x1 + 2x2
GAIN1 = 2.0        # d 进 x2' 那一路的权重（x1' 那一路直接接 d，权重 1）-> E = [1;2]
GAIN2 = -2.0       # x1 到 x2' 的系数
GAIN6 = -4.0       # 内模反馈
GAIN7 = 2.0        # x2 到 x2' 的系数
A_D = np.zeros((1, 1))       # 干扰 d = 1(t) 由 0 阶外系统给出
B_D = np.zeros((1, 1))
C_D = np.ones((1, 1))
D_D = np.zeros((1, 1))
D_START = 1.0


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
    return [{"real": float(np.real(v)), "imag": float(np.imag(v))} for v in np.asarray(z).ravel()]


def check_tracking(A_aug, B_aug, C_aug, K_aug, B_ref, C_y, omega=2.0):
    """内模原理的两条零点检查（不看仿真也能判断跟不跟得上）。

    e = C_y x - r：
      * r = 0.1cos(omega t) -> e/r 在 s = +-j omega 处应为 0
      * 常值干扰          -> d->e 在 s = 0 处应为 0（无静差）
    B_ref 是外部参考 r 进增广系统的列向量。
    """
    A_cl = A_aug + B_aug @ K_aug
    n = A_cl.shape[0]
    Cx = np.hstack([C_y, np.zeros((C_y.shape[0], n - C_y.shape[1]))])
    zero_r = abs((Cx @ np.linalg.solve(1j * omega * np.eye(n) - A_cl, B_ref))[0, 0] - 1.0)
    return A_cl, float(zero_r)


def main():
    # ================= 模型 1 =================
    A_aug1, B_aug1, C_aug1, im1 = cal_internal_model(
        A1, B1, C_Y1, A_V, Ac=AC1, Bc=BC1)
    K1, pp1 = cal_state_feedback_pole_placement(A_aug1, B_aug1, POLES_1)
    K1 = np.atleast_2d(K1)
    _, cl1 = cal_internal_model_closed_loop(A1, B1, C_Y1, A_V, K1, Ac=AC1, Bc=BC1)
    B_ref1 = np.vstack([np.zeros((A1.shape[0], 1)), -BC1])     # r 以 -Bc 进入内模
    _, zero_r1 = check_tracking(A_aug1, B_aug1, np.zeros((1, 6)), K1, B_ref1, C_Y1)

    k_x = K1[0, :4].reshape(1, -1)
    kyita2 = float(K1[0, 4])       # 乘 a（第一个积分器输出）
    kyita1 = float(K1[0, 5])       # 乘 b（第二个积分器输出）

    data1 = {
        # ---- Simulink 变量（ex_04_1_sim.slx 直接用）----
        "A": A1, "B": B1,
        "C": np.eye(4), "D": np.zeros((4, 1)),      # State-Space1 输出全部状态（C = In）
        "c": C_Y1,                                   # y = c x
        "A_v": A_V, "B_v": B_V, "C_v": C_V, "D_v": D_V,
        "v_start": INIT_V,
        "r": R_GAIN,
        "k_x": k_x, "kyita2": kyita2, "kyita1": kyita1, "w": W1,
        # ---- 设计/复核 ----
        "design": {
            "poles": cplx_pair(POLES_1),
            "nu": pp1["nu"],
            "K_aug": K1,
            "closed_loop_poles": cplx_pair(cl1["poles"]),
            "stable": cl1["stable"],
            "ctrb_rank": im1["ctrb_rank"], "n_aug": im1["n_aug"],
            "Ac": AC1, "Bc": BC1,
            "zero_at_r": zero_r1,
            "e_ss_amplitude_note": "r 频率 2 rad/s 处 |e/r| 应为 0",
        },
    }

    # ================= 模型 2 =================
    A_aug2, B_aug2, C_aug2, im2 = cal_internal_model(
        A2, B2, C_Y2, np.block([[A_V, np.zeros((2, 1))], [np.zeros((1, 3))]]), Ac=AC2, Bc=BC2)
    K2, pp2 = cal_state_feedback_pole_placement(A_aug2, B_aug2, POLES_2)
    K2 = np.atleast_2d(K2)
    _, cl2 = cal_internal_model_closed_loop(
        A2, B2, C_Y2, np.block([[A_V, np.zeros((2, 1))], [np.zeros((1, 3))]]), K2, Ac=AC2, Bc=BC2)
    B_ref2 = np.vstack([np.zeros((A2.shape[0], 1)), -BC2])
    A_cl2, zero_r2 = check_tracking(A_aug2, B_aug2, np.zeros((1, 5)), K2, B_ref2, C_Y2)
    # 常值干扰 d 的直流增益（B_d_aug 在状态 x 上，权重 E；模型现状 E = [1;1]）
    E_model = np.array([[1.0], [GAIN1]])
    B_d2 = np.vstack([E_model, np.zeros((AC2.shape[0], 1))])
    Cx2 = np.hstack([C_Y2, np.zeros((1, AC2.shape[0]))])
    dc_d2 = float(abs((Cx2 @ np.linalg.solve(-A_cl2, B_d2))[0, 0]))

    data2 = {
        # ---- Simulink 变量（ex_04_2_sim.slx 直接用）----
        "A_v": A_V, "B_v": B_V, "C_v": C_V, "D_v": D_V,
        "v_start": INIT_V,
        "r": R_GAIN,
        "A_d": A_D, "B_d": B_D, "C_d": C_D, "D_d": D_D, "d_start": D_START,
        "gain1": GAIN1, "gain2": GAIN2, "gain6": GAIN6, "gain7": GAIN7,
        "k_x1": float(K2[0, 0]), "k_x2": float(K2[0, 1]),
        "kyita3": float(K2[0, 2]), "kyita2": float(K2[0, 3]), "kyita1": float(K2[0, 4]),
        # ---- 设计/复核 ----
        "design": {
            "poles": cplx_pair(POLES_2),
            "nu": pp2["nu"],
            "K_aug": K2,
            "closed_loop_poles": cplx_pair(cl2["poles"]),
            "stable": cl2["stable"],
            "ctrb_rank": im2["ctrb_rank"], "n_aug": im2["n_aug"],
            "Ac": AC2, "Bc": BC2,
            "zero_at_r": zero_r2,
            "dc_gain_d_to_e": dc_d2,
            "E_model": E_model,  # [1; 2]，与模型接线一致
        },
    }

    for path, data in ((OUT_JSON_1, data1), (OUT_JSON_2, data2)):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2, default=_json_default)

    # ================= 可读摘要 =================
    lines = []
    lines.append("实验四设计计算（内模原理）")
    lines.append("=" * 60)
    lines.append("")
    lines.append("公共：外系统 w' = A_v w, w(0) = [1 0]' -> w = [cos2t; -sin2t]，r = 0.1 w_1")
    lines.append("")
    lines.append("[模型1] 倒立摆 + 内模 1/(s^2+4)：a' = e - 4b, b' = a")
    lines.append(f"  增广系统阶数 = {im1['n_aug']}，能控 rank = {im1['ctrb_rank']}")
    lines.append(f"  期望极点 = {POLES_1}")
    lines.append(f"  K_aug = {np.array2string(K1.ravel(), precision=4)}")
    lines.append(f"  k_x = {np.array2string(k_x.ravel(), precision=4)}")
    lines.append(f"  kyita2(a) = {kyita2:.4f}   kyita1(b) = {kyita1:.4f}   w = {W1:g}")
    lines.append(f"  闭环极点 = {np.array2string(np.sort_complex(cl1['poles']), precision=4)}"
                 f"   稳定 {cl1['stable']}")
    lines.append(f"  跟踪零点检查 |e/r(j2)| = {zero_r1:.2e}（应为 0）")
    lines.append("")
    lines.append("[模型2] 2 阶对象 + d=1(t) + 内模 1/(s(s^2+4))：p' = e - 4q, q' = p, s' = q")
    lines.append(f"  增广系统阶数 = {im2['n_aug']}，能控 rank = {im2['ctrb_rank']}")
    lines.append(f"  期望极点 = {POLES_2}")
    lines.append(f"  K_aug = {np.array2string(K2.ravel(), precision=4)}")
    lines.append(f"  k_x1 = {K2[0, 0]:.4f}  k_x2 = {K2[0, 1]:.4f}")
    lines.append(f"  kyita3(p) = {K2[0, 2]:.4f}  kyita2(q) = {K2[0, 3]:.4f}  kyita1(s) = {K2[0, 4]:.4f}")
    lines.append(f"  对象实现系数：gain1 = {GAIN1:g}(x2' 那路的 d 权重), gain2 = {GAIN2:g}, "
                 f"gain6 = {GAIN6:g}(内模), gain7 = {GAIN7:g}")
    lines.append(f"  闭环极点 = {np.array2string(np.sort_complex(cl2['poles']), precision=4)}"
                 f"   稳定 {cl2['stable']}")
    lines.append(f"  跟踪零点检查 |e/r(j2)| = {zero_r2:.2e}（应为 0）")
    lines.append(f"  干扰零点检查 |d->e 直流增益| = {dc_d2:.2e}（应为 0）")
    lines.append("")
    lines.append("备注：两个模型都已自带 Constant 0（外系统自治输入）和变量初值 v_start，")
    lines.append("      模型2 的 d 拆成两路（x1' 直接 +d，x2' 经 gain1=2），即 E = [1;2] 与任务书一致。")
    lines.append("")
    lines.append(f"输出: {OUT_JSON_1}")
    lines.append(f"      {OUT_JSON_2}")

    text = "\n".join(lines)
    with open(OUT_LOG, "w", encoding="utf-8") as f:
        f.write(text + "\n")
    print(text)


if __name__ == "__main__":
    main()
