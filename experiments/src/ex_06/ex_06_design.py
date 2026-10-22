"""实验六设计计算：观测器与状态反馈 —— 降维观测器 + 内模原理的渐近跟踪。

任务书 (四)：磁盘上的数据不完全在中心圆上，伺服系统必须跟踪一个角频率为
omega0（磁盘自转速度）的正弦输入

    r''(t) = -omega0^2 r(t),      取 omega0 = 1  ->  r(t) = sin t

设计内容（软件里把参考做成自治外系统 w' = A_r w, r = C_r w）：

    1) 降维观测器（与实验五任务二同一个）：z' = F z + G y + H u,  x̂ = M z + N y
       F = -5, G = -20, H = 1, M = [0;1], N = [1;4]（误差极点 -5）

    2) 内模 1/(s^2 + 1)（课件第 7 章 / 9.4 节）：eta' = A_c eta + B_c e,  e = y - r

           A_c = [[0, -1], [1, 0]],   B_c = [1; 0]
           即  eta1' = e - eta2,  eta2' = eta1

       （与实验四的 1/(s^2+4) 是同一套写法，只是 omega0 = 1；模型里那个
        W = -1 的增益就是 A_c 里的 -omega0^2。）

    3) 动态状态反馈 u = K_x x̂ + K_eta eta，对增广系统 [x; eta] 做极点配置：

           增广系统 A_aug = [[A, 0], [B_c C, A_c]],  B_aug = [B; 0]
           期望极点 {-2±j}（原来的状态反馈那对）+ {-1±j}（内模那一对）

       得 K_aug = [K_x, K_eta] = [-14, -5, -12, 4]，
       即 K_x = [-14, -5]，kyita1 = -12 乘 eta1、kyita2 = 4 乘 eta2。

闭环验证：
    * eig(A_cl) = {-2±j, -1±j, -5} ∪ {±j}（±j 是参考外系统自己的极点）
    * 内模原理：从 r 到 e 的传递函数在 s = ±j 处有零点（|e/r(j omega0)| = 0）
    * 内模控制器 C_im(s) = K_eta (sI - A_c)^{-1} B_c = (4 - 12 s)/(s^2 + 1)
      在 omega = 1 处增益无穷大 —— 这就是"内模控制器 Bode 图"要看的东西

本脚本输出 ex_06_1_design.json（键名 = Simulink 变量名）+ 可读摘要。

运行：
    python experiments\\src\\ex_06\\ex_06_design.py
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
    cal_observer_closed_loop,
    cal_reduced_order_observer,
)
from linear_system_utils.cal_matrix_operation.cal_output_regulation import (  # noqa: E402
    cal_internal_model,
    cal_internal_model_closed_loop,
)
from linear_system_utils.cal_matrix_operation.cal_pole_placement.cal_state_feedback_pole_placement import (  # noqa: E402
    cal_state_feedback_pole_placement,
)

OUT_JSON = os.path.join(HERE, "ex_06_1_design.json")
OUT_LOG = os.path.join(HERE, "ex_06_design.txt")

# ---------------- 对象（磁盘驱动器的磁头位置/速度）----------------
A = np.array([[0.0, 1.0],
              [0.0, -1.0]])
B = np.array([[0.0], [1.0]])
C_Y = np.array([[1.0, 0.0]])
D = np.zeros((1, 1))

# ---------------- 参考：r = sin(t) 由自治外系统生成 ----------------
OMEGA0 = 1.0
A_R = np.array([[0.0, 1.0], [-1.0, 0.0]])       # w' = A_r w
B_R = np.zeros((2, 1))
C_R = np.array([[1.0, 0.0]])                     # r = C_r w
D_R = 0.0
W0 = [0.0, 1.0]                                  # w = [sin t; cos t]

# ---------------- 内模 1/(s^2 + 1) ----------------
AC = np.array([[0.0, -1.0], [1.0, 0.0]])         # eta1' = e - eta2
BC = np.array([[1.0], [0.0]])                    # eta2' = eta1
ETA0 = [0.0, 0.0]

# ---------------- 控制指标 ----------------
POLES_K = [-2 + 1j, -2 - 1j]                      # 状态反馈那一对
POLES_AUG = [-2 + 1j, -2 - 1j, -1 + 1j, -1 - 1j]  # 增广系统（状态 + 内模）
POLES_OBS_RED = [-5.0]                            # 降维观测器误差极点

X0 = [1.0, 0.0]                                   # 磁头初始位置偏差 1
Z0 = 0.0                                          # 降维观测器内部状态初值
TF = 20.0                                         # 约 3 个参考周期


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
    M = np.array(M, dtype=float, copy=True)
    M[np.abs(M) < tol] = 0.0
    return M


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

    n = A.shape[0]
    p = C_Y.shape[0]

    # ================= 1) 内模 + 增广系统极点配置 =================
    A_aug, B_aug, C_aug, im = cal_internal_model(A, B, C_Y, A_R, Ac=AC, Bc=BC)
    K_aug, _ = cal_state_feedback_pole_placement(A_aug, B_aug, POLES_AUG)
    K_aug = clean(np.atleast_2d(K_aug))
    _, cl = cal_internal_model_closed_loop(A, B, C_Y, A_R, K_aug, Ac=AC, Bc=BC)

    K_x = clean(K_aug[:, :n])            # 状态反馈部分（乘 x̂）
    K_eta = clean(K_aug[:, n:])          # 内模反馈部分（乘 eta = [eta1; eta2]）
    kyita1, kyita2 = float(K_eta.ravel()[0]), float(K_eta.ravel()[1])

    # ================= 2) 降维观测器 =================
    F, G, H, M, N, info_obs = cal_reduced_order_observer(A, B, C_Y, POLES_OBS_RED)
    F, G, H = clean(F), clean(G), clean(H)
    M, N = clean(M), clean(N)
    B_obs = np.hstack([G, H])

    # ================= 3) 闭环（含参考外系统）=================
    # 状态 z = [x(2); z_obs(1); eta(2); w(2)]，u = K_x (M z_obs + N y) + K_eta eta
    KC = K_x @ N @ C_Y                 # (1, n)
    KM = K_x @ M                       # (1, 1)
    A_cl3, _ = cal_observer_closed_loop(A, B, C_Y, K_x, F, G, H, M, N)

    A_big = np.zeros((n + 1 + 2 + 2, n + 1 + 2 + 2))
    A_big[:3, :3] = A_cl3                                   # [x; z_obs] 闭环
    A_big[:3, 3:5] = np.vstack([B @ K_eta, H @ K_eta])       # eta -> u -> [x; z]
    A_big[3:5, :3] = np.hstack([BC @ C_Y, np.zeros((2, 1))])  # e = C x - r 的 +C 部分
    A_big[3:5, 3:5] = AC
    A_big[3:5, 5:7] = -BC @ C_R                              # 参考以 -Bc 进内模
    A_big[5:7, 5:7] = A_R

    z0_big = np.concatenate([np.asarray(X0, dtype=float),
                             np.atleast_1d(np.asarray(Z0, dtype=float)),
                             np.asarray(ETA0, dtype=float),
                             np.asarray(W0, dtype=float)])
    poles_big = np.linalg.eigvals(A_big)

    # ================= 4) 传递函数 =================
    # 4a) 内模控制器 C_im(s) = K_eta (sI - A_c)^{-1} Bc = (kyita1 s + kyita2)/(s^2 + 1)
    num_im = np.array([[kyita1, kyita2]])
    den_im = np.array([1.0, 0.0, OMEGA0 ** 2])

    # 4b) 从 r 到 e 的闭环（状态 [x; z_obs; eta]，输入 r，输出 e = C x - r）
    A_ctrl = np.block([
        [A + B @ KC, B @ KM, B @ K_eta],
        [G @ C_Y + H @ KC, F + H @ KM, H @ K_eta],
        [BC @ C_Y, np.zeros((2, 1)), AC],
    ])
    B_ctrl = np.vstack([np.zeros((n, 1)),
                        np.zeros((1, 1)),
                        -BC])
    C_e = np.hstack([C_Y, np.zeros((1, 1 + AC.shape[0]))])
    D_e = -1.0

    def H_er(w):
        sol = np.linalg.solve(1j * w * np.eye(A_ctrl.shape[0]) - A_ctrl, B_ctrl)
        return complex((C_e @ sol + D_e).item())

    omega = np.linspace(0.05, 6.0, 1200)
    H_scan = np.array([H_er(w) for w in omega])
    abs_at_w0 = abs(H_er(OMEGA0))

    # ================= 5) 写 JSON =================
    data = {
        # ---- Simulink 变量 ----
        "A": clean(A), "B": B, "C": np.eye(n), "D": np.zeros((n, 1)),
        "c": C_Y,
        "x0": np.array(X0),
        "A_r": A_R, "B_r": B_R, "C_r": C_R, "D_r": D_R, "w0": np.array(W0),
        "A_obs": F, "B_obs": B_obs, "C_obs": np.ones((1, 1)), "D_obs": 0.0,
        "M": M, "N": N, "z0": Z0,
        "Ac": AC, "Bc": BC, "eta0": np.array(ETA0),
        "KX": K_x, "kyita": K_eta,
        "kyita1": kyita1, "kyita2": kyita2,
        # ---- 复核用 ----
        "design": {
            "task": "降维观测器 + 内模原理的渐近跟踪",
            "Tf": TF, "omega0": OMEGA0,
            "poles_K": cplx_pair(np.linalg.eigvals(A + B @ K_x)),
            "poles_obs": cplx_pair(info_obs["poles"]),
            "poles_aug": cplx_pair(cl["poles"]),
            "poles_A_cl": cplx_pair(poles_big),
            "K_aug": K_aug,
            "A_aug": clean(A_aug), "B_aug": B_aug, "C_aug": C_aug,
            "ctrb_rank_aug": im["ctrb_rank"], "n_aug": im["n_aug"],
            "A_cl": clean(A_big), "z0_big": z0_big,
            "A_ctrl": clean(A_ctrl), "B_ctrl": B_ctrl,
            "C_e": C_e, "D_e": D_e,
            "inter_model_num": num_im, "inter_model_den": den_im,
            "e_over_r_at_omega0": abs_at_w0,
            "X0": np.array(X0),
            "stable": bool(cl["stable"]),
            "K_x": K_x, "K_eta": K_eta,
            "note": "u = KX*x̂ + kyita1*eta1 + kyita2*eta2；"
                    "x̂ = M z + N y；e = y - r 进内模",
        },
    }

    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2, default=_json_default)

    # ================= 可读摘要 =================
    lines = []
    lines.append("实验六设计计算（降维观测器 + 内模原理的渐近跟踪）")
    lines.append("=" * 66)
    lines.append("")
    lines.append("对象: x' = [[0,1],[0,-1]] x + [0;1] u,  y = [1 0] x")
    lines.append(f"参考: r'' = -{OMEGA0:g}^2 r  ->  r(t) = sin({OMEGA0:g} t)（外系统 w' = A_r w）")
    lines.append("内模: eta' = A_c eta + B_c e,  A_c = [[0,-1],[1,0]], B_c = [1;0]")
    lines.append("")
    lines.append("[1] 增广系统极点配置（状态 + 内模）")
    lines.append(f"  期望极点 {POLES_AUG}")
    lines.append(f"  能控 rank = {im['ctrb_rank']} / {im['n_aug']}")
    lines.append(f"  K_aug = {np.array2string(K_aug.ravel(), precision=6)}")
    lines.append(f"     K_x   = {np.array2string(K_x.ravel(), precision=6)}")
    lines.append(f"     K_eta = {np.array2string(K_eta.ravel(), precision=6)}"
                 f"（kyita1 = {kyita1:g} 乘 eta1，kyita2 = {kyita2:g} 乘 eta2）")
    lines.append(f"  增广闭环极点 = "
                 f"{np.array2string(np.sort_complex(cl['poles']), precision=4)}")
    lines.append("")
    lines.append("[2] 降维观测器（与实验五任务二相同）")
    lines.append(f"  误差极点 {POLES_OBS_RED}  ->  F = {float(F.ravel()[0]):g}, "
                 f"G = {float(G.ravel()[0]):g}, H = {float(H.ravel()[0]):g}")
    lines.append(f"  M = {np.array2string(M.ravel(), precision=0)}, "
                 f"N = {np.array2string(N.ravel(), precision=0)}")
    lines.append("")
    lines.append("[3] 闭环（含参考外系统）")
    lines.append(f"  eig(A_cl) = {np.array2string(np.sort_complex(poles_big), precision=4)}")
    lines.append("  = 状态反馈 {-2±j} ∪ 内模 {-1±j} ∪ 观测器 {-5} ∪ 参考外系统 {±j}")
    lines.append(f"  从 r 到 e 的闭环传递函数：|e/r(j omega0)| = {abs_at_w0:.2e}"
                 f"（内模原理要求 0），全频段 min|e/r| = {np.min(np.abs(H_scan)):.2e}")
    lines.append("")
    lines.append("[4] 内模控制器")
    lines.append(f"  C_im(s) = K_eta (sI - A_c)^(-1) B_c = ({kyita1:g} s + {kyita2:g})"
                 f" / (s^2 + {OMEGA0 ** 2:g})")
    lines.append(f"  omega = {OMEGA0:g} 处增益无穷大（分子在 s = ±{OMEGA0:g}j 处不为零）")
    lines.append("")
    lines.append(f"输出: {OUT_JSON}")

    text = "\n".join(lines)
    with open(OUT_LOG, "w", encoding="utf-8") as f:
        f.write(text + "\n")
    print(text)


if __name__ == "__main__":
    main()
