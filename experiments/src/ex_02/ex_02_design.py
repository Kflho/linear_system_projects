"""实验二设计计算：卫星运动的输入输出解耦控制。

要求（omega = 1 rad/s）：

    u = K x + H v
    闭环传递函数矩阵  G(K,H)(s) = 1/(s^2+4s+5)  ⊕  1/(s^2+4s+4)

    (2) v(t) = [1(t), 1(t)]'，求阶跃响应 y(t), x(t), u(t)
    (3) 要求两条阶跃响应的稳态值都为 1，重新设计 H

解耦的思路（与课件第 6 章一致）：
    两个输出 y1 = x1、y2 = x3 的相对阶都是 2，解耦矩阵
        B* = C A B = I_2 （非奇异）
    且 d1 + d2 = 2 + 2 = 4 = n，没有剩余动态。于是只要让 A + B K 按两个通道
    块对角，闭环就从 u 到 y 彻底解耦；再取 H 把直流增益调到 1 即可。
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "common"))

import linsys_lab as lab  # noqa: E402

lab.ensure_lib_importable()

OUT_JSON = os.path.join(HERE, "ex_02_design.json")
OUT_LOG = os.path.join(HERE, "ex_02_design.txt")

OMEGA = 1.0
TF = 10.0


def _json_default(o):
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.floating):
        return float(o)
    if isinstance(o, np.bool_):
        return bool(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError(f"Object of type {type(o).__name__} is not JSON serializable")


def transfer_matrix(M, B, C, H, s):
    """G(s) = C (sI - M)^-1 B H"""
    n = M.shape[0]
    return C @ np.linalg.inv(s * np.eye(n) - M) @ B @ H


def main():
    A, B, C, D = lab.cw_model(OMEGA)
    n, m = A.shape[0], B.shape[1]

    # ---- 相对阶与解耦矩阵 --------------------------------------------
    CAB = C @ A @ B
    d = [2, 2]                     # y1 = x1 需要求导两次才出现 u
    bstar = CAB

    # ---- 让 A + BK 块对角（两个期望二阶块）---------------------------
    block1 = np.array([[0.0, 1.0], [-5.0, -4.0]])   # s^2 + 4s + 5 -> -2 +- j
    block2 = np.array([[0.0, 1.0], [-4.0, -4.0]])   # s^2 + 4s + 4 -> -2, -2
    K, M = lab.block_diag_gain(A, B, [block1, block2])

    # ---- 两通道的标量传递函数 g_i(s) = C_i (sI - M_i)^-1 b_i ----------
    def gi(s, blk, i):
        b = np.array([[0.0], [1.0]]) if i == 0 else np.array([[0.0], [1.0]])
        c = np.array([[1.0, 0.0]])
        return c @ np.linalg.inv(s * np.eye(2) - blk) @ b

    # ---- (1) H = I 时应当是 1/(s^2+4s+5) ⊕ 1/(s^2+4s+4) --------------
    H1 = np.eye(m)
    s_samples = [0.0, 0.5, 1.0, 2.0, 1 + 2j, 3j]
    err = 0.0
    for s in s_samples:
        G = transfer_matrix(M, B, C, H1, s)
        Gdes = np.diag([1.0 / (s ** 2 + 4 * s + 5), 1.0 / (s ** 2 + 4 * s + 4)])
        err = max(err, float(np.max(np.abs(G - Gdes))))

    G0 = transfer_matrix(M, B, C, H1, 0.0)
    dc1 = np.real(np.diag(G0))

    # ---- (3) 稳态值都为 1：H = diag(1/g1(0), 1/g2(0)) -----------------
    H3 = np.diag(1.0 / dc1)
    G0b = transfer_matrix(M, B, C, H3, 0.0)
    dc3 = np.real(np.diag(G0b))

    data = {
        "omega": OMEGA,
        "A": A.tolist(), "B": B.tolist(), "C": C.tolist(), "D": D.tolist(),
        "relative_degree": d,
        "decoupling_matrix_CAB": CAB.tolist(),
        "K": K.tolist(),
        "H1": H1.tolist(),
        "H3": H3.tolist(),
        "closed_loop_A_BK": M.tolist(),
        "poles": sorted([complex(p) for p in np.linalg.eigvals(M)], key=lambda z: (z.real, z.imag)),
        "transfer_check": {"s_samples": [complex(s) for s in s_samples], "max_error": err},
        "dc_gain_H1": dc1.tolist(),
        "dc_gain_H3": dc3.tolist(),
        "x0": [0.0, 0.0, 0.0, 0.0],
        "Tf": TF,
    }
    # complex 不能直接进 JSON，转成 real/imag 两列
    data["poles"] = [[float(p.real), float(p.imag)] for p in data["poles"]]
    data["transfer_check"]["s_samples"] = [[float(s.real), float(s.imag)]
                                           for s in data["transfer_check"]["s_samples"]]

    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2, default=_json_default)

    lines = [
        "实验二设计计算（卫星输入输出解耦控制）",
        "=" * 56,
        f"omega = {OMEGA}",
        "",
        "[1] 相对阶与解耦矩阵",
        f"    d = {d},  sum(d) = {sum(d)} = n = {n}（无剩余动态）",
        "    B* = C A B =",
    ]
    for row in CAB:
        lines.append("        " + "  ".join(f"{v:7.3f}" for v in row))
    lines += [
        "",
        "[2] 解耦增益（使 A+BK 块对角）",
        "    K =",
    ]
    for row in K:
        lines.append("        " + "  ".join(f"{v:8.3f}" for v in row))
    lines += [
        "    A + B K =",
    ]
    for row in M:
        lines.append("        " + "  ".join(f"{v:8.3f}" for v in row))
    lines += [
        "",
        "[1] 验证 G(s) = 1/(s^2+4s+5) ⊕ 1/(s^2+4s+4)（H = I）",
        f"    采样 s = {[complex(s) for s in s_samples]}",
        f"    最大偏差 = {err:.3e}",
        "",
        "[2] 直流增益（H = I）",
        f"    G(0) = diag{np.round(dc1, 6).tolist()}  -> 稳态 0.2 / 0.25",
        "",
        "[3] 重新设计 H，使稳态值均为 1",
        "    H = diag(5, 4) =",
    ]
    for row in H3:
        lines.append("        " + "  ".join(f"{v:8.3f}" for v in row))
    lines.append(f"    G(0)H = diag{np.round(dc3, 6).tolist()}  -> 稳态 1 / 1")
    lines.append("")
    lines.append(f"输出: {OUT_JSON}")

    text = "\n".join(lines)
    with open(OUT_LOG, "w", encoding="utf-8") as f:
        f.write(text + "\n")
    print(text)


if __name__ == "__main__":
    main()
