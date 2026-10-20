"""实验一设计计算：卫星相对轨道运动的状态反馈极点配置。

数学部分全部复用 Python 仓库的 linear_system_utils 包：
    能控性指数集  linear_system_utils.cal_matrix_parameters.cal_set_of_controllability_indices
    能控规范型    linear_system_utils.cal_matrix_operation.cal_controllable_canonical_form (Q, p, T, Tau)
    极点配置公式  linear_system_utils.cal_matrix_operation.cal_pole_placement.cal_state_feedback_pole_placement

本文件只做"实验专用"的三件事：
    1) 建 C-W 模型（A, B, C, D）；
    2) 能控性秩 + 不可控模态定位（任务 2、3）；
    3) 两组 K：4.4 公式解 + 解耦型块对角解（任务 4）。

结果写到 ex_01_design.json，供 MATLAB 侧读取（跑 Simulink、算解析响应、画图、写报告）。

包已 editable 装进 Python 仓库的 .venv，直接用那个 python 跑即可；
没装包时下面的 ensure_lib_importable() 会兜底注入仓库根，可用环境变量
LINEAR_SYSTEM_UTILS_ROOT 覆盖。
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "common"))

from linsys_lab import ensure_lib_importable  # noqa: E402

ensure_lib_importable()

from linear_system_utils.cal_matrix_operation.cal_pole_placement.cal_state_feedback_pole_placement import (  # noqa: E402
    cal_state_feedback_pole_placement,
)

OUT_JSON = os.path.join(HERE, "ex_01_design.json")
OUT_LOG = os.path.join(HERE, "ex_01_design.txt")


def _json_default(o):
    """numpy 标量/数组转成 JSON 能接受的原生类型"""
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.floating):
        return float(o)
    if isinstance(o, np.bool_):
        return bool(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError(f"Object of type {type(o).__name__} is not JSON serializable")

OMEGA_DESIGN = 1.0        # 设计用轨道角速度 (rad/s)，题目要求
OMEGA_REAL = 0.0011       # 实际 250 海里轨道角速度
POLES = [-2 + 1j, -2 - 1j, -2, -2]
X0 = [1.0, 0.0, 0.0, 0.0]  # 初始状态：径向 1 m 偏差


# ----------------------------------------------------------------------
# 模型与能控性
# ----------------------------------------------------------------------
def model(omega):
    """C-W 方程状态空间模型，x = [x, xdot, y, ydot]', u = [u_r, u_t]'"""
    w = omega
    A = np.array([[0, 1, 0, 0],
                  [3 * w ** 2, 0, 0, 2 * w],
                  [0, 0, 0, 1],
                  [0, -2 * w, 0, 0]], dtype=float)
    B = np.array([[0, 0], [1, 0], [0, 0], [0, 1]], dtype=float)
    C = np.array([[1, 0, 0, 0], [0, 0, 1, 0]], dtype=float)
    D = np.zeros((2, 2))
    return A, B, C, D


def ctrb(A, B):
    """能控性矩阵 [B, AB, ..., A^(n-1)B]"""
    n = A.shape[0]
    cols = [B]
    for _ in range(n - 1):
        cols.append(A @ cols[-1])
    return np.hstack(cols)


def null_space(M, tol=1e-9):
    """M 的零空间（列向量），用 SVD 实现，不依赖 scipy"""
    U, S, Vt = np.linalg.svd(M)
    s_max = S[0] if S.size else 0.0
    r = int(np.sum(S > tol * max(M.shape) * max(s_max, 1.0)))
    return Vt[r:].T.conj()


def uncontrollable_mode(A, b):
    """单输入 (A,b) 的不可控模态：左零向量 w 满足 w'A = lambda w'、w'b = 0"""
    Wc = ctrb(A, b.reshape(-1, 1))
    r = np.linalg.matrix_rank(Wc)
    out = {"rank": r, "controllable": bool(r == A.shape[0])}
    if out["controllable"]:
        return out

    w = null_space(Wc.T)[:, 0]
    w = w / np.linalg.norm(w)
    # 零向量符号本来就任意，不同 numpy/LAPACK 版本会翻（同一份代码换环境就 +-变）。
    # 统一成"模最大的分量取正"，输出才可复现，也和报告里的 w ∝ [2,0,0,1] 对得上。
    k = int(np.argmax(np.abs(w)))
    if np.real(w[k]) < 0:
        w = -w
    lam = np.vdot(w, A @ w) / np.vdot(w, w)      # w' A w / w' w
    if abs(lam) < 1e-9:
        lam = 0.0 + 0.0j
    eigvals, eigvecs = np.linalg.eig(A)
    idx = int(np.argmin(np.abs(eigvals - lam)))
    v = eigvecs[:, idx]

    out.update({
        "lambda": [float(np.real(lam)), float(np.imag(lam))],
        "left_vector": np.real_if_close(w).real.tolist(),
        "right_vector": np.real_if_close(v / np.max(np.abs(v))).real.tolist(),
        "residual_left": float(np.linalg.norm(A.T @ w - lam * w)),
        "wH_b": float(abs(np.vdot(w, b.reshape(-1, 1)))),
        "pbm_rank": int(np.linalg.matrix_rank(np.hstack([A - lam * np.eye(A.shape[0]),
                                                         b.reshape(-1, 1)]))),
    })
    return out


# ----------------------------------------------------------------------
# 两组增益
# ----------------------------------------------------------------------
def gain_formula(A, B):
    """第一组：课件 4.4 极点配置公式（复用仓库里的实现）"""
    K, info = cal_state_feedback_pole_placement(A, B, POLES)
    return K, {"nu": list(info["nu"]),
               "T": np.asarray(info["T"]).tolist(),
               "Tau": np.asarray(info["Tau"]).tolist(),
               "gamma_star": np.asarray(info["gamma_star"]).real.tolist()}


def gain_decoupling(A):
    """第二组：解耦型块对角解。

    让闭环矩阵直接分成两个二阶块：

        A + B K = blkdiag([0 1; -5 -4], [0 1; -4 -4])

    对应特征多项式 s^2+4s+5 = (s+2)^2+1 -> -2±j，s^2+4s+4 = (s+2)^2 -> -2,-2。
    B 的第 2、4 行是单位行，所以 K 直接从 (blkdiag - A) 的第 2、4 行读出。
    """
    n = A.shape[0]
    blk = np.zeros((n, n))
    blk[0:2, 0:2] = np.array([[0, 1], [-5, -4]])
    blk[2:4, 2:4] = np.array([[0, 1], [-4, -4]])
    return (blk - A)[[1, 3], :], blk


# ----------------------------------------------------------------------
# 主流程
# ----------------------------------------------------------------------
def main():
    A, B, C, D = model(OMEGA_DESIGN)
    Ar, Br, _, _ = model(OMEGA_REAL)

    ranks = {
        "both": int(np.linalg.matrix_rank(ctrb(A, B))),
        "radial": int(np.linalg.matrix_rank(ctrb(A, B[:, [0]]))),
        "tangential": int(np.linalg.matrix_rank(ctrb(A, B[:, [1]]))),
        "both_real": int(np.linalg.matrix_rank(ctrb(Ar, Br))),
        "radial_real": int(np.linalg.matrix_rank(ctrb(Ar, Br[:, [0]]))),
        "tangential_real": int(np.linalg.matrix_rank(ctrb(Ar, Br[:, [1]]))),
    }
    radial_mode = uncontrollable_mode(A, B[:, 0])
    tangential_mode = uncontrollable_mode(A, B[:, 1])

    K1, info1 = gain_formula(A, B)
    K2, blk = gain_decoupling(A)

    def charpoly(K):
        coeffs = np.poly(np.linalg.eigvals(A + B @ K))
        return np.real_if_close(coeffs).real.tolist()

    want = np.poly(POLES)
    data = {
        "omega_design": OMEGA_DESIGN,
        "omega_real": OMEGA_REAL,
        "A": A.tolist(), "B": B.tolist(), "C": C.tolist(), "D": D.tolist(),
        "A_real": Ar.tolist(), "B_real": Br.tolist(),
        "ranks": ranks,
        "radial_mode": radial_mode,
        "tangential_mode": tangential_mode,
        "poles": {"real": [p.real for p in POLES], "imag": [p.imag for p in POLES]},
        "K1": K1.real.tolist(),
        "K2": K2.real.tolist(),
        "K1_info": info1,
        "closed_loop_matrix_K2": blk.tolist(),
        "charpoly_desired": want.tolist(),
        "charpoly_K1": charpoly(K1),
        "charpoly_K2": charpoly(K2),
        "residual_K1": float(np.max(np.abs(np.array(charpoly(K1)) - want))),
        "residual_K2": float(np.max(np.abs(np.array(charpoly(K2)) - want))),
        "x0": X0,
    }

    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2, default=_json_default)

    lines = []
    lines.append("实验一设计计算（卫星状态反馈极点配置）")
    lines.append("=" * 56)
    lines.append(f"omega_design = {OMEGA_DESIGN}, omega_real = {OMEGA_REAL}")
    lines.append("")
    lines.append("[任务1] 状态空间模型")
    lines.append("A =")
    for row in A:
        lines.append("    " + "  ".join(f"{v:9.4g}" for v in row))
    lines.append("B =")
    for row in B:
        lines.append("    " + "  ".join(f"{v:9.4g}" for v in row))
    lines.append("")
    lines.append("[任务2/3] 能控性 rank(ctrb(A,B))")
    lines.append(f"  两路推力        : {ranks['both']}   (omega = {OMEGA_DESIGN})")
    lines.append(f"  仅径向 u_r      : {ranks['radial']}  -> "
                 f"{'能控' if ranks['radial'] == 4 else '不能控'}")
    lines.append(f"  仅切向 u_t      : {ranks['tangential']}  -> "
                 f"{'能控' if ranks['tangential'] == 4 else '不能控'}")
    lines.append(f"  (omega = {OMEGA_REAL} 时: {ranks['both_real']} / "
                 f"{ranks['radial_real']} / {ranks['tangential_real']}，秩与 omega 无关)")
    if not radial_mode["controllable"]:
        lam = radial_mode["lambda"][0]
        lines.append(f"  仅径向时不可控模态: lambda = {lam:g}, "
                     f"左零向量 w' = {np.array2string(np.array(radial_mode['left_vector']), precision=4)}")
        lines.append(f"      对应右特征向量 v = {np.array2string(np.array(radial_mode['right_vector']), precision=4)}")
        lines.append(f"      PBH: rank([A-lambda I, b]) = {radial_mode['pbm_rank']} < 4")
    lines.append("")
    lines.append("[任务4] 两组反馈增益")
    lines.append(f"  nu = {info1['nu']}  （能控性指数集）")
    lines.append("  K1（课件 4.4 公式） =")
    for row in K1:
        lines.append("      " + "  ".join(f"{v:9.4f}" for v in row))
    lines.append("  K2（解耦型块对角） =")
    for row in K2:
        lines.append("      " + "  ".join(f"{v:9.4f}" for v in row))
    lines.append(f"  期望特征多项式 : {np.array2string(want, precision=4)}")
    lines.append(f"  K1 闭环特征多项式: {np.array2string(np.array(charpoly(K1)), precision=4)}"
                 f"   偏差 {data['residual_K1']:.3g}")
    lines.append(f"  K2 闭环特征多项式: {np.array2string(np.array(charpoly(K2)), precision=4)}"
                 f"   偏差 {data['residual_K2']:.3g}")
    lines.append("")
    lines.append(f"输出: {OUT_JSON}")

    text = "\n".join(lines)
    with open(OUT_LOG, "w", encoding="utf-8") as f:
        f.write(text + "\n")
    print(text)


if __name__ == "__main__":
    main()
