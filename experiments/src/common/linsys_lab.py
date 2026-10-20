"""实验公用脚手架（不含课程算法的实现）。

真正的线性系统算法（能控性指数、能控规范型、极点配置、解耦、Riccati）都在
Python 仓库的 linear_system_utils 包里（editable 安装，正常 import 即可），
这里只放各实验都要用的几件小事：

    ensure_lib_importable() 没装包时把 Python 仓库根挂进 sys.path（兜底）
    cw_model(omega)         卫星 C-W 状态空间模型
    ctrb(A, B)              能控性矩阵
    null_space(M)           零空间（SVD，不依赖 scipy）
    block_diag_gain(A,B,·)  "让 A+BK 变成块对角" 的 K（解耦控制的标准做法）
"""
import os
import sys

import numpy as np

# linear_system_utils 包的所在目录（Python 仓库根）
DEFAULT_LIB_ROOT = r"D:\data\local\software\programming\Python\projects"


def ensure_lib_importable(root=None):
    """确保 linear_system_utils 可导入。

    正常情况（.venv 里 pip install -e 过）什么都不做；只在没装包时把仓库根
    临时挂到 sys.path 上兜底，避免为了跑一个脚本先折腾环境。返回注入的路径
    （没注入则为 None）。
    """
    try:
        import linear_system_utils  # noqa: F401
        return None
    except ImportError:
        pass
    root = root or os.environ.get("LINEAR_SYSTEM_UTILS_ROOT", DEFAULT_LIB_ROOT)
    if root not in sys.path:
        sys.path.insert(0, root)
    return root


def cw_model(omega):
    """卫星相对轨道运动（C-W / Hill 方程）

        xddot - 2*w*ydot - 3*w^2*x = u_r
        yddot + 2*w*xdot           = u_t

    状态 x = [x, xdot, y, ydot]'，输入 u = [u_r, u_t]'，输出 y = [x, y]'
    返回 (A, B, C, D)
    """
    w = float(omega)
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
    cols = [np.asarray(B, dtype=float)]
    for _ in range(n - 1):
        cols.append(A @ cols[-1])
    return np.hstack(cols)


def null_space(M, tol=1e-9):
    """M 的零空间（列向量），用 SVD 实现"""
    _, S, Vt = np.linalg.svd(np.asarray(M, dtype=float))
    s_max = S[0] if S.size else 0.0
    r = int(np.sum(S > tol * max(M.shape) * max(s_max, 1.0)))
    return Vt[r:].T.conj()


def block_diag_gain(A, B, blocks):
    """选 K 使 A + B K = blkdiag(blocks...)

    仅适用于"块的最后一行对应 B 的某个单位行"的情形（卫星、倒立摆都是）：
    若 B 的第 i 行只有一个非零元 +-1 在第 j 列，则 K 的第 j 行 = (目标 - A) 的第 i 行。
    解耦控制就是靠这种结构把各通道拆开。
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    n = A.shape[0]
    target = np.zeros((n, n))
    s = 0
    for blk in blocks:
        m = blk.shape[0]
        target[s:s + m, s:s + m] = blk
        s += m
    if s != n:
        raise ValueError(f"块维数之和 {s} 不等于 n = {n}")

    delta = target - A
    K = np.zeros((B.shape[1], n))
    for i in range(n):
        row = B[i, :]
        nz = np.flatnonzero(np.abs(row) > 1e-12)
        if nz.size == 1 and abs(abs(row[nz[0]]) - 1) < 1e-12:
            K[nz[0], :] = delta[i, :] / row[nz[0]]
    return K, target
