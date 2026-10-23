# 线性系统练习

课程例题/习题的 Python 演算，一章一个目录，一个题号一个脚本；带"标准答案"的题再配一份同名
`.md`（推导 + 结果 + 图）。这里只放"跟着课本章节走"的练习，实验（Simulink、报告、PPT）在隔壁
`../experiments/`。

## 目录约定

```
exercises/exports/
  ls_02/                第 2 章（能控性）
    ex_10/              例题 10，分小问 → 目录
      ex_10_1.py
      ex_10_2.py
      ex_10_3.py
  ls_04/                第 4 章（反馈镇定）
    ex_9.py             例题 9，不分小问 → 单文件
  ls_07/                第 7 章（轨迹跟踪）
    ex_1.py  ex_1.md    有答案的题：脚本 + 解答
    fig/                脚本画出来的图
  ls_09/                第 9 章（观测器）
    ex_6.py  ex_6.md
```

- `ls_NN` = 第 NN 章（ls = linear system），和 `../docs/第N章*.pdf` 对应。
- `ex_M` = 该章例题/习题的第 M 题。
- 一个题目分几小问、或想保留"手算 vs 调库"两份演算时，开 `ex_M/` 目录，按 `ex_M_<小问号>.py` 命名。
- `.md` 只写"答案本身"：条件核对、设计结果（矩阵）、验证；脚本负责复算和画图。

## 现在有什么

| 目录 | 章节 | 内容 |
|---|---|---|
| `ls_02/ex_10/` | 第 2 章 能控性 | 能控规范型：`ex_10_1` 手算 $Q,p,T,\tau$；`ex_10_2` 符号推演；`ex_10_3` 调库算同一组量做对照 |
| `ls_03/ex_7.py` | 第 3 章 稳定性 | 解 Lyapunov 方程 $A^TP+PA=-Q$，用 sympy 判 $P$ 正定（scipy 求解） |
| `ls_04/ex_9.py` | 第 4 章 反馈镇定 | 极点配置：调库直接算增益 $K$ |
| `ls_04/ex_10.py` | 第 4 章 反馈镇定 | 能控性指标集 $\nu$ |
| `ls_05/ex_2.py` | 第 5 章 最优控制 | 草稿（只写了 $A,B$，待补） |
| `ls_05/ex_5.py` | 第 5 章 最优控制 | 代数 Riccati 方程求 $P$ |
| `ls_06/ex_7.py` | 第 6 章 解耦控制 | 解耦矩阵 $\tau$、判据、控制律 $u=Kx+Gv$ |
| `ls_06/ex_8.py` | 第 6 章 解耦控制 | 多输入多输出系统的解耦判据 |
| `ls_07/ex_1.py` + `.md` | 第 7 章 轨迹跟踪 | 7.1：$r=2$、$\Gamma=cAB=\beta_2$，控制律 $u=Kx+\Gamma^{-1}v^*$，跟踪 $y^*=\sin\omega t$ |
| `ls_07/ex_2.py` + `.md` | 第 7 章 轨迹跟踪 | 7.2：最小相位 SISO，$K=[2,-1,-3,-1]$，跟踪周期信号 $\sin^3\pi t$ |
| `ls_07/ex_6.py` + `.md` | 第 7 章 轨迹跟踪 | 7.6：$\Gamma=CB$ 奇异 → 输出变换 $\tilde y=Ty$ → $\tilde\Gamma$ 非奇异 → 跟踪 $y^*=[\sin t,\cos2t]^T$ |
| `ls_09/ex_6.py` + `.md` | 第 9 章 观测器 | 9.6：全维（$L$）、降维（$F,G,H,M,N$）、$q=1$ 函数观测器（$F,G,T,H,M,N$） |
| `ls_09/ex_8.py` + `.md` | 第 9 章 观测器 | 9.8：吊车系统降阶观测器，系统分解法与函数观测器法（结果一致） |
| `ls_09/ex_9.py` + `.md` | 第 9 章 观测器 | 9.9：已知 $F$，反求 $G,H,M$（解族 + 最简解） |

## 算法在哪

所有"算一遍就复用"的算法都在 Python 仓库的 `linear_system_utils` 包里，本目录**不重新实现**：

| 功能 | 入口 |
|---|---|
| 能控性矩阵 $Q_c$ | `linear_system_utils.cal_matrix_parameters.cal_controllablility_matrix` |
| 能控性指标集 $\nu$ | `linear_system_utils.cal_matrix_parameters.cal_set_of_controllability_indices` |
| 能控规范型 $Q,p,T,\tau$ | `linear_system_utils.cal_matrix_operation.cal_controllable_canonical_form` |
| 极点配置 $K$ | `linear_system_utils.cal_matrix_operation.cal_pole_placement.cal_state_feedback_pole_placement` |
| 解耦控制 $K,G$ | `linear_system_utils.cal_matrix_operation.cal_decoupling_control.cal_decoupling_control` |
| 代数 Riccati | `linear_system_utils.cal_matrix_operation.cal_optimal_control.cal_algebra_Riccati_function` |
| 相对阶 $r$ | `linear_system_utils.cal_matrix_operation.cal_decoupling_control.cal_r` |
| 耦合矩阵 $\Gamma$、跟踪律 $u=Kx+\Gamma^{-1}v^*$ | `linear_system_utils.cal_matrix_operation.cal_tracking_control.cal_tracking_control` |
| 参考前馈 $v^*$、误差系统 $A_e$ | `...cal_tracking_control.cal_reference_vector` / `cal_error_dynamics` |
| 观测器（数值）：全维 / 降维 / 函数 | `linear_system_utils.cal_matrix_operation.cal_observer.cal_observer` |
| 观测器（符号）：极点配置 / 降维 / 函数 / 已知 $F,T$ 反求 $G,H,M,N$ | `...cal_observer.cal_observer_symbolic` |

包在 `D:\data\local\software\programming\Python\projects\linear_system_utils`，
已 editable 装进同目录的 `.venv`。

## 运行

```powershell
$py = 'D:\data\local\software\programming\Python\projects\.venv\Scripts\python.exe'
& $py 'D:\data\local\study\postgraduate\linear_system\projects\exercises\exports\ls_07\ex_1.py'
```

脚本在哪个目录都能跑（库是装好的，不靠相对路径）。个别练习要用 scipy（如 `ls_03/ex_7.py`、
第 7/9 章的仿真）和 matplotlib（画图），venv 里都已经装了。
