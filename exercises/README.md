# 线性系统练习

课程例题/习题的 Python 演算，一章一个目录，一个例题一个脚本。这里只放"跟着课本章节走"的练习，
实验（Simulink、报告、PPT）在隔壁 `../experiments/`。

## 目录约定

```
exercises/
  ls_02/              第 2 章（能控性）
    ex_10/            例题 10，分小问 → 目录
      ex_10_1.py
      ex_10_2.py
      ex_10_3.py
  ls_04/              第 4 章（反馈镇定）
    ex_9.py           例题 9，不分小问 → 单文件
  ls_03/  ls_05/  ls_06/
```

- `ls_NN` = 第 NN 章（ls = linear system），和 `../docs/第N章*.pdf` 对应。
- `ex_M` = 该章例题/习题的第 M 题。
- 一个题目分几小问、或想保留"手算 vs 调库"两份演算时，开 `ex_M/` 目录，按 `ex_M_<小问号>.py` 命名。

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

包在 `D:\data\local\software\programming\Python\projects\linear_system_utils`，
已 editable 装进同目录的 `.venv`。

## 运行

```powershell
$py = 'D:\data\local\software\programming\Python\projects\.venv\Scripts\python.exe'
& $py 'D:\data\local\study\postgraduate\linear_system\projects\exercises\ls_04\ex_9.py'
```

脚本在哪个目录都能跑（库是装好的，不靠相对路径）。个别练习要用 scipy（如 `ls_03/ex_7.py`），
venv 里已经装了。
