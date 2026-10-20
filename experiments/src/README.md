# 源码目录

一个实验一个目录；数学计算复用 Python 仓库的 `linear_system_utils` 包，这里不重复实现。

目录结构、`bin/` 下 data/exports/references 的分工、运行方式，见 [`../README.md`](../README.md)。

## 每个实验的固定五件套

| 文件 | 作用 |
|---|---|
| `ex_XX_design.py` | 建模型、算设计量（$K$、$H$ 等），输出 `ex_XX_design.json` + `.txt` 摘要 |
| `ex_XX_design.json` | 设计结果，供 MATLAB 读取（唯一数据源，避免 MATLAB 里再写一遍模型） |
| `ex_XX_main.m` | 复核 + 解析响应 + 跑 Simulink + 出图 + 写日志 → `bin/data/ex_XX/` |
| `ex_XX_run_sim.m` | 给手搭的 `.slx` 灌参数并仿真（只在内存里改，不保存磁盘文件） |
| `ex_XX_sim.slx` | 手搭的 Simulink 模型 |

## common/

| 文件 | 作用 |
|---|---|
| `linsys_lab.py` | C-W 模型、能控性矩阵、零空间、块对角增益、仓库路径兜底 |
| `initial_state_response.m` | 解析初始状态响应 $x(t)=e^{A_ct}x_0$，$u=Kx$ |
| `export_figure.m` | 统一出图（PNG 200 dpi + FIG） |
| `project_paths.m` | 把 `src/` 加入 MATLAB 路径 |

## 复用到的 Python 库（linear_system_utils）

| 功能 | 函数 |
|---|---|
| 能控性指数集（按幂次逐层扫） | `cal_set_of_controllability_indices` |
| 能控规范型 $Q,\;p_i,\;T,\;\tau$ | `cal_Q` / `cal_p` / `cal_T` / `cal_Tau` |
| 4.4 极点配置公式（已改为返回 K） | `cal_state_feedback_pole_placement(A,B,poles)` → `K, info` |
| 解耦控制（实验二） | `cal_decoupling_control` / `cal_r` / `cal_Tau` |
| Riccati | `cal_algebra_Riccati_function` |

## 运行（以实验一为例）

```powershell
python experiments\src\ex_01\ex_01_design.py
matlab -batch "cd('experiments'); run('src/ex_01/ex_01_main.m')"
```
