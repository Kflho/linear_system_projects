# 线性系统 · 工程目录

课程材料（课件 PDF）在 `../docs/`，本目录放**动手做的东西**，按"练习"和"实验"两分：

| 目录 | 是什么 | 产出 |
|---|---|---|
| `exercises/` | 课程例题/习题的 Python 演算，按章节走（`ls_NN/ex_M.py`） | 控制台结果、对照结论 |
| `experiments/` | 课程实验：Simulink 仿真 + 报告 + PPT 页稿（`ex_01`…`ex_06`） | 图、mat、日志、提交稿 |

## 算力和算法分开放

- **算法只写一遍**：能控性指标、能控规范型、极点配置、解耦、Riccati 全在 Python 仓库的
  `linear_system_utils` 包里 → `D:\data\local\software\programming\Python\projects\linear_system_utils`
- **Python 环境**：`D:\data\local\software\programming\Python\projects\.venv`（包已 editable 安装，
  numpy/sympy/scipy 齐全）。跑任何练习/设计脚本都用这个 venv 的 python。
- **MATLAB 只管仿真**：`experiments/src/*.m` 读 Python 侧算好的 JSON，不在 MATLAB 里重算设计量。

Python 仓库那边只剩"练习 + 库"两件东西：

```
Python\projects\
  .venv\                Python 环境（editable 装好 linear_system_utils）
  linear_system_utils\  算法包（被这里和 experiments 复用）
  python\               Python 语言练习（py_01、py_02）
```

## 入口

- 练习说明：`exercises/README.md`
- 实验说明：`experiments/README.md`
- 课件：`../docs/`
