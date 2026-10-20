# 线性系统实验工作区

课件在 `../docs/`，任务书在 `bin/requirements/`，实验代码与产出都在本目录。
课程练习（例题/习题）不在本目录，在隔壁 `../exercises/`。

## 目录约定

```
experiments/
  bin/
    requirements/   任务书 PDF（原始材料，只读）
    data/           实验产出：图、mat 数据、运行日志 —— 脚本自动生成，不要手改
      ex_01/  ex_02/ ...
    exports/        最终提交稿：报告 + 2 分钟 PPT 页稿（对齐任务书要求）
      ex_01.md  ex_02.md
    references/     解读笔记：推导细节、踩过的坑、与课件的对应关系
      ex_01.md  ex_02.md
  src/
    common/         公用脚手架
      linsys_lab.py        C-W 模型 / 能控性矩阵 / 零空间 / 块对角增益
      initial_state_response.m   解析初始状态响应
      export_figure.m     统一出图
      project_paths.m     MATLAB 路径
    ex_01/  ex_02/ ...  每个实验：设计脚本(py) + 主脚本(m) + Simulink 模型(slx)
```

三类文档的分工：

| 目录 | 是什么 | 给谁看 |
|---|---|---|
| `bin/data/` | 机器的产出（png/mat/log） | 报告引用、复现 |
| `bin/references/` | 我的解读：为什么这么设计、哪里容易错 | 自己复盘、答辩准备 |
| `bin/exports/` | 最终稿：结论表 + PPT 逐页口播 + 可能被问的问题 | 直接交/上台讲 |

## 数学在 Python，仿真在 MATLAB

课程算法（能控性指数、能控规范型、极点配置、解耦、Riccati）只写一遍，放在 Python 仓库的
`linear_system_utils` 包里；实验目录只做"实验专用"的活：

```
ex_XX_design.py  调 linear_system_utils 算设计量 → ex_XX_design.json（+ 可读的 .txt 摘要）
ex_XX_main.m     读 JSON，跑 Simulink，解析解对照，出图，写日志 → bin/data/ex_XX/
ex_XX_run_sim.m  给手搭的 .slx 灌参数并仿真（只在内存里改，不保存磁盘文件）
ex_XX_sim.slx    手搭的 Simulink 模型（保持不动）
```

库在 `D:\data\local\software\programming\Python\projects\linear_system_utils`，
已 editable 装进同目录的 `.venv`，所以设计脚本用那个 venv 的 python 跑就行；
没装包时脚本会兜底把仓库根挂进 `sys.path`（环境变量 `LINEAR_SYSTEM_UTILS_ROOT` 可覆盖）。

## 运行

```powershell
$py = 'D:\data\local\software\programming\Python\projects\.venv\Scripts\python.exe'
& $py experiments\src\ex_01\ex_01_design.py
matlab -batch "cd('experiments'); run('src/ex_01/ex_01_main.m')"
```

## 符号约定

- 状态反馈取 **u = Kx**，闭环 **A + BK**（与课件第 4 章一致；用 u = −Kx 时取 −K）。
- MATLAB `place` / `acker` 的约定是 **A − BK**，与本工程相反。
- 卫星：x = [x, ẋ, y, ẏ]ᵀ，u = [u_r, u_t]ᵀ，y = [x, y]ᵀ。

## 进度

| 编号 | 主题 | 代码 | 产出 | 提交稿 |
|---|---|---|---|---|
| 实验一 | 状态反馈极点配置（卫星） | `src/ex_01/` | `bin/data/ex_01/` | `bin/exports/ex_01.md` |
| 实验二 | 输入输出解耦控制（卫星） | `src/ex_02/` | `bin/data/ex_02/` | `bin/exports/ex_02.md` |
| 实验三 | 输出调节 · 静态状态反馈（倒立摆） | `src/ex_03/` | `bin/data/ex_03/` | `bin/exports/ex_03.md` |
| 实验四 | 输出调节 · 内模原理（倒立摆） | 待做 | | |
| 实验五 | 观测器设计（磁盘驱动器） | 待做 | | |
| 实验六 | 观测器 + 状态反馈 | 待做 | | |

> 任务书 `lab12` 最后一页的"机床滑动台"是**报告示例**（自选工程背景），不是必做内容。
