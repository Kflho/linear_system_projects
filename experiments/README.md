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
      lab12_ppt.pptx  lab12_ppt.pdf   lab12_ppt.md   实验一、二合册页片 + 说明
    references/     解读笔记：推导细节、踩过的坑、与课件的对应关系
      ex_01.md  ex_02.md
  src/
    common/         公用脚手架
      linsys_lab.py        C-W 模型 / 能控性矩阵 / 零空间 / 块对角增益
      initial_state_response.m   解析初始状态响应
      export_figure.m     统一出图
      project_paths.m     MATLAB 路径
    ex_01/  ex_02/ ...  每个实验：设计脚本(py) + 主脚本(m) + Simulink 模型(slx)
    lab12_deck/      实验一、二合册页片生成器（pptxgenjs）+ 公式 PNG
```

三类文档的分工：

| 目录 | 是什么 | 给谁看 |
|---|---|---|
| `bin/data/` | 机器的产出（png/mat/log） | 报告引用、复现 |
| `bin/references/` | 我的解读：为什么这么设计、哪里容易错 | 自己复盘、答辩准备 |
| `bin/exports/` | 最终稿：结论表 + PPT 逐页口播 + 可能被问的问题 | 直接交/上台讲 |

## 数学在 Python，仿真在 MATLAB

课程算法（能控性指数、能控规范型、极点配置、解耦、Riccati、观测器）只写一遍，放在 Python
仓库的 `linear_system_utils` 包里；实验目录只做"实验专用"的活：

```
ex_XX_design.py  调 linear_system_utils 算设计量 → ex_XX_design.json（+ 可读的 .txt 摘要）
ex_XX_main.m     读 JSON，解析解 / Simulink 仿真，出图，写日志 → bin/data/ex_XX/
ex_XX_run_sim.m  （可选）给手搭的 .slx 灌参数并仿真（只在内存里改，不保存磁盘文件）
ex_XX_sim.slx    （可选）手搭的 Simulink 模型
```

实验一~四有手搭的 `.slx`（`main.m` 里解析解与 Simulink 对照）；实验五、六任务书只把
Simulink 图当"仿真示例"，所以**不搭模型**，`main.m` 直接用闭环矩阵的 `expm` 解析解
（`z(t) = e^{A_cl t} z0`）出全部曲线，跑起来更省事、也没有建模误差。JSON 里的键名仍然
按 Simulink 变量名起（`A/B/c/A_obs/B_obs/F/G/H/M/N/...`），以后想补模型可以直接用。

库在 `D:\data\local\software\programming\Python\projects\linear_system_utils`，
已 editable 装进同目录的 `.venv`，所以设计脚本用那个 venv 的 python 跑就行；
没装包时脚本会兜底把仓库根挂进 `sys.path`（环境变量 `LINEAR_SYSTEM_UTILS_ROOT` 可覆盖）。

## 运行

```powershell
$py = 'D:\data\local\software\programming\Python\projects\.venv\Scripts\python.exe'

# 实验一~四（设计 + Simulink 对照）
& $py experiments\src\ex_01\ex_01_design.py
matlab -batch "cd('experiments'); run('src/ex_01/ex_01_main.m')"

# 实验五、六（设计 + 纯数值仿真，不需要 Simulink）
& $py experiments\src\ex_05\ex_05_design.py
& $py experiments\src\ex_06\ex_06_design.py
matlab -batch "cd('experiments'); run('src/ex_05/ex_05_main.m')"
matlab -batch "cd('experiments'); run('src/ex_06/ex_06_main.m')"
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
| 实验四 | 输出调节 · 内模原理（倒立摆 / 跟踪+抗扰） | `src/ex_04/` | `bin/data/ex_04/` | `bin/exports/ex_04.md` |
| 实验五 | 观测器设计（磁盘驱动器：全维/降维/函数） | `src/ex_05/` | `bin/data/ex_05/` | `bin/exports/ex_05.md` |
| 实验六 | 观测器与状态反馈（降维观测器 + 内模跟踪） | `src/ex_06/` | `bin/data/ex_06/` | `bin/exports/ex_06.md` |

> 任务书 `lab12` 最后一页的"机床滑动台"是**报告示例**（自选工程背景），不是必做内容。
>
> 任务书 `lab56` 一页里给了实验五、实验六两题的内容和 Simulink 示意图。这里按
> **(一)(二)(三) 观测器设计 → 实验五**、**(四) 降维观测器 + 内模跟踪 → 实验六** 拆分；
> 那几张图只是"仿真示例"，实验五/六 **不搭 Simulink 模型**，纯数值仿真（`expm` 解析解）
> 就能跑出全部曲线，细节见 `bin/references/ex_05.md`、`ex_06.md`。
