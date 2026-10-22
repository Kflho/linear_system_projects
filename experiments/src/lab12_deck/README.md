# 实验一、二 页片生成器

用 `pptxgenjs` 生成 `实验一 + 实验二` 合册页片（PDF 一起交），输出可编辑 `.pptx`。
每页都写口播备注；标题按**实验目的/要求**命名，结论放在页面底部的「结论｜」行。

## 依赖

- Node + `npm i pptxgenjs`（在本目录执行一次）
- 公式 PNG 由 `kf-proposal-exports/scripts/formula_png.py` 生成（需要 TeX Live + Pillow），
  产物在本目录 `formulas/`（含 `sizes.json`）
- 自检脚本同样来自 `kf-proposal-exports/scripts/`

## 生成

```bash
# 在本目录
node gen_deck.js --out ../../bin/exports/lab12_ppt.pptx

# 图数据默认取 ../../bin/data/ex_01|ex_02，可用环境变量覆盖
DECK_DATA=<path-to-data> node gen_deck.js --out /tmp/deck.pptx
```

## 自检（缺一不可）

```bash
python <kf-proposal-exports>/scripts/deck_check.py ../../bin/exports/lab12_ppt.pptx --strict --require-notes
python <kf-proposal-exports>/scripts/text_check.py ../../bin/exports/lab12_ppt.pptx
python <kf-proposal-exports>/scripts/render_pptx_png.py ../../bin/exports/lab12_ppt.pptx --out <预览目录>
```

2026-10-22 记录：`deck_check` 0 error / 0 warning；`text_check` 0 error、19 warning
（warning 来自「标题按实验目的命名」与图注/数字标签被计入要点，属预期）。

## 素材与出处

| 名称 | 来源 |
| --- | --- |
| `formulas.json` → `formulas/*.png` | LaTeX（`formula_png.py`，400 dpi，透明底） |
| `ex_01_*.png` | `../../bin/data/ex_01/`（MATLAB `export_figure.m`，200 dpi） |
| `ex_02_*.png` | `../../bin/data/ex_02/` |
| 数值 | `bin/data/ex_01|ex_02/*_log.txt`（脚本生成，不手改） |
