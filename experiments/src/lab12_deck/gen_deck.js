// gen_deck.js — 实验一 + 实验二 页片（线性系统 lab12）
// 标题按实验目的/要求命名，结论写在每页正文的「结论｜」行。
// 令牌见 kf-proposal-exports/references/ppt-deck.md：13.333x7.5in，边距 0.5in。

const pptxgen = require("pptxgenjs");
const fs = require("fs");
const path = require("path");

const HERE = __dirname;
const sizes = JSON.parse(fs.readFileSync(path.join(HERE, "formulas", "sizes.json"), "utf8"));

const DATA = process.env.DECK_DATA || path.resolve(__dirname, "../../bin/data");
const FIG01 = path.join(DATA, "ex_01"), FIG02 = path.join(DATA, "ex_02");

const NAVY = "1E2761", ICE = "CADCFC", AMBER = "E8A33D", AMBER_L = "FDF3E3";
const INK = "1F2937", MUTED = "6B7A90", TINT = "F4F7FC", LINE = "DCE6F5";
const CN = "Microsoft YaHei";
const W = 13.333, H = 7.5, MARGIN = 0.5;
const BODY_TOP = 1.88, BOTTOM = 6.9;

const pres = new pptxgen();
pres.layout = "LAYOUT_WIDE";
pres.author = "kf-proposal-exports";
pres.title = "线性系统实验一、二：状态反馈极点配置与输入输出解耦";

/* ---------- 工具 ---------- */

// 富文本：支持 _r / _1 下标、^2 / ^(-11) 上标（t 可以写成 ẋ 等直接字符）
function rich(text) {
  const out = [];
  const re = /_\{([^}]*)\}|\^\{([^}]*)\}|_([A-Za-z0-9])|\^([-−]?\d+|[A-Za-z0-9])/g;
  let last = 0, m;
  while ((m = re.exec(text)) !== null) {
    if (m.index > last) out.push({ text: text.slice(last, m.index), options: {} });
    const sub = m[1] !== undefined ? m[1] : m[3];
    const sup = m[2] !== undefined ? m[2] : m[4];
    out.push({ text: sub !== undefined ? sub : sup,
               options: sub !== undefined ? { subscript: true } : { superscript: true } });
    last = re.lastIndex;
  }
  if (last < text.length) out.push({ text: text.slice(last), options: {} });
  return out;
}

function eyebrow(s, text) {
  s.addText(text, { x: 0.7, y: 0.53, w: 8, h: 0.28, fontFace: CN, fontSize: 11,
                    color: MUTED, charSpacing: 2, margin: 0 });
}

function slideTitle(s, text) {
  s.addText(text, { x: 0.68, y: 0.86, w: W - 1.35, h: 0.8, fontFace: CN, fontSize: 30,
                    bold: true, color: NAVY, margin: 0, valign: "top" });
}

function pageNum(s, n, total) {
  s.addText(`${n} / ${total}`, { x: W - MARGIN - 1.2, y: H - MARGIN - 0.32, w: 1.2, h: 0.3,
                                 fontFace: CN, fontSize: 10, color: MUTED, align: "right", margin: 0 });
}

// 结论行：琥珀色引导词 + 深色粗体
function conclusion(s, text, y = BOTTOM - 0.45) {
  const runs = [{ text: "结论｜", options: { color: AMBER, bold: true } }].concat(rich(text));
  s.addText(runs, { x: 0.7, y, w: W - 1.5, h: 0.42, fontFace: CN, fontSize: 15,
                    bold: true, color: NAVY, margin: 0, valign: "middle" });
}

function bullets(s, items, box) {
  const runs = [];
  items.forEach(([lead, body]) => {
    runs.push({ text: lead, options: { bold: true, color: NAVY, bullet: { code: "25CF" } } });
    rich(body).forEach((r) => runs.push(r));
    const tail = runs[runs.length - 1];
    tail.options = Object.assign({}, tail.options, { breakLine: true, paraSpaceAfter: 13 });
  });
  s.addText(runs, Object.assign({ fontFace: CN, fontSize: 15, color: INK,
                                  lineSpacingMultiple: 1.38, margin: 0, valign: "top" }, box));
}

function card(s, box, color = TINT) {
  s.addShape(pres.ShapeType.roundRect, Object.assign({
    rectRadius: 0.1, fill: { color }, line: { color }, }, box));
}

function eqSize(name, maxW, maxH = 3.4) {
  const [px, py] = sizes[name];
  let w = maxW, h = w * py / px;
  if (h > maxH) { h = maxH; w = h * px / py; }
  return { w, h };
}

// 在 (cx 居中, top) 放公式，返回底边 y
function putEq(s, name, cx, top, maxW, maxH) {
  const b = eqSize(name, maxW, maxH);
  s.addImage({ path: path.join(HERE, "formulas", `${name}.png`), x: cx - b.w / 2, y: top, w: b.w, h: b.h });
  if (top + b.h > BOTTOM + 0.01) console.log(`!! 越界：${name} bottom=${(top + b.h).toFixed(2)}`);
  return top + b.h;
}

function img(s, file, x, y, w, h) {
  s.addImage({ path: file, x, y, w, h });
}

function caption(s, text, box) {
  s.addText(text, Object.assign({ fontFace: CN, fontSize: 10, color: MUTED, margin: 0 }, box));
}

function stat(s, x, y, value, label, w = 2.2) {
  s.addText(value, { x, y, w, h: 0.7, fontFace: CN, fontSize: 26, bold: true, color: NAVY, margin: 0 });
  s.addText(label, { x, y: y + 0.58, w, h: 0.5, fontFace: CN, fontSize: 11, color: MUTED, margin: 0 });
}

/* ---------- 幻灯片 ---------- */

function cover() {
  const s = pres.addSlide();
  s.background = { color: NAVY };
  // 轨道母题：两条同心圆 + 一个卫星点
  s.addShape(pres.ShapeType.ellipse, { x: 8.9, y: 2.2, w: 6.2, h: 6.2, fill: { color: NAVY },
                                       line: { color: ICE, width: 1.0, transparency: 55 } });
  s.addShape(pres.ShapeType.ellipse, { x: 10.35, y: 3.65, w: 3.3, h: 3.3, fill: { color: NAVY },
                                       line: { color: ICE, width: 0.8, transparency: 70 } });
  s.addShape(pres.ShapeType.ellipse, { x: 11.85, y: 3.72, w: 0.16, h: 0.16,
                                       fill: { color: AMBER }, line: { color: AMBER } });
  s.addShape(pres.ShapeType.ellipse, { x: 13.0, y: 5.05, w: 0.5, h: 0.5, fill: { color: NAVY },
                                       line: { color: ICE, width: 0.8, transparency: 40 } });

  s.addText("线性系统实验 · 卫星相对运动", { x: 0.85, y: 1.15, w: 8, h: 0.32, fontFace: CN,
                                             fontSize: 12, color: ICE, charSpacing: 3, margin: 0 });
  s.addText("状态反馈极点配置与输入输出解耦", { x: 0.82, y: 1.72, w: 9.6, h: 0.9, fontFace: CN,
                                               fontSize: 34, bold: true, color: "FFFFFF", margin: 0 });
  s.addText("实验一、实验二 · 建模 → 能控性 → 极点配置 → 解耦", {
    x: 0.85, y: 2.85, w: 9.2, h: 0.45, fontFace: CN, fontSize: 17, color: ICE, margin: 0 });
  s.addText("C-W 方程　·　ω = 1 rad/s　·　MATLAB / Simulink", {
    x: 0.85, y: 3.45, w: 8.6, h: 0.4, fontFace: CN, fontSize: 13, color: "A9BCE0", margin: 0 });
  s.addText("汇报人：________　　学号：________　　日期：________", {
    x: 0.85, y: 6.35, w: 8.6, h: 0.35, fontFace: CN, fontSize: 12, color: ICE, margin: 0 });
  s.addNotes("老师好，我汇报线性系统实验一和实验二，主线是建模、能控性、极点配置、输入输出解耦四步。");
  return s;
}

function p2() {
  const s = pres.addSlide();
  s.background = { color: "FFFFFF" };
  eyebrow(s, "实验一 · 状态反馈极点配置");
  slideTitle(s, "建立卫星相对运动的状态空间模型");
  bullets(s, [
    ["状态", "：径向位置与速率、切向位置与速率，共 4 维。"],
    ["输入", "：两路小推力发动机，径向 u_r、切向 u_t。"],
    ["输出", "：只能测到两个位置 y = [x, y]^T。"],
    ["耦合", "：A 中的 2ωẋ、2ωẏ 让径向与切向互相带动。"],
    ["尺度", "：设计取 ω = 1，真实轨道约 0.0011 rad/s。"],
  ], { x: 0.75, y: BODY_TOP, w: 5.4, h: 4.4 });

  card(s, { x: 6.35, y: 1.78, w: 6.45, h: 4.1 });
  s.addText("C-W 方程（Hill 方程）", { x: 6.55, y: 1.95, w: 5, h: 0.3, fontFace: CN,
                                       fontSize: 11, color: MUTED, margin: 0 });
  putEq(s, "eq_cw", 9.52, 2.32, 5.7);
  const a = eqSize("eq_A", 3.25, 2.1), b = eqSize("eq_B", 1.45, 2.1);
  const rowW = a.w + 0.3 + b.w, rowX = 9.52 - rowW / 2;
  s.addImage({ path: path.join(HERE, "formulas", "eq_A.png"), x: rowX, y: 2.95, w: a.w, h: a.h });
  s.addImage({ path: path.join(HERE, "formulas", "eq_B.png"), x: rowX + a.w + 0.3, y: 2.95, w: b.w, h: b.h });
  const c = eqSize("eq_C", 2.85, 1.1);
  s.addImage({ path: path.join(HERE, "formulas", "eq_C.png"), x: 9.52 - c.w / 2, y: 3.0 + Math.max(a.h, b.h) + 0.22,
               w: c.w, h: c.h });
  conclusion(s, "开环特征多项式 s^2(s^2+ω^2)，极点为 0、0、±jω：临界稳定，必须反馈。");
  pageNum(s, 2, 13);
  s.addNotes("模型是 C-W 方程，四个状态、两路推力、两个位置输出。算出特征多项式 s²(s²+ω²)，两个极点在原点、一对在虚轴，属于临界稳定：不发散也回不到零，所以必须做状态反馈。");
  return s;
}

function p3() {
  const s = pres.addSlide();
  s.background = { color: "FFFFFF" };
  eyebrow(s, "实验一 · 状态反馈极点配置");
  slideTitle(s, "判别单通道推力下的能控性");

  const head = (t) => ({ text: t, options: { bold: true, color: "FFFFFF", fill: { color: NAVY }, fontSize: 15, align: "center" } });
  const cell = (t, o = {}) => ({ text: t, options: Object.assign({ color: INK, fontSize: 15, align: "center" }, o) });
  s.addTable([
    [head("工况"), head("rank 𝒞"), head("结论")],
    [cell("两路推力"), cell("4", { bold: true }), cell("能控，可任意配置极点", { align: "left" })],
    [cell("仅径向推力"), cell("3", { bold: true, color: AMBER }), cell("不能控", { align: "left", bold: true })],
    [cell("仅切向推力"), cell("4", { bold: true }), cell("能控", { align: "left" })],
  ], { x: 0.7, y: 1.9, w: 7.5, colW: [2.3, 1.6, 3.6], rowH: 0.7, fontFace: CN, valign: "middle",
       margin: 0.1, border: { type: "solid", color: LINE, pt: 1 }, fill: { color: "FFFFFF" } });

  s.addText(rich("不可控模态：λ = 0，左零向量 w ∝ [2,0,0,1]^T，右特征向量 v = e_3（切向位置）。"), {
    x: 0.7, y: 4.85, w: 7.5, h: 0.4, fontFace: CN, fontSize: 12, color: INK, margin: 0 });
  s.addText(rich("PBH 判据：rank[A−λI, b_1] = 3 < 4；ω = 0.0011 时三个秩仍是 4 / 3 / 4。"), {
    x: 0.7, y: 5.3, w: 7.5, h: 0.4, fontFace: CN, fontSize: 12, color: INK, margin: 0 });

  img(s, path.join(FIG01, "ex_01_poles.png"), 8.55, 1.95, 3.95, 3.95);
  caption(s, "开环极点：两个在原点、一对在虚轴（ω = 1）", { x: 8.55, y: 5.98, w: 3.95, h: 0.6 });
  conclusion(s, "只开径向时秩为 3：切向位置的常值漂移，径向推力拉不回来。");
  pageNum(s, 3, 13);
  s.addNotes("两路推力秩为 4，能控。只开径向秩只有 3：径向力不出现在切向方程里，切向位置只能靠耦合项间接带动，而丢的正是常值漂移。只开切向反而满秩；换成真实角速度结论不变。");
  return s;
}

function p4() {
  const s = pres.addSlide();
  s.background = { color: "FFFFFF" };
  eyebrow(s, "实验一 · 状态反馈极点配置");
  slideTitle(s, "求解两组反馈增益 K");

  card(s, { x: 0.7, y: 1.8, w: 7.3, h: 4.45 });
  s.addText("期望极点 {−2±j, −2, −2}", { x: 1.0, y: 1.96, w: 5, h: 0.3, fontFace: CN, fontSize: 11, color: MUTED, margin: 0 });
  putEq(s, "eq_gamma", 4.35, 2.3, 6.5);
  const k1 = eqSize("eq_k1", 4.5, 1.2);
  s.addImage({ path: path.join(HERE, "formulas", "eq_k1.png"), x: 4.35 - k1.w / 2, y: 2.95, w: k1.w, h: k1.h });
  const k2 = eqSize("eq_k2", 3.9, 1.2);
  s.addImage({ path: path.join(HERE, "formulas", "eq_k2.png"), x: 4.35 - k2.w / 2, y: 3.0 + k1.h + 0.22, w: k2.w, h: k2.h });
  s.addText(rich("特征多项式偏差：K₁ 2.5×10^−14，K₂ 1.4×10^−14"), {
    x: 1.0, y: 5.55, w: 6.6, h: 0.35, fontFace: CN, fontSize: 12, color: MUTED, margin: 0 });

  bullets(s, [
    ["要求", "：闭环极点集合为 {−2±j, −2, −2}。"],
    ["K₁", "：课件 4.4 公式，ν = (2,2)，本例 T、τ 恰好是单位阵。"],
    ["K₂", "：令 A+BK 按通道块对角，K 由目标矩阵减 A 读出。"],
    ["核对", "：两组闭环系统的特征多项式都等于 γ(s)。"],
  ], { x: 8.3, y: BODY_TOP, w: 4.3, h: 3.6 });

  stat(s, 8.3, 4.72, "8 > 4", "K 的元素个数 vs 特征值约束个数", 4.2);
  conclusion(s, "两组 K 都精确实现同一组极点，但解不唯一：4 维解族。");
  pageNum(s, 4, 13);
  s.addNotes("极点是负二加减 j 和二重负二。第一组用课件四点四公式，能控性指数集是二二，坐标变换阵恰好是单位阵。第二组直接令 A 加 BK 块对角，K 从目标矩阵减 A 读出来。两组特征多项式完全相同。");
  return s;
}

function p5() {
  const s = pres.addSlide();
  s.background = { color: "FFFFFF" };
  eyebrow(s, "实验一 · 状态反馈极点配置");
  slideTitle(s, "求解初始状态响应并与 Simulink 对比");

  const st = { w: 3.95, h: 3.95 / 1.549 };
  img(s, path.join(FIG01, "ex_01_states_K1.png"), 0.7, 1.85, st.w, st.h);
  caption(s, "状态响应 x(t)：x₀ = [1,0,0,0]^T，闭环系统用 K₁", { x: 0.7, y: 1.85 + st.h + 0.08, w: 3.95, h: 0.28 });
  const ct = { w: 3.6, h: 3.6 / 2.855 };
  img(s, path.join(FIG01, "ex_01_controls_K1.png"), 0.7, 4.8, ct.w, ct.h);
  caption(s, "控制量 u(t) = K₁x(t)：切向峰值 20 m/s²", { x: 0.7, y: 6.12, w: 3.6, h: 0.26 });

  bullets(s, [
    ["解析解", "：x(t) = e^{(A+BK)t}x_0，u(t) = Kx(t)，线性定常系统即精确解。"],
    ["收敛", "：位置约 5.06 s 进入 2% 误差带，末态 ‖x(10)‖ = 1.7×10^−6。"],
    ["耦合", "：切向位置被径向误差带出，最大约 0.63 m。"],
    ["核对", "：Simulink 与解析解状态偏差 8.7×10^−11、控制偏差 2.4×10^−10。"],
  ], { x: 5.0, y: BODY_TOP, w: 7.6, h: 3.2 });

  stat(s, 5.0, 5.05, "5.06 s", "2% 调节时间", 2.2);
  s.addText(rich("1.7×10^−6"), { x: 7.4, y: 5.05, w: 2.2, h: 0.7, fontFace: CN, fontSize: 26,
                                 bold: true, color: NAVY, margin: 0 });
  s.addText("末态 ‖x(10)‖", { x: 7.4, y: 5.63, w: 2.2, h: 0.5, fontFace: CN, fontSize: 11, color: MUTED, margin: 0 });
  s.addText(rich("8.7×10^−11"), { x: 9.7, y: 5.05, w: 2.9, h: 0.7, fontFace: CN, fontSize: 26,
                                  bold: true, color: NAVY, margin: 0 });
  s.addText("Simulink 偏差（状态）", { x: 9.7, y: 5.63, w: 2.9, h: 0.5, fontFace: CN, fontSize: 11, color: MUTED, margin: 0 });
  conclusion(s, "解析解与 Simulink 吻合到 10^−11：模型、增益、仿真三者自洽。");
  pageNum(s, 5, 13);
  s.addNotes("初值取径向偏一米。解析解就是 e 的 Ac t 次方乘初值。位置约五秒进百分之二误差带，切向被带出零点六三米。Simulink 搭同一模型，状态偏差八点七乘十的负十一次方，三者自洽。");
  return s;
}

function p6() {
  const s = pres.addSlide();
  s.background = { color: "FFFFFF" };
  eyebrow(s, "实验一 · 状态反馈极点配置");
  slideTitle(s, "对比两组 K 的过渡过程");

  const iw = 6.0, ih = iw / 1.549;
  img(s, path.join(FIG01, "ex_01_K1_vs_K2.png"), 0.7, 1.9, iw, ih);
  caption(s, "同一组极点下 K₁（公式解）与 K₂（解耦型）的响应对比", { x: 0.7, y: 1.9 + ih + 0.1, w: iw, h: 0.3 });

  const head = (t) => ({ text: t, options: { bold: true, color: "FFFFFF", fill: { color: NAVY }, fontSize: 12, align: "center" } });
  const cell = (t, o = {}) => ({ text: t, options: Object.assign({ color: INK, fontSize: 13, align: "center" }, o) });
  s.addTable([
    [head("增益"), head("径向峰值"), head("切向峰值"), head("调节时间"), head("切向 y")],
    [cell("K₁", { bold: true }), cell("4.85"), cell("20.0", { color: AMBER, bold: true }), cell("5.06 s"), cell("最大 0.63 m")],
    [cell("K₂", { bold: true }), cell("8.00"), cell("1.77", { bold: true }), cell("2.50 s"), cell("≡ 0")],
  ], { x: 6.85, y: 1.95, w: 5.75, colW: [0.85, 1.25, 1.25, 1.3, 1.1], rowH: 0.62, fontFace: CN,
       valign: "middle", margin: 0.05, border: { type: "solid", color: LINE, pt: 1 }, fill: { color: "FFFFFF" } });

  bullets(s, [
    ["K₁", "：公式解不关心通道耦合，径向误差被带进切向，切向推力峰值接近 2g。"],
    ["K₂", "：两个通道拆开，切向位置恒为零，切向推力降到 1.77。"],
    ["原因", "：极点只定模态衰减快慢，模态在各状态的分额由特征向量决定。"],
  ], { x: 7.0, y: 4.15, w: 5.6, h: 2.2 });

  conclusion(s, "极点定快慢，特征向量定形态：选哪组 K 决定执行机构负担。");
  pageNum(s, 6, 13);
  s.addNotes("两组极点相同，曲线差很远。K1 把径向误差带进切向，切向推力峰值二十，接近两个 g；K2 两通道拆开，切向位置恒为零，推力一点七七，调节时间二点五秒。");
  return s;
}

function p7() {
  const s = pres.addSlide();
  s.background = { color: "FFFFFF" };
  eyebrow(s, "实验二 · 输入输出解耦控制");
  slideTitle(s, "求相对阶与解耦矩阵，判断解耦条件");

  bullets(s, [
    ["相对阶", "：两个输出求导两次才出现控制量，d_1 = d_2 = 2。"],
    ["解耦矩阵", "：B* = CAB = I_2，非奇异，静态解耦条件满足。"],
    ["剩余动态", "：Σd_i = 4 = n，系统没有剩余动态。"],
    ["含义", "：可以精确解耦，且极点仍可任意配置。"],
  ], { x: 0.75, y: BODY_TOP, w: 5.1, h: 3.9 });

  card(s, { x: 6.2, y: 1.8, w: 6.6, h: 3.15 });
  s.addText("输出求导两次后出现控制量", { x: 6.5, y: 1.96, w: 5, h: 0.3, fontFace: CN, fontSize: 11, color: MUTED, margin: 0 });
  putEq(s, "eq_relorder", 9.5, 2.3, 5.6);
  putEq(s, "eq_bstar", 9.5, 3.35, 6.0);

  conclusion(s, "相对阶之和正好等于系统阶数：解耦之外，极点还能随便配。");
  pageNum(s, 7, 13);
  s.addNotes("两个输出都是位置量，求导两次才出现控制量，相对阶都是二。解耦矩阵 CAB 恰好是单位阵，非奇异；两个相对阶加起来正好是四，等于系统阶数，说明没有剩余动态。");
  return s;
}

function p8() {
  const s = pres.addSlide();
  s.background = { color: "FFFFFF" };
  eyebrow(s, "实验二 · 输入输出解耦控制");
  slideTitle(s, "设计解耦控制律 u = Kx + Hv");

  card(s, { x: 0.7, y: 1.8, w: 7.2, h: 3.75 });
  s.addText("令闭环系统按通道块对角", { x: 1.0, y: 1.96, w: 5, h: 0.3, fontFace: CN, fontSize: 11, color: MUTED, margin: 0 });
  putEq(s, "eq_blkdiag", 4.3, 2.35, 6.3);
  const k2 = eqSize("eq_k2", 4.3, 1.2);
  s.addImage({ path: path.join(HERE, "formulas", "eq_k2.png"), x: 4.3 - k2.w / 2, y: 3.75, w: k2.w, h: k2.h });
  s.addText("s²+4s+5 ⇒ −2±j　　s²+4s+4 ⇒ −2, −2", { x: 1.0, y: 5.05, w: 6.4, h: 0.35,
                                                      fontFace: CN, fontSize: 12, color: MUTED, margin: 0 });

  bullets(s, [
    ["目标", "：u = Kx + Hv 下传递矩阵为给定对角阵。"],
    ["读法", "：B 的第 2、4 行是单位行，K 由目标矩阵减 A 直接读出。"],
    ["对照", "：这组 K 与实验一的 K₂ 完全相同。"],
  ], { x: 8.2, y: BODY_TOP, w: 4.4, h: 3.2 });

  conclusion(s, "解耦增益 K 就是实验一的 K₂：两条路走到同一个矩阵。");
  pageNum(s, 8, 13);
  s.addNotes("没有剩余动态，最直接的做法是让 A 加 BK 变成两个二阶块，分母就是题给的两个多项式。B 的第二第四行是单位行，K 从目标矩阵减 A 读出来，读出来正好是实验一的 K2。");
  return s;
}

function p9() {
  const s = pres.addSlide();
  s.background = { color: "FFFFFF" };
  eyebrow(s, "实验二 · 输入输出解耦控制");
  slideTitle(s, "验证解耦后的传递矩阵与题目要求一致");

  card(s, { x: 0.7, y: 1.8, w: 11.95, h: 2.35 });
  s.addText("取 H = I，解耦后的传递矩阵", { x: 1.0, y: 1.94, w: 6, h: 0.3, fontFace: CN, fontSize: 11, color: MUTED, margin: 0 });
  putEq(s, "eq_gs", 6.67, 2.55, 9.6);

  bullets(s, [
    ["比对点", "：s = 0、0.5、1、2、1+2j、3j，共六个点。"],
    ["偏差", "：与题目要求逐点比较，最大偏差 4.2×10^−17，等于机器精度。"],
    ["结论", "：H = I 时 G(s) 严格等于两个二阶环节的对角和，解耦是精确的。"],
  ], { x: 0.75, y: 4.45, w: 11.6, h: 1.8 });

  conclusion(s, "传递矩阵与题目要求逐点一致，偏差在 10^−17 量级。");
  pageNum(s, 9, 13);
  s.addNotes("取 H 为单位阵，传递矩阵是两个二阶环节的对角和。在六个采样点上与题目要求比对，最大偏差四点二乘十的负十七次方，等于机器精度，说明解耦是精确的而不是近似。");
  return s;
}

function p10() {
  const s = pres.addSlide();
  s.background = { color: "FFFFFF" };
  eyebrow(s, "实验二 · 输入输出解耦控制");
  slideTitle(s, "计算单位阶跃响应（y、x、u）");

  const ow = 3.8, oh = ow / 1.674;
  img(s, path.join(FIG02, "ex_02_outputs_H1.png"), 0.7, 1.85, ow, oh);
  caption(s, "输出 y(t)：稳态 0.2000 / 0.2500", { x: 0.7, y: 1.85 + oh + 0.06, w: ow, h: 0.28 });
  const sw = 3.55, sh = sw / 1.552;
  img(s, path.join(FIG02, "ex_02_states_H1.png"), 4.75, 1.85, sw, sh);
  caption(s, "状态 x(t)", { x: 4.75, y: 1.85 + sh + 0.06, w: sw, h: 0.28 });
  const cw = 3.55, ch = cw / 2.881;
  img(s, path.join(FIG02, "ex_02_controls_H1.png"), 4.75, 4.75, cw, ch);
  caption(s, "控制 u(t)：峰值 1.00 / 1.00", { x: 4.75, y: 4.75 + ch + 0.06, w: cw, h: 0.28 });

  bullets(s, [
    ["参考", "：v(t) = [1(t), 1(t)]^T，H = I。"],
    ["仿真", "：Simulink 与解析解偏差 x 8.5×10^−13、y 3.5×10^−13、u 1.4×10^−12。"],
    ["现状", "：稳态只有 0.2 和 0.25，还没到题目要求的 1。"],
  ], { x: 8.85, y: BODY_TOP, w: 3.8, h: 3.0 });

  conclusion(s, "H = I 时两条曲线稳态只有 0.2 和 0.25，因为直流增益是 1/5 和 1/4。");
  pageNum(s, 10, 13);
  s.addNotes("给两路单位阶跃，H 取单位阵。输出、状态、控制三张图和 Simulink 都对过了，偏差在十的负十二到十三次方。但稳态只有零点二和零点二五，因为两个环节的直流增益是五分之一和四分之一。");
  return s;
}

function p11() {
  const s = pres.addSlide();
  s.background = { color: "FFFFFF" };
  eyebrow(s, "实验二 · 输入输出解耦控制");
  slideTitle(s, "调整 H 使稳态输出为 1");

  const w1 = 3.95, h1 = w1 / 1.674;
  img(s, path.join(FIG02, "ex_02_outputs_H1.png"), 0.7, 1.9, w1, h1);
  caption(s, "H = I：稳态 0.2000 / 0.2500", { x: 0.7, y: 1.9 + h1 + 0.08, w: w1, h: 0.28 });
  const w3 = 3.95, h3 = w3 / 1.667;
  img(s, path.join(FIG02, "ex_02_outputs_H3.png"), 4.75, 1.9, w3, h3);
  caption(s, "H = diag(5,4)：稳态 1.0000 / 1.0000", { x: 4.75, y: 1.9 + h3 + 0.08, w: w3, h: 0.28 });

  bullets(s, [
    ["原因", "：两个二阶环节直流增益为 1/5 与 1/4，H = I 时按此缩放。"],
    ["取法", "：H = diag(5,4)，则 G(0)H = I_2。"],
    ["代价", "：控制峰值升到 5.0 / 4.0，动态形状不变。"],
  ], { x: 8.9, y: BODY_TOP, w: 3.7, h: 2.3 });
  putEq(s, "eq_h", 10.75, 4.75, 3.4);

  conclusion(s, "H 只改参考到输入的增益：稳态从 0.2/0.25 提到 1，闭环极点不动。");
  pageNum(s, 11, 13);
  s.addNotes("稳态不到一是因为直流增益。把 H 取成对角五和四，G(0)H 就是单位阵，两条曲线稳态都到一。H 只改参考到输入的增益，闭环极点没动，所以动态形状不变，控制峰值升到五和四。");
  return s;
}

function p12() {
  const s = pres.addSlide();
  s.background = { color: "FFFFFF" };
  eyebrow(s, "实验二 · 输入输出解耦控制");
  slideTitle(s, "验证两通道解耦");

  const iw = 6.6, ih = iw / 1.904;
  img(s, path.join(FIG02, "ex_02_decoupling_check.png"), 0.7, 1.9, iw, ih);
  caption(s, "只加 v₁ = 1(t)、v₂ = 0：y₁ 正常响应，y₂ 全程为零", { x: 0.7, y: 1.9 + ih + 0.1, w: iw, h: 0.3 });

  stat(s, 7.85, 2.0, "0", "max|y₂|（解析解与 Simulink 都是 0）", 4.6);
  bullets(s, [
    ["做法", "：只给第一路参考，第二路置零。"],
    ["结果", "：y₁ 按 s²+4s+5 响应，y₂ 恒等于零。"],
    ["意义", "：两个通道真正独立，不是幅值小到看不见。"],
  ], { x: 7.85, y: 3.55, w: 4.7, h: 2.4 });

  conclusion(s, "只加一路参考，另一路输出严格为零：输入输出解耦成立。");
  pageNum(s, 12, 13);
  s.addNotes("这一页是解耦的直接证据。只给第一路阶跃，第一路按两个二阶环节正常响应，第二路输出全程为零，解析解和 Simulink 的最大绝对值都是零。");
  return s;
}

function p13() {
  const s = pres.addSlide();
  s.background = { color: NAVY };
  s.addShape(pres.ShapeType.ellipse, { x: 10.6, y: 4.9, w: 4.6, h: 4.6, fill: { color: NAVY },
                                       line: { color: ICE, width: 0.8, transparency: 70 } });
  s.addText("实验结论与下一步", { x: 0.85, y: 0.9, w: 8, h: 0.6, fontFace: CN, fontSize: 30, bold: true, color: "FFFFFF", margin: 0 });
  const runs = [];
  [
    ["建模与能控性｜", "开环临界稳定；两路推力能控，只开径向秩为 3，切向漂移不可控。"],
    ["极点配置｜", "两组 K 都实现 {−2±j, −2, −2}；8 个元素、4 个约束，解是四维族。"],
    ["解耦控制｜", "块对角 K 即 K₂；H = diag(5,4) 使稳态为 1；只加一路参考另一路恒为零。"],
    ["下一步｜", "速度量不到，实验五、六用观测器重构状态后再做反馈。"],
  ].forEach(([lead, body]) => {
    runs.push({ text: lead, options: { bold: true, color: AMBER } });
    rich(body).forEach((r) => runs.push(r));
    const tail = runs[runs.length - 1];
    tail.options = Object.assign({}, tail.options, { breakLine: true, paraSpaceAfter: 22 });
  });
  s.addText(runs, { x: 0.85, y: 1.95, w: 10.4, h: 3.4, fontFace: CN, fontSize: 17, color: ICE,
                    lineSpacingMultiple: 1.5, margin: 0 });
  s.addText("谢谢老师", { x: 0.85, y: 6.2, w: 4, h: 0.5, fontFace: CN, fontSize: 20, bold: true, color: AMBER, margin: 0 });
  s.addNotes("三句收尾：开环临界稳定、只开径向不可控；两组 K 都实现要求极点，解不唯一；块对角 K 直接给出解耦，加 H 后稳态为一。下一步用观测器解决速度不可测。");
  return s;
}

/* ---------- 生成 ---------- */

cover(); p2(); p3(); p4(); p5(); p6(); p7(); p8(); p9(); p10(); p11(); p12(); p13();

if (pres.slides.length !== 13) throw new Error(`幻灯片数不对：${pres.slides.length}`);

const outArg = process.argv.indexOf("--out");
const outFile = path.resolve(outArg > 0 ? process.argv[outArg + 1] : "deck.pptx");
fs.mkdirSync(path.dirname(outFile), { recursive: true });
pres.writeFile({ fileName: outFile }).then((f) => console.log("written:", f));
