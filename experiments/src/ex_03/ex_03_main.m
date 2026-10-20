%% ===================== 实验三：倒立摆输出调节（静态状态反馈） =====================
%
%   x' = A x + B u,  y = [1 0 0 0] x,  r(t) = 0.1 cos 2t
%
%   设计在 ex_03_design.py 里完成（算法全在 Python 仓的 linear_system_utils）：
%       1) 极点配置定 K：u = K x，闭环极点在 -5+-3j、-5+-2j
%       2) 解调节器方程 Pi A_v = A Pi + B Gamma, C Pi = Q     （稳态 x = Pi w）
%       3) K_v = Gamma - K Pi，控制律 u = K x + K_v w
%
%   本脚本只做"实验专用"的活：
%       * 用调节器方程残差、能调节性判据独立复核设计量
%       * 跑 Simulink（ex_03_run_sim.m 只在内存里改模型），与增广闭环的解析解逐点对照
%       * 出图：r/y 跟踪、误差、状态、控制量（含前馈分量）→ bin/data/ex_03/
%       * 对照组：把 k_v 换成课件图上的值再跑一遍，看还能不能精确跟踪
%
%   运行：
%       matlab -batch "cd('experiments'); run('src/ex_03/ex_03_main.m')"

clear; close all; clc;

thisDir = fileparts(mfilename('fullpath'));
srcDir  = fileparts(thisDir);
expRoot = fileparts(srcDir);
addpath(fullfile(srcDir, 'common'));

outDir = fullfile(expRoot, 'bin', 'data', 'ex_03');
if ~exist(outDir, 'dir'), mkdir(outDir); end
logFid = fopen(fullfile(outDir, 'ex_03_log.txt'), 'w', 'n', 'UTF-8');

jsonFile = fullfile(thisDir, 'ex_03_design.json');
if ~exist(jsonFile, 'file')
    error('找不到 %s，请先运行：python experiments\\src\\ex_03\\ex_03_design.py', jsonFile);
end
d = jsondecode(fileread(jsonFile));

A  = d.A;  B = d.B;  C = d.C;  D = d.D;
c  = d.c(:).';                       % y = c x
A_v = d.A_v;  Initial_v = d.Initial_v(:);
rg = d.r(:).';                       % r = rg * w
K  = d.k_x;  Kv = d.k_v;
Pi = d.design.Pi;  Gam = d.design.Gamma;
Tf = 10;                             % 10 s ≈ 3.2 个参考周期

say = @(varargin) fprintf(logFid, varargin{:});

%% ---------------- 设计量复核 ----------------
say('【对象】倒立摆 x'' = A x + B u,  y = [1 0 0 0] x\n');
say('A =\n%s\n\nB =\n%s\n\n', fmtm(A), fmtm(B));
say('【参考】w'' = A_v w, w(0) = [1 0]'',  r = 0.1 w_1 = 0.1 cos 2t\n');
say('A_v =\n%s\n\n', fmtm(A_v));

say('【1】极点配置：u = K x\n');
say('  K   = %s\n', mat2str(K, 8));
say('  课件 kx = %s\n', mat2str(d.design.k_x_handout, 6));
say('  eig(A+B*K) = %s\n', mat2str(sort(eig(A + B * K)), 6));
say('  期望极点    = %s\n', mat2str(sort(complex([d.design.poles.real], [d.design.poles.imag])), 6));
say('  A+B*K 稳定 = %d\n\n', all(real(eig(A + B * K)) < 0));

say('【2】调节器方程 Pi A_v = A Pi + B Gamma,  C Pi = Q\n');
say('  Pi =\n%s\n', fmtm(Pi));
say('  Gamma = %s\n', mat2str(Gam, 8));
say('  C*Pi (应等于 [0.1 0]) = %s\n', mat2str(c * Pi, 8));
say('  残差 Pi A_v - A Pi - B Gamma   = %.2e\n', ...
    max(abs(Pi * A_v - A * Pi - B * Gam), [], 'all'));
say('  残差 C Pi - Q                  = %.2e\n', ...
    max(abs(c * Pi - d.design.Q), [], 'all'));

% 能调节性判据：外系统频率 +-2j 处 rank([A-lambda I, B; C, 0]) 必须满
need = d.design.solvability_need;
for lam = unique(round(eig(A_v), 10)).'
    M = [A - lam * eye(size(A)), B; c, 0];
    say('  能调节性 rank([A-(%s)I, B; C, 0]) = %d / %d\n', ...
        num2str(lam), rank(M), need);
end
say('\n');

say('【3】静态输出调节增益 K_v = Gamma - K Pi\n');
say('  K_v = %s\n', mat2str(Kv, 8));
say('  课件 kv = %s （不满足调节器方程，见 references）\n\n', ...
    mat2str(d.design.k_v_handout, 6));

%% ---------------- 仿真 ----------------
say('【4】Simulink 仿真（%g s）\n', Tf);
sim1 = struct('ok', false, 'err', '', 't', [], 'X', [], 'U', [], 'R', [], 'Y', []);
try
    [sim1.t, sim1.X, sim1.U, sim1.R, sim1.Y] = ex_03_run_sim(d, Tf);
    sim1.ok = true;
catch err
    sim1.err = err.message;
end

if sim1.ok
    say('  采样点数 = %d, 步长中位数 = %.4g s\n', numel(sim1.t), median(diff(sim1.t)));

    % 增广闭环 [x; w] 的解析解，与仿真在仿真自己的时间点上逐点比
    Aaug = [A + B * K, B * Kv; zeros(size(A_v, 1), size(A, 1)), A_v];
    z0 = [zeros(size(A, 1), 1); Initial_v];
    n = numel(sim1.t);
    Za = zeros(n, size(Aaug, 1));
    for k = 1:n
        Za(k, :) = (expm(Aaug * sim1.t(k)) * z0).';
    end
    Xa = Za(:, 1:size(A, 1));
    Wa = Za(:, size(A, 1) + 1:end);
    Ua = (K * Xa.' + Kv * Wa.').';
    Ra = (rg * Wa.').';
    Ya = (c * Xa.').';

    eX = max(abs(sim1.X - Xa), [], 'all');
    eU = max(abs(sim1.U - Ua), [], 'all');
    eY = max(abs(sim1.Y - Ya), [], 'all');
    say('  与解析解 expm([Ac B*Kv; 0 A_v] t) 的最大偏差：状态 %.2e, 控制 %.2e, 输出 %.2e\n', eX, eU, eY);

    % 跟踪误差
    e = sim1.Y - sim1.R;
    tail = sim1.t > Tf - 2;
    say('  max|r| = %.4f, max|y| = %.4f, max|u| = %.4f\n', ...
        max(abs(sim1.R)), max(abs(sim1.Y)), max(abs(sim1.U)));
    say('  max|y-r| 全程 = %.3e, 末段(>%g s) = %.3e\n', ...
        max(abs(e)), Tf - 2, max(abs(e(tail))));

    % 稳态：x 应收敛到 Pi w
    Xss = (Pi * Wa.').';
    say('  max|x - Pi w| 末段 = %.3e （Pi w 是调节器方程给出的稳态轨迹）\n', ...
        max(abs(sim1.X(tail, :) - Xss(tail, :)), [], 'all'));
else
    say('  仿真失败：%s\n', sim1.err);
end

% 对照组：把 k_v 换成课件图上的值
d2 = d;
d2.k_v = d.design.k_v_handout;
sim2 = struct('ok', false, 'err', '');
try
    [t2, X2, U2, R2, Y2] = ex_03_run_sim(d2, Tf);
    sim2.ok = true;
    e2 = Y2 - R2; tail2 = t2 > Tf - 2;
    say('  [对照] 用课件 kv = %s：末段 max|y-r| = %.3e（我们的 kv: %.3e）\n', ...
        mat2str(d.design.k_v_handout, 6), max(abs(e2(tail2))), max(abs(e(tail))));
catch err
    say('  [对照] 未运行：%s\n', err.message);
end
say('\n');

%% ---------------- 出图 ----------------
if ~sim1.ok
    say('仿真没跑通，跳过出图；模型/变量问题见上面的错误信息。\n');
else
    % 图1：跟踪 + 误差
    fig1 = figure('Position', [60 60 1000 560], 'Visible', 'off');
    tiledlayout(fig1, 2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
    nexttile;
    plot(sim1.t, sim1.R, 'LineWidth', 1.4); hold on;
    plot(sim1.t, sim1.Y, '--', 'LineWidth', 1.8);
    grid on; ylabel('y = s  [m]');
    legend({'r(t) = 0.1cos2t', 'y(t)'}, 'Location', 'best');
    title('实验三 输出调节：参考与输出');
    nexttile;
    plot(sim1.t, e, 'LineWidth', 1.4); hold on; yline(0, ':');
    grid on; xlabel('t [s]'); ylabel('e = y - r  [m]');
    title(sprintf('跟踪误差（末段 max|e| = %.1e）', max(abs(e(tail)))));
    export_figure(fig1, outDir, 'ex_03_tracking');

    % 图2：四个状态 + 解析解
    lbl = {'x_1 = s  [m]', 'x_2 = ds/dt  [m/s]', 'x_3 = \theta  [rad]', 'x_4 = d\theta/dt  [rad/s]'};
    fig2 = figure('Position', [60 60 1000 620], 'Visible', 'off');
    tiledlayout(fig2, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    for i = 1:size(sim1.X, 2)
        nexttile;
        plot(sim1.t, sim1.X(:, i), 'LineWidth', 1.6); hold on;
        plot(sim1.t, Xa(:, i), '--', 'LineWidth', 1.1);
        grid on; xlabel('t [s]'); ylabel(lbl{i});
        if i == 1
            legend({'Simulink', '解析解'}, 'Location', 'best');
        end
    end
    sgtitle('实验三 状态响应（u = Kx + K_v w）');
    export_figure(fig2, outDir, 'ex_03_states');

    % 图3：控制量及其两个分量
    fig3 = figure('Position', [60 60 1000 480], 'Visible', 'off');
    tiledlayout(fig3, 1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    nexttile;
    plot(sim1.t, sim1.U, 'LineWidth', 1.6); hold on;
    plot(sim1.t, Ua, '--', 'LineWidth', 1.1);
    grid on; xlabel('t [s]'); ylabel('u');
    legend({'Simulink', '解析解'}, 'Location', 'best');
    title('控制量');
    nexttile;
    uK = (K * sim1.X.').';
    uF = (Kv * Wa.').';
    plot(sim1.t, uK, 'LineWidth', 1.4); hold on;
    plot(sim1.t, uF, 'LineWidth', 1.4);
    grid on; xlabel('t [s]'); ylabel('分量');
    legend({'状态反馈 Kx', '参考前馈 K_v w'}, 'Location', 'best');
    title('控制量的两个分量');
    export_figure(fig3, outDir, 'ex_03_control');
end

%% ---------------- 存数据与日志 ----------------
save(fullfile(outDir, 'ex_03_data.mat'), 'A', 'B', 'C', 'D', 'A_v', 'Initial_v', ...
     'rg', 'K', 'Kv', 'Pi', 'Gam', 'Tf');
if sim1.ok
    save(fullfile(outDir, 'ex_03_data.mat'), 'eX', 'eU', 'eY', '-append');
    t = sim1.t; X = sim1.X; U = sim1.U; R = sim1.R; Y = sim1.Y;
    save(fullfile(outDir, 'ex_03_data.mat'), 't', 'X', 'U', 'R', 'Y', '-append');
end
fclose(logFid);

if sim1.ok
    fprintf(1, 'K   = %s\n', mat2str(round(K, 6)));
    fprintf(1, 'Kv  = %s\n', mat2str(round(Kv, 6)));
    fprintf(1, 'eig(A+BK) = %s\n', mat2str(round(sort(eig(A + B * K)), 6)));
    fprintf(1, 'max|y-r| tail = %.3e\n', max(abs(e(tail))));
    fprintf(1, 'sim vs analytic: X %.3e  U %.3e  Y %.3e\n', eX, eU, eY);
end
fprintf(1, 'figures + data -> %s\n', outDir);

%% ---------------- 局部函数 ----------------
function s = fmtm(M)
rows = arrayfun(@(i) mat2str(M(i, :), 6), 1:size(M, 1), 'UniformOutput', false);
s = strjoin(rows, newline);
end
