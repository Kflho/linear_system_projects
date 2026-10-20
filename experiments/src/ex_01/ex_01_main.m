%% ===================== 实验一：卫星状态反馈极点配置 =====================
%
%   x = [x, xdot, y, ydot]'  径向/切向位置与速度
%   u = [u_r, u_t]'          径向/切向轨控发动机推力
%   u = K x                  状态反馈，闭环 A + B K
%
%  数学与设计（模型、能控性、两组 K）在 ex_01_design.py 里完成，
%  那里复用 Python 仓库的 linear_system_utils；本脚本只负责：
%      * 独立复核（特征多项式、MATLAB place 对照）
%      * 解析初始状态响应 x(t)=expm(Ac t)x0, u(t)=Kx(t)
%      * 用 ex_01_sim.slx 跑 Simulink，并与解析解逐点对照
%      * 出图、存数据、写日志
%
%  运行：
%      matlab -batch "cd('experiments'); run('src/ex_01/ex_01_main.m')"

clear; close all; clc;

thisDir = fileparts(mfilename('fullpath'));
srcDir  = fileparts(thisDir);
expRoot = fileparts(srcDir);
addpath(fullfile(srcDir, 'common'));

outDir = fullfile(expRoot, 'bin', 'data', 'ex_01');
if ~exist(outDir, 'dir'), mkdir(outDir); end
logFid = fopen(fullfile(outDir, 'ex_01_log.txt'), 'w', 'n', 'UTF-8');

jsonFile = fullfile(thisDir, 'ex_01_design.json');
if ~exist(jsonFile, 'file')
    error('找不到 %s，请先运行：python experiments\\src\\ex_01\\ex_01_design.py', jsonFile);
end
d = jsondecode(fileread(jsonFile));

A  = d.A;  B = d.B;  C = d.C;  D = d.D;
K1 = d.K1; K2 = d.K2;
x0 = d.x0(:);
poles = d.poles.real(:) + 1i * d.poles.imag(:);
Tf = 10;

say = @(varargin) fprintf(logFid, varargin{:});

%% ---------------- 任务 (1)(2)(3)：模型与能控性复核 ----------------
say('【任务 1】状态空间模型（omega = %g rad/s）\n', d.omega_design);
say('A =\n%s\n\nB =\n%s\n\nC =\n%s\nD = 0\n\n', fmtm(A), fmtm(B), fmtm(C));

r = d.ranks;
say('【任务 2/3】能控性 rank(ctrb(A,B))\n');
say('  两路推力      : %d\n', r.both);
say('  仅径向 u_r    : %d  -> %s\n', r.radial, ctrl_txt(r.radial == 4));
say('  仅切向 u_t    : %d  -> %s\n', r.tangential, ctrl_txt(r.tangential == 4));
say('  (omega = %g 时为 %d / %d / %d，秩与 omega 无关)\n', ...
    d.omega_real, r.both_real, r.radial_real, r.tangential_real);
say('  仅径向时的不可控模态: lambda = %g\n', d.radial_mode.lambda(1));
say('      左零向量 w'' = %s\n', mat2str(round(d.radial_mode.left_vector.', 4), 4));
say('      右特征向量 v = %s   (切向位置 y，无法由 u_r 改变)\n', ...
    mat2str(round(d.radial_mode.right_vector.', 4), 4));
say('      PBH: rank([A - lambda I, b]) = %d < 4\n\n', d.radial_mode.pbm_rank);

rB = rank(ctrb(A, B));
say('  本机 rank(ctrb(A,B)) = %d（与设计脚本一致：%d）\n\n', rB, r.both);

%% ---------------- 任务 (4)：两组 K 的独立复核 ----------------
cpWant = real(poly(poles));
say('【任务 4】两组反馈增益 K（u = K x）\n');
say('  K1 =\n%s\n  K2 =\n%s\n', fmtm(K1), fmtm(K2));
say('  ||K1 - K2|| = %.4f  （多输入系统极点配置解不唯一）\n', norm(K1 - K2));

cp1 = real(poly(eig(A + B * K1)));
cp2 = real(poly(eig(A + B * K2)));
say('  期望特征多项式     : %s\n', mat2str(cpWant, 6));
say('  K1 闭环特征多项式  : %s   偏差 %.2e\n', mat2str(cp1, 6), max(abs(cp1 - cpWant)));
say('  K2 闭环特征多项式  : %s   偏差 %.2e\n', mat2str(cp2, 6), max(abs(cp2 - cpWant)));
say('  eig(A+B*K1) = %s\n', mat2str(sort(eig(A + B * K1)), 6));
say('  eig(A+B*K2) = %s\n', mat2str(sort(eig(A + B * K2)), 6));

% MATLAB place 是 A - B*K 的约定，取负号后才能与本报告的 u = Kx 比较
Kplace = -place(A, B, poles);
say('  MATLAB place 对照 (取负号): eig(A+B*Kplace) = %s\n', ...
    mat2str(sort(eig(A + B * Kplace)), 6));
say('  ||Kplace - K1|| = %.4f, ||Kplace - K2|| = %.4f\n\n', ...
    norm(Kplace - K1), norm(Kplace - K2));

% K2 的闭环矩阵结构（说明"解耦型"）
say('  A + B*K2 =\n%s\n', fmtm(round(A + B * K2, 12)));
say('  即 blkdiag([0 1; -5 -4], [0 1; -4 -4])：两个解耦的二阶子系统\n');
say('      s^2 + 4s + 5 = (s+2)^2 + 1  ->  -2 +- j\n');
say('      s^2 + 4s + 4 = (s+2)^2      ->  -2, -2\n\n');

%% ---------------- 任务 (5)：初始状态响应 ----------------
[t, X, U] = initial_state_response(A, B, K1, x0, Tf, 800);
[tb, Xb, Ub] = initial_state_response(A, B, K2, x0, Tf, 800);

nx = vecnorm(X, 2, 2) / norm(x0);
k = find(nx > 0.02, 1, 'last');
if isempty(k)
    t2 = 0;
elseif k < numel(t)
    t2 = t(k + 1);
else
    t2 = NaN;
end

say('【任务 5】初始状态响应（x0 = %s）\n', mat2str(x0.', 6));
say('  K1: max|u_r| = %.4f, max|u_t| = %.4f, 2%% 调节时间 %.2f s, |x(%.0f)| = %.2e\n', ...
    max(abs(U(:, 1))), max(abs(U(:, 2))), t2, Tf, norm(X(end, :)));
say('  K2: max|u_r| = %.4f, max|u_t| = %.4f\n', ...
    max(abs(Ub(:, 1))), max(abs(Ub(:, 2))));

%% ---------------- Simulink 对照 ----------------
sim1 = try_sim(A, B, K1, x0, Tf);
sim2 = try_sim(A, B, K2, x0, Tf);
if sim1.ok
    [eX, eU] = compare_log(sim1, A + B * K1, K1, x0);
    say('  Simulink(K1) 与解析解最大偏差: 状态 %.2e, 控制 %.2e\n', eX, eU);
else
    say('  Simulink(K1) 未运行：%s\n', sim1.err);
end
if sim2.ok
    [eX2, eU2] = compare_log(sim2, A + B * K2, K2, x0);
    say('  Simulink(K2) 与解析解最大偏差: 状态 %.2e, 控制 %.2e\n', eX2, eU2);
end
say('\n');

%% ---------------- 出图 ----------------
lbl = {'x_1 = x  [m]', 'x_2 = dx/dt  [m/s]', 'x_3 = y  [m]', 'x_4 = dy/dt  [m/s]'};

fig1 = figure('Position', [60 60 1000 620], 'Visible', 'off');
tiledlayout(fig1, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
for i = 1:4
    nexttile;
    plot(t, X(:, i), 'LineWidth', 1.6); hold on;
    if sim1.ok
        plot(sim1.t, sim1.X(:, i), '--', 'LineWidth', 1.1);
    end
    grid on; xlabel('t [s]'); ylabel(lbl{i});
    if i == 1
        if sim1.ok
            legend({'解析解 expm(A_c t)x_0', 'Simulink'}, 'Location', 'best');
        else
            legend({'解析解 expm(A_c t)x_0'}, 'Location', 'best');
        end
    end
end
sgtitle('实验一 状态响应（K_1，u = K_1 x）');
export_figure(fig1, outDir, 'ex_01_states_K1');

fig2 = figure('Position', [60 60 1000 340], 'Visible', 'off');
tiledlayout(fig2, 1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
ul = {'u_r  [m/s^2]', 'u_t  [m/s^2]'};
for i = 1:2
    nexttile;
    plot(t, U(:, i), 'LineWidth', 1.6); hold on;
    if sim1.ok
        plot(sim1.t, sim1.U(:, i), '--', 'LineWidth', 1.1);
    end
    grid on; xlabel('t [s]'); ylabel(ul{i});
    if i == 1
        if sim1.ok
            legend({'解析解 K_1 x(t)', 'Simulink'}, 'Location', 'best');
        else
            legend({'解析解 K_1 x(t)'}, 'Location', 'best');
        end
    end
end
sgtitle('实验一 控制量（K_1）');
export_figure(fig2, outDir, 'ex_01_controls_K1');

fig3 = figure('Position', [60 60 560 520], 'Visible', 'off');
plot(real(poles), imag(poles), 'o', 'MarkerSize', 10, 'LineWidth', 1.8); hold on;
plot(real(eig(A + B * K1)), imag(eig(A + B * K1)), 'x', 'MarkerSize', 11, 'LineWidth', 1.8);
plot(real(eig(A + B * K2)), imag(eig(A + B * K2)), '+', 'MarkerSize', 11, 'LineWidth', 1.8);
grid on; axis equal;
xlabel('Re'); ylabel('Im');
title('实验一 极点分布');
legend({'期望极点 {-2\pmj, -2, -2}', 'K_1 闭环极点', 'K_2 闭环极点'}, 'Location', 'best');
export_figure(fig3, outDir, 'ex_01_poles');

fig4 = figure('Position', [60 60 1000 620], 'Visible', 'off');
tiledlayout(fig4, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
for i = 1:4
    nexttile;
    plot(t, X(:, i), 'LineWidth', 1.6); hold on;
    plot(tb, Xb(:, i), '--', 'LineWidth', 1.3);
    grid on; xlabel('t [s]'); ylabel(lbl{i});
    if i == 1
        legend({'K_1（公式解）', 'K_2（解耦型解）'}, 'Location', 'best');
    end
end
sgtitle('实验一 同一组极点、两组 K 的响应对比');
export_figure(fig4, outDir, 'ex_01_K1_vs_K2');

%% ---------------- 存数据与日志 ----------------
save(fullfile(outDir, 'ex_01_data.mat'), 'A', 'B', 'C', 'D', 'K1', 'K2', ...
     'poles', 'x0', 't', 'X', 'U', 'tb', 'Xb', 'Ub');
fclose(logFid);

fprintf(1, 'rank(ctrb): both=%d radial=%d tangential=%d\n', r.both, r.radial, r.tangential);
fprintf(1, 'K1 = %s\n', mat2str(round(K1, 6)));
fprintf(1, 'K2 = %s\n', mat2str(round(K2, 6)));
fprintf(1, 'eig(K1) = %s\n', mat2str(round(sort(eig(A + B * K1)), 6)));
fprintf(1, 'eig(K2) = %s\n', mat2str(round(sort(eig(A + B * K2)), 6)));
if sim1.ok
    fprintf(1, 'sim err: X %.3e  U %.3e\n', eX, eU);
end
fprintf(1, 'figures + data -> %s\n', outDir);

%% ---------------- 局部函数 ----------------
function s = ctrl_txt(flag)
if flag, s = '能控'; else, s = '不能控'; end
end

function s = fmtm(M)
rows = arrayfun(@(i) mat2str(M(i, :), 6), 1:size(M, 1), 'UniformOutput', false);
s = strjoin(rows, newline);
end

function res = try_sim(A, B, K, x0, Tf)
res = struct('ok', false, 'err', '', 't', [], 'X', [], 'U', []);
try
    [res.t, res.X, res.U] = ex_01_run_sim(A, B, eye(size(A, 1)), ...
        zeros(size(A, 1), size(B, 2)), K, x0, Tf);
    res.ok = true;
catch err
    res.err = err.message;
end
end

function [eX, eU] = compare_log(simRes, Ac, K, x0)
% 在仿真自己的时间点上算解析解（不做插值，避免采样点稀疏/重复带来的误差）
n  = numel(simRes.t);
Xa = zeros(n, size(Ac, 1));
for k = 1:n
    Xa(k, :) = (expm(Ac * simRes.t(k)) * x0(:)).';
end
Ua = Xa * K.';
eX = max(abs(simRes.X - Xa), [], 'all');
eU = max(abs(simRes.U - Ua), [], 'all');
end
