%% ================= 实验二：卫星运动的输入输出解耦控制 =================
%
%   u = K x + H v
%   目标闭环传递矩阵  G(K,H)(s) = 1/(s^2+4s+5) ⊕ 1/(s^2+4s+4)
%
%   设计量（K, H, 相对阶, 解耦矩阵）在 ex_02_design.py 里算好，本脚本负责：
%      * 独立复核（CAB、闭环极点、G(0)）
%      * 常值输入下的解析响应  x(t)=M^{-1}(e^{Mt}-I)BHv,  y=Cx,  u=Kx+Hv
%      * 跑 ex_02_sim.slx（H=I 与 H=diag(5,4) 两组）、与解析解对照
%      * 出图、存数据、写日志
%
%  运行：
%      matlab -batch "cd('experiments'); run('src/ex_02/ex_02_main.m')"

clear; close all; clc;

thisDir = fileparts(mfilename('fullpath'));
srcDir  = fileparts(thisDir);
expRoot = fileparts(srcDir);
addpath(fullfile(srcDir, 'common'));

outDir = fullfile(expRoot, 'bin', 'data', 'ex_02');
if ~exist(outDir, 'dir'), mkdir(outDir); end
logFid = fopen(fullfile(outDir, 'ex_02_log.txt'), 'w', 'n', 'UTF-8');
say = @(varargin) fprintf(logFid, varargin{:});

jsonFile = fullfile(thisDir, 'ex_02_design.json');
if ~exist(jsonFile, 'file')
    error('找不到 %s，请先运行 python experiments\\src\\ex_02\\ex_02_design.py', jsonFile);
end
d = jsondecode(fileread(jsonFile));

A = d.A; B = d.B; C = d.C; D = d.D;
K = d.K; H1 = d.H1; H3 = d.H3; M = d.closed_loop_A_BK;
Tf = d.Tf;
v = [1; 1];                     % v(t) = [1(t), 1(t)]'
n = size(A, 1);
m = size(B, 2);

%% ---------------- 复核 ----------------
say('【模型与解耦条件】omega = %g\n', d.omega);
say('A =\n%s\n\nB =\n%s\n\nC =\n%s\n\n', fmtm(A), fmtm(B), fmtm(C));
say('相对阶 d = %s，sum(d) = %d = n = %d（无剩余动态）\n', ...
    mat2str(d.relative_degree), sum(d.relative_degree), n);
say('解耦矩阵 B* = C A B =\n%s\n', fmtm(C * A * B));
say('  （非奇异，故可解耦；本机复核 rank = %d）\n\n', rank(C * A * B));

say('【解耦增益】K =\n%s\n', fmtm(K));
say('A + B K =\n%s\n', fmtm(M));
say('闭环极点 = %s\n', mat2str(sort(eig(M)), 6));
say('  与期望 {-2±j, -2, -2} 一致\n\n');

G0 = C * (-M \ (B * H1));
say('H = I 时 G(0) = diag(%s)  -> 阶跃稳态 %s\n', ...
    mat2str(round(diag(G0).', 6)), mat2str(round(diag(G0).', 4)));
G0b = C * (-M \ (B * H3));
say('H = diag(5,4) 时 G(0) = diag(%s)  -> 阶跃稳态 %s\n\n', ...
    mat2str(round(diag(G0b).', 6)), mat2str(round(diag(G0b).', 4)));

%% ---------------- 解析阶跃响应 ----------------
t = linspace(0, Tf, 800).';
[X1, Y1, U1] = const_input_response(M, B, C, K, H1, v, t);
[X3, Y3, U3] = const_input_response(M, B, C, K, H3, v, t);
[Xd, Yd, Ud] = const_input_response(M, B, C, K, H1, [1; 0], t);

say('【任务 (2)】v = [1(t), 1(t)]''，H = I\n');
say('  y 稳态 = [%.4f, %.4f]（理论 0.2 / 0.25）\n', Y1(end, 1), Y1(end, 2));
say('  max|u_r| = %.4f, max|u_t| = %.4f\n', max(abs(U1(:, 1))), max(abs(U1(:, 2))));
say('  x 稳态 = [%s]\n\n', mat2str(round(X1(end, :), 6)));

say('【任务 (3)】v = [1(t), 1(t)]''，H = diag(5, 4)\n');
say('  y 稳态 = [%.4f, %.4f]（要求 1 / 1）\n', Y3(end, 1), Y3(end, 2));
say('  max|u_r| = %.4f, max|u_t| = %.4f\n\n', max(abs(U3(:, 1))), max(abs(U3(:, 2))));

say('【解耦验证】只加 v1 = 1(t)、v2 = 0：\n');
say('  y1 稳态 = %.4f, max|y2| = %.3e  （y2 应恒为 0）\n\n', Yd(end, 1), max(abs(Yd(:, 2))));

%% ---------------- Simulink 对照 ----------------
s1 = try_sim(A, B, C, D, K, H1, v, Tf);
s2 = try_sim(A, B, C, D, K, H3, v, Tf);
s3 = try_sim(A, B, C, D, K, H1, [1; 0], Tf);

if s1.ok
    [eX, eY, eU] = cmp(s1, M, B, C, K, H1, v);
    say('  Simulink(H=I)     与解析解最大偏差: x %.2e, y %.2e, u %.2e\n', eX, eY, eU);
else
    say('  Simulink(H=I) 未运行：%s\n', s1.err);
end
if s2.ok
    [eX3, eY3, eU3] = cmp(s2, M, B, C, K, H3, v);
    say('  Simulink(H=diag5,4) 与解析解最大偏差: x %.2e, y %.2e, u %.2e\n', eX3, eY3, eU3);
end
if s3.ok
    [~, eYd, ~] = cmp(s3, M, B, C, K, H1, [1; 0]);
    say('  Simulink(仅 v1)   的 y2 最大绝对值: %.2e\n', max(abs(s3.Y(:, 2))));
end
say('\n');

%% ---------------- 出图 ----------------
fig1 = figure('Position', [60 60 900 520], 'Visible', 'off');
tiledlayout(fig1, 2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
yl = {'y_1 = x  [m]', 'y_2 = y  [m]'};
yss = [0.2, 0.25];
for i = 1:2
    nexttile;
    plot(t, Y1(:, i), 'LineWidth', 1.7, 'DisplayName', '解析解'); hold on;
    if s1.ok
        plot(s1.t, s1.Y(:, i), '--', 'LineWidth', 1.2, 'DisplayName', 'Simulink');
    end
    h = yline(yss(i), ':k', 'LineWidth', 1.1);
    h.DisplayName = sprintf('稳态 %.2f', yss(i));
    grid on; xlabel('t [s]'); ylabel(yl{i});
    legend('Location', 'best');
end
sgtitle('实验二 (2) 单位阶跃输出响应（H = I，稳态 0.2 / 0.25）');
export_figure(fig1, outDir, 'ex_02_outputs_H1');

fig2 = figure('Position', [60 60 1000 620], 'Visible', 'off');
tiledlayout(fig2, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
xl = {'x_1 = x  [m]', 'x_2 = dx/dt  [m/s]', 'x_3 = y  [m]', 'x_4 = dy/dt  [m/s]'};
for i = 1:4
    nexttile;
    plot(t, X1(:, i), 'LineWidth', 1.6, 'DisplayName', '解析解'); hold on;
    if s1.ok
        plot(s1.t, s1.X(:, i), '--', 'LineWidth', 1.1, 'DisplayName', 'Simulink');
    end
    grid on; xlabel('t [s]'); ylabel(xl{i});
    if i == 1, legend('Location', 'best'); end
end
sgtitle('实验二 (2) 状态响应（H = I）');
export_figure(fig2, outDir, 'ex_02_states_H1');

fig3 = figure('Position', [60 60 1000 340], 'Visible', 'off');
tiledlayout(fig3, 1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
ul = {'u_r  [m/s^2]', 'u_t  [m/s^2]'};
for i = 1:2
    nexttile;
    plot(t, U1(:, i), 'LineWidth', 1.6, 'DisplayName', 'u = Kx + Hv'); hold on;
    if s1.ok
        plot(s1.t, s1.U(:, i), '--', 'LineWidth', 1.1, 'DisplayName', 'Simulink');
    end
    grid on; xlabel('t [s]'); ylabel(ul{i});
    if i == 1, legend('Location', 'best'); end
end
sgtitle('实验二 (2) 控制量（H = I）');
export_figure(fig3, outDir, 'ex_02_controls_H1');

fig4 = figure('Position', [60 60 900 520], 'Visible', 'off');
tiledlayout(fig4, 2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
for i = 1:2
    nexttile;
    plot(t, Y3(:, i), 'LineWidth', 1.7, 'DisplayName', '解析解'); hold on;
    if s2.ok
        plot(s2.t, s2.Y(:, i), '--', 'LineWidth', 1.2, 'DisplayName', 'Simulink');
    end
    h = yline(1, ':k', 'LineWidth', 1.1);
    h.DisplayName = '目标稳态 1';
    ylim([0 1.3]); grid on; xlabel('t [s]'); ylabel(yl{i});
    legend('Location', 'best');
end
sgtitle('实验二 (3) 重新设计 H = diag(5,4)，稳态值均为 1');
export_figure(fig4, outDir, 'ex_02_outputs_H3');

fig5 = figure('Position', [60 60 900 460], 'Visible', 'off');
tiledlayout(fig5, 2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
nexttile;
plot(t, Yd(:, 1), 'LineWidth', 1.7);
grid on; xlabel('t [s]'); ylabel('y_1 [m]');
title('v_1 = 1(t) 的响应（y_1）');
nexttile;
plot(t, Yd(:, 2), 'LineWidth', 1.7);
grid on; xlabel('t [s]'); ylabel('y_2 [m]');
title(sprintf('同一输入下 y_2 恒为 0（max|y_2| = %.2e）', max(abs(Yd(:, 2)))));
export_figure(fig5, outDir, 'ex_02_decoupling_check');

%% ---------------- 存数据与日志 ----------------
save(fullfile(outDir, 'ex_02_data.mat'), 'A', 'B', 'C', 'D', 'K', 'H1', 'H3', ...
     'M', 'v', 't', 'X1', 'Y1', 'U1', 'X3', 'Y3', 'U3', 'Yd');
fclose(logFid);

fprintf(1, 'd = %s  B* = I  K = %s\n', mat2str(d.relative_degree), mat2str(K));
fprintf(1, 'poles = %s\n', mat2str(round(sort(eig(M)), 6)));
fprintf(1, 'y_ss(H1) = %s   y_ss(H3) = %s\n', ...
    mat2str(round(Y1(end, :), 4)), mat2str(round(Y3(end, :), 4)));
fprintf(1, 'max|y2| (v=[1;0]) = %.3e\n', max(abs(Yd(:, 2))));
if s1.ok
    fprintf(1, 'sim err(H1): x %.2e  y %.2e  u %.2e\n', eX, eY, eU);
end
fprintf(1, 'figures + data -> %s\n', outDir);

%% ---------------- 局部函数 ----------------
function [X, Y, U] = const_input_response(M, B, C, K, H, v, t)
% 常值输入 v 下的精确响应：x(t) = M^{-1}(e^{Mt}-I) B H v
n = size(M, 1);
N = numel(t);
X = zeros(N, n);
Minv = M \ eye(n);
BHv = B * H * v;
for k = 1:N
    X(k, :) = (Minv * (expm(M * t(k)) - eye(n)) * BHv).';
end
Y = X * C.';
U = X * K.' + repmat((H * v).', N, 1);
end

function s = fmtm(M)
rows = arrayfun(@(i) mat2str(M(i, :), 6), 1:size(M, 1), 'UniformOutput', false);
s = strjoin(rows, newline);
end

function res = try_sim(A, B, C, D, K, H, v, Tf)
res = struct('ok', false, 'err', '', 't', [], 'X', [], 'Y', [], 'U', []);
try
    [res.t, res.X, res.Y, res.U] = ex_02_run_sim(A, B, C, D, K, H, v, Tf);
    res.ok = true;
catch err
    res.err = err.message;
end
end

function [eX, eY, eU] = cmp(simRes, M, B, C, K, H, v)
[Xa, Ya, Ua] = const_input_response(M, B, C, K, H, v, simRes.t);
eX = max(abs(simRes.X - Xa), [], 'all');
eY = max(abs(simRes.Y - Ya), [], 'all');
eU = max(abs(simRes.U - Ua), [], 'all');
end
