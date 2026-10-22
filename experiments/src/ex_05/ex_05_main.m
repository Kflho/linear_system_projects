%% ===================== 实验五：观测器设计（三组，纯数值仿真） =====================
%
%   对象：磁盘驱动器磁头位置伺服
%       x' = [[0,1],[0,-1]] x + [0;1] u,   y = [1 0] x
%   状态反馈：u = K x̂，K = [-5 -3]（极点 -2±j，与任务书一致）
%
%   (一) 全维观测器：x̂' = F x̂ + G y + H u,  F = A + L C, G = -L, H = B
%        L = [-9; -20]（观测器极点 -5±2j），初值 x̂(0) = 0
%   (二) 降维观测器：z' = F z + G y + H u,  x̂ = M z + N y
%        F = -5, G = -20, H = 1, M = [0;1], N = [1;4]（误差极点 -5）
%   (三) 函数观测器：只重构 f = Kx，z' = F z + G y + H u, u = M_f z + N_f y
%        (F,G) = (-5,1) -> T = [0.2,-0.05], H = -0.05, u = 60 z - 17 y
%
%   本脚本不依赖 Simulink：闭环矩阵已在 ex_05_design.py 里拼好（design.A_cl），
%   直接 x(t) = expm(A_cl t) z0 算解析解，再从 z(t) 里取 x、z、x̂、u、e。
%   想用 Simulink 也行：JSON 里的键名就是模型变量名（A/B/C/A_obs/B_obs/...），
%   见 bin/references/ex_05.md。
%
%   运行：
%       matlab -batch "cd('experiments'); run('src/ex_05/ex_05_main.m')"

clear; close all; clc;

thisDir = fileparts(mfilename('fullpath'));
srcDir  = fileparts(thisDir);
expRoot = fileparts(srcDir);
addpath(fullfile(srcDir, 'common'));

outDir = fullfile(expRoot, 'bin', 'data', 'ex_05');
if ~exist(outDir, 'dir'), mkdir(outDir); end
logFid = fopen(fullfile(outDir, 'ex_05_log.txt'), 'w', 'n', 'UTF-8');
say = @(varargin) fprintf(logFid, varargin{:});

json1 = fullfile(thisDir, 'ex_05_1_design.json');
json2 = fullfile(thisDir, 'ex_05_2_design.json');
json3 = fullfile(thisDir, 'ex_05_3_design.json');
if ~exist(json1, 'file') || ~exist(json2, 'file') || ~exist(json3, 'file')
    error(['找不到设计 JSON，请先运行：' newline ...
           'python experiments\\src\\ex_05\\ex_05_design.py']);
end
d1 = jsondecode(fileread(json1));
d2 = jsondecode(fileread(json2));
d3 = jsondecode(fileread(json3));

A = d1.A;  B = d1.B;  c = d1.c(:).';  K = d1.K;  x0 = d1.x0(:);
Tf = d1.design.Tf;
N  = 3001;

%% ==================== 复核设计量 ====================
say('================ 实验五 观测器设计（磁盘驱动器磁头）================\n');
say('【对象】x'' = A x + B u,  y = c x = [1 0] x\n');
say('  A = %s,  B = %s\n', mat2str(A, 6), mat2str(B, 6));
say('  开环极点 = %s（积分器 + 一阶惯性，磁头位置自身会漂）\n\n', ...
    mat2str(sort(eig(A)), 6));

say('【状态反馈】u = K x̂ 中的 K（先用真状态设计，再换成估计值）\n');
say('  K = %s\n', mat2str(K, 6));
say('  eig(A + B K) = %s\n\n', mat2str(sort(eig(A + B * K)), 6));

%% ---------------- 三组观测器的极点复核 ----------------
say('【(一) 全维观测器】F = A + L C, G = -L, H = B\n');
say('  L = %s\n', mat2str(d1.design.L, 6));
say('  F = %s\n', mat2str(d1.design.F, 6));
say('  eig(F) = %s\n', mat2str(sort(eig(d1.design.F)), 6));
say('  eig(A_cl) = %s\n', mat2str(sort(eig(d1.design.A_cl)), 6));
say('  （分离原理：闭环极点 = 状态反馈极点 ∪ 观测器极点）\n\n');

say('【(二) 降维观测器】z'' = F z + G y + H u,  x̂ = M z + N y\n');
say('  L21 = %g,  F = %g,  G = %g,  H = %g\n', d2.design.L21, d2.F, d2.G, d2.H);
say('  M = %s,  N = %s\n', mat2str(d2.M(:).', 4), mat2str(d2.N(:).', 4));
say('  eig(F) = %s,  eig(A_cl) = %s\n\n', ...
    mat2str(sort(d2.design.poles_obs.real + 1i * d2.design.poles_obs.imag).'), ...
    mat2str(sort(eig(d2.design.A_cl)), 6));

say('【(三) 函数观测器】只重构 f = K x\n');
say('  (F, G) = (%g, %g),  T = %s,  H = T B = %g\n', d3.F, d3.G, ...
    mat2str(d3.design.T, 4), d3.H);
say('  控制律直接写：u = M_f z + N_f y = %g z %+g y\n', ...
    d3.design.M_f, d3.design.N_f);
say('  模型里画估计状态用的 M = %s, N = %s（满足 M T + N C = I）\n', ...
    mat2str(d3.M(:).', 4), mat2str(d3.N(:).', 4));
say('  eig(A_cl) = %s\n\n', mat2str(sort(eig(d3.design.A_cl)), 6));

%% ==================== (一) 全维观测器仿真 ====================
Acl1 = d1.design.A_cl;
z01  = [x0; d1.xh0(:)];
[t, Z1] = free_response(Acl1, z01, Tf, N);

X1  = Z1(:, 1:2);                    % 真实状态
Xh1 = Z1(:, 3:4);                    % 观测器状态
Y1  = X1 * c.';
U1  = Xh1 * K.';                     % u = K x̂
E1  = X1 - Xh1;

m1 = metrics(E1, U1, t);
say('【(一) 仿真】Tf = %g s，采样 %d 点\n', Tf, N);
say('  x̂(0) = 0，估计误差 e(0) = x(0) = %s\n', mat2str(x0.', 4));
say('  max|u| = %.3f，max|e| = %.3f，e 衰减到 2%% 用时 %.2f s，末段 max|e| = %.2e\n\n', ...
    m1.umax, m1.emax, m1.tset, m1.etail);

%% ==================== (二) 降维观测器仿真 ====================
Acl2 = d2.design.A_cl;
z02  = [x0; d2.z0];
[t, Z2] = free_response(Acl2, z02, Tf, N);

X2  = Z2(:, 1:2);
Z2b = Z2(:, 3);                      % 降维观测器内部状态 z
Y2  = X2 * c.';
Xh2 = Z2b * d2.M.' + Y2 * d2.N.';    % x̂ = M z + N y
U2  = Xh2 * K.';
E2  = X2 - Xh2;

m2 = metrics(E2, U2, t);
say('【(二) 仿真】z(0) = %g -> x̂(0) = [%g; %g]（x̂1 = y 直接量到，只有速度有初差）\n', ...
    d2.z0, Xh2(1, 1), Xh2(1, 2));
say('  max|u| = %.3f，max|e| = %.3f，e 衰减到 2%% 用时 %.2f s，末段 max|e| = %.2e\n\n', ...
    m2.umax, m2.emax, m2.tset, m2.etail);

%% ==================== (三) 函数观测器仿真 ====================
Acl3 = d3.design.A_cl;
% 模型里用 x̂ = M z + N y 再乘 K；坐标 z 与(二)只差一个标度
z03  = [x0; d3.z0];
[t, Z3] = free_response(Acl3, z03, Tf, N);

X3  = Z3(:, 1:2);
Z3b = Z3(:, 3);
Y3  = X3 * c.';
Xh3 = Z3b * d3.M.' + Y3 * d3.N.';
U3  = Xh3 * K.';
U3d = Z3b * d3.design.M_f + Y3 * d3.design.N_f;    % 直接 u = M_f z + N_f y
E3  = X3 - Xh3;

m3 = metrics(E3, U3, t);
say('【(三) 仿真】u = M_f z + N_f y = %g z %+g y\n', d3.design.M_f, d3.design.N_f);
say('  max|u| = %.3f，max|e| = %.3f，e 衰减到 2%% 用时 %.2f s，末段 max|e| = %.2e\n', ...
    m3.umax, m3.emax, m3.tset, m3.etail);
say('  模型里 x̂→K 与直接 M_f z + N_f y 的差 = %.2e（数值一致）\n', ...
    max(abs(U3 - U3d)));
say('  函数观测器内部状态 z3 与降维观测器 z2 的关系：max|z3 + z2/20| = %.2e\n', ...
    max(abs(Z3b + Z2b / 20)));
say('  -> 同一条闭环（误差极点都是 -5），只是 z 的坐标标度不同，u(t) 完全相同\n\n');
say('  max|u_(三) - u_(二)| = %.2e\n\n', max(abs(U3 - U2)));

%% ==================== 三种观测器对比 ====================
say('【对比】\n');
say('  %-18s %10s %10s %10s %10s\n', '观测器', 'max|u|', 'max|e|', 'e 2% 时间', 'u(0)');
say('  %-18s %10.3f %10.3f %10.2f %10.3f\n', '(一) 全维', m1.umax, m1.emax, m1.tset, U1(1));
say('  %-18s %10.3f %10.3f %10.2f %10.3f\n', '(二) 降维', m2.umax, m2.emax, m2.tset, U2(1));
say('  %-18s %10.3f %10.3f %10.2f %10.3f\n', '(三) 函数', m3.umax, m3.emax, m3.tset, U3(1));
say('  （初值不同：全维 x̂(0)=0，降维/函数 x̂1(0)=y(0)，所以 u(0) 有差别）\n\n');

%% ==================== 出图 ====================
lbl = {'x_1 = 位置', 'x_2 = 速度'};

% ---- 图 1：全维观测器 ----
fig1 = figure('Position', [60 60 1000 620], 'Visible', 'off');
tiledlayout(fig1, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
for i = 1:2
    nexttile;
    plot(t, X1(:, i), 'LineWidth', 1.7); hold on;
    plot(t, Xh1(:, i), '--', 'LineWidth', 1.5);
    grid on; xlabel('t [s]'); ylabel(lbl{i});
    if i == 1
        legend({'x(t)', 'x̂(t)'}, 'Location', 'northeast');
        title('全维观测器：状态 vs 估计');
    end
end
nexttile;
plot(t, E1(:, 1), 'LineWidth', 1.5); hold on;
plot(t, E1(:, 2), 'LineWidth', 1.5); grid on;
xlabel('t [s]'); ylabel('e = x - x̂');
legend({'e_1', 'e_2'}, 'Location', 'best');
title('估计误差（极点 -5±2j）');
nexttile;
semilogy(t, sqrt(sum(E1.^2, 2)), 'LineWidth', 1.5); hold on;
yline(0.02 * sqrt(sum(E1(1, :).^2)), ':');
grid on; xlabel('t [s]'); ylabel('||e(t)||');
title(sprintf('误差范数（2%% 线，%.2f s 收敛）', m1.tset));
export_figure(fig1, outDir, 'ex_05_1_full_order');

% ---- 图 2：全维观测器的控制量 ----
fig2 = figure('Position', [60 60 1000 340], 'Visible', 'off');
tiledlayout(fig2, 1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
nexttile;
plot(t, U1, 'LineWidth', 1.7); grid on;
xlabel('t [s]'); ylabel('u = K x̂');
title(sprintf('控制量（max = %.2f，u(0) = %.2f）', m1.umax, U1(1)));
nexttile;
plot(t, X1(:, 1), 'LineWidth', 1.7); hold on;
plot(t, U1, '--', 'LineWidth', 1.4); grid on;
xlabel('t [s]');
legend({'x_1(t)', 'u(t)'}, 'Location', 'best');
title('位置与控制量同图');
export_figure(fig2, outDir, 'ex_05_1_control');

% ---- 图 3：降维观测器 ----
fig3 = figure('Position', [60 60 1000 620], 'Visible', 'off');
tiledlayout(fig3, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
for i = 1:2
    nexttile;
    plot(t, X2(:, i), 'LineWidth', 1.7); hold on;
    plot(t, Xh2(:, i), '--', 'LineWidth', 1.5);
    grid on; xlabel('t [s]'); ylabel(lbl{i});
    if i == 1
        legend({'x(t)', 'x̂(t)'}, 'Location', 'northeast');
        title('降维观测器：状态 vs 估计');
    end
end
nexttile;
plot(t, Z2b, 'LineWidth', 1.7); grid on;
xlabel('t [s]'); ylabel('z(t)');
title('观测器内部状态（F = -5）');
nexttile;
semilogy(t, abs(E2(:, 2)), 'LineWidth', 1.5); hold on;
yline(0.02 * abs(E2(1, 2)), ':');
grid on; xlabel('t [s]'); ylabel('|e_2|');
title('速度估计误差（e_2(0) = -4，指数收敛）');
export_figure(fig3, outDir, 'ex_05_2_reduced_order');

% ---- 图 4：函数观测器 ----
fig4 = figure('Position', [60 60 1000 620], 'Visible', 'off');
tiledlayout(fig4, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
for i = 1:2
    nexttile;
    plot(t, X3(:, i), 'LineWidth', 1.7); hold on;
    plot(t, Xh3(:, i), '--', 'LineWidth', 1.5);
    grid on; xlabel('t [s]'); ylabel(lbl{i});
    if i == 1
        legend({'x(t)', 'x̂(t)'}, 'Location', 'northeast');
        title('函数观测器：状态 vs 估计');
    end
end
nexttile;
plot(t, Z3b, 'LineWidth', 1.7); grid on;
xlabel('t [s]'); ylabel('z(t)');
title('函数观测器内部状态（F = -5，G = 1）');
nexttile;
plot(t, Z2b, 'LineWidth', 1.7); hold on;
plot(t, -20 * Z3b, '--', 'LineWidth', 1.4); grid on;
xlabel('t [s]');
legend({'z_{(二)}', '-20 z_{(三)}'}, 'Location', 'best');
title('两种观测器的 z 只差一个标度');
export_figure(fig4, outDir, 'ex_05_3_function');

% ---- 图 5：三种观测器对比 ----
fig5 = figure('Position', [60 60 1000 620], 'Visible', 'off');
tiledlayout(fig5, 2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
nexttile;
plot(t, U1, 'LineWidth', 1.7); hold on;
plot(t, U2, '--', 'LineWidth', 1.5);
plot(t, U3, ':', 'LineWidth', 2.0);
grid on; xlabel('t [s]'); ylabel('u(t)');
legend({'(一) 全维', '(二) 降维', '(三) 函数'}, 'Location', 'best');
title('三种观测器下的控制量（(二)(三) 完全重合）');
nexttile;
semilogy(t, sqrt(sum(E1.^2, 2)), 'LineWidth', 1.7); hold on;
semilogy(t, sqrt(sum(E2.^2, 2)), '--', 'LineWidth', 1.5);
semilogy(t, sqrt(sum(E3.^2, 2)), ':', 'LineWidth', 2.0);
grid on; xlabel('t [s]'); ylabel('||e(t)||');
legend({'(一) 全维', '(二) 降维', '(三) 函数'}, 'Location', 'best');
title('估计误差范数（全维从 0 初值起步，误差最大）');
export_figure(fig5, outDir, 'ex_05_compare');

% ---- 图 6：极点分布 ----
fig6 = figure('Position', [60 60 560 520], 'Visible', 'off');
pl = @(v) [v.real] + 1i * [v.imag];
plot(real(eig(A)), imag(eig(A)), 's', 'MarkerSize', 9, 'LineWidth', 1.6); hold on;
plot(real(eig(A + B * K)), imag(eig(A + B * K)), 'o', 'MarkerSize', 9, 'LineWidth', 1.6);
plot(real(pl(d1.design.poles_obs)), imag(pl(d1.design.poles_obs)), 'x', ...
     'MarkerSize', 11, 'LineWidth', 1.8);
plot(real(pl(d2.design.poles_obs)), imag(pl(d2.design.poles_obs)), '+', ...
     'MarkerSize', 12, 'LineWidth', 1.8);
grid on; xline(0, 'k:'); yline(0, 'k:');
xlabel('Re'); ylabel('Im');
legend({'开环极点 {0,-1}', '状态反馈 {-2±j}', '全维观测器 {-5±2j}', ...
        '降维/函数观测器 {-5}'}, 'Location', 'southwest');
title('实验五 极点分布');
export_figure(fig6, outDir, 'ex_05_poles');

%% ==================== 存数据 ====================
save(fullfile(outDir, 'ex_05_data.mat'), ...
     'A', 'B', 'c', 'K', 'x0', 'Tf', 't', ...
     'X1', 'Xh1', 'U1', 'E1', 'X2', 'Xh2', 'U2', 'E2', 'Z2b', ...
     'X3', 'Xh3', 'U3', 'E3', 'Z3b', 'U3d', 'm1', 'm2', 'm3');
fclose(logFid);

fprintf(1, 'K = %s   eig(A+BK) = %s\n', mat2str(round(K, 6)), ...
        mat2str(round(sort(eig(A + B * K)), 6)));
fprintf(1, '(一) max|u| = %.3f, e 2%% 时间 %.2f s\n', m1.umax, m1.tset);
fprintf(1, '(二) max|u| = %.3f, e 2%% 时间 %.2f s\n', m2.umax, m2.tset);
fprintf(1, '(三) max|u| = %.3f, e 2%% 时间 %.2f s\n', m3.umax, m3.tset);
fprintf(1, 'figures + data -> %s\n', outDir);

%% ==================== 局部函数 ====================
function [t, Z] = free_response(A_cl, z0, Tf, N)
%FREE_RESPONSE  z(t) = expm(A_cl t) z0（齐次解，闭环里没有外部输入）
t = linspace(0, Tf, N).';
Z = zeros(N, numel(z0));
for k = 1:N
    Z(k, :) = (expm(A_cl * t(k)) * z0(:)).';
end
end

function m = metrics(E, U, t)
%METRICS  估计误差/控制量的几个指标
en = sqrt(sum(E.^2, 2));
m.emax = max(abs(E), [], 'all');
m.umax = max(abs(U), [], 'all');
k = find(en > 0.02 * en(1), 1, 'last');
if isempty(k)
    m.tset = 0;
elseif k < numel(t)
    m.tset = t(k + 1);
else
    m.tset = NaN;
end
m.etail = max(en(t > t(end) - 0.5));
end
