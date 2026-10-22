%% ===================== 实验六：观测器与状态反馈（内模渐近跟踪） =====================
%
%   任务书 (四)：磁头要跟踪 r(t) = sin(omega0 t)，omega0 = 1（磁盘自转角速度）
%
%   对象：  x' = [[0,1],[0,-1]] x + [0;1] u,   y = [1 0] x
%   降维观测器：z' = F z + G y + H u,  x̂ = M z + N y
%               F = -5, G = -20, H = 1, M = [0;1], N = [1;4]（误差极点 -5）
%   内模：   eta' = A_c eta + B_c e,  e = y - r,  A_c = [[0,-1],[1,0]], B_c = [1;0]
%            （即 eta1' = e - eta2, eta2' = eta1，特征多项式 s^2 + 1）
%   控制：   u = K_x x̂ + K_eta eta，K = [-14, -5, -12, 4]
%            （对 [x; eta] 增广系统极点配置到 {-2±j, -1±j}）
%   参考：   w' = A_r w, r = C_r w, w(0) = [0;1] -> r = sin t（自治，不用外部信号源）
%
%   闭环状态 z = [x(2); z_obs(1); eta(2); w(2)] 共 7 维，A_cl 已在 design JSON 里拼好，
%   直接 z(t) = expm(A_cl t) z0 得解析解；不依赖 Simulink。
%   （JSON 里同时给了 Simulink 变量名 A/B/c/A_obs/...，想搭模型见
%    bin/references/ex_06.md）
%
%   运行：
%       matlab -batch "cd('experiments'); run('src/ex_06/ex_06_main.m')"

clear; close all; clc;

thisDir = fileparts(mfilename('fullpath'));
srcDir  = fileparts(thisDir);
expRoot = fileparts(srcDir);
addpath(fullfile(srcDir, 'common'));

outDir = fullfile(expRoot, 'bin', 'data', 'ex_06');
if ~exist(outDir, 'dir'), mkdir(outDir); end
logFid = fopen(fullfile(outDir, 'ex_06_log.txt'), 'w', 'n', 'UTF-8');
say = @(varargin) fprintf(logFid, varargin{:});

jsonFile = fullfile(thisDir, 'ex_06_1_design.json');
if ~exist(jsonFile, 'file')
    error(['找不到设计 JSON，请先运行：' newline ...
           'python experiments\\src\\ex_06\\ex_06_design.py']);
end
d = jsondecode(fileread(jsonFile));

A = d.A;  B = d.B;  c = d.c(:).';  x0 = d.x0(:);
F = d.A_obs;  G = d.B_obs(1);  H = d.B_obs(2);
M = d.M;  N = d.N;  z0 = d.z0;
Ac = d.Ac;  Bc = d.Bc;  eta0 = d.eta0(:);
A_r = d.A_r;  C_r = d.C_r;  w0 = d.w0(:);
KX = d.KX;  K_eta = d.kyita;
K_aug = [KX, K_eta];
Tf = d.design.Tf;
w0f = d.design.omega0;
Npt = 3001;

%% ==================== 复核设计量 ====================
say('================ 实验六 观测器 + 状态反馈：内模渐近跟踪 ================\n');
say('【对象】x'' = A x + B u,  y = c x = [1 0] x\n');
say('  A = %s,  B = %s\n\n', mat2str(A, 6), mat2str(B, 6));

say('【参考】r'' = -%g^2 r  ->  r(t) = sin(%g t)\n', w0f, w0f);
say('  外系统 A_r = %s,  C_r = %s,  w(0) = %s\n\n', ...
    mat2str(A_r, 6), mat2str(C_r, 6), mat2str(w0.', 4));

say('【内模】eta'' = A_c eta + B_c e,  e = y - r\n');
say('  A_c = %s,  B_c = %s,  eta(0) = %s\n', ...
    mat2str(Ac, 6), mat2str(Bc, 6), mat2str(eta0.', 4));
say('  即 eta1'' = e - eta2,  eta2'' = eta1     （特征多项式 s^2 + %g）\n\n', w0f^2);

say('【增广系统极点配置】[x; eta] 共 %d 阶\n', d.design.n_aug);
say('  能控 rank = %d / %d\n', d.design.ctrb_rank_aug, d.design.n_aug);
say('  K_aug = [K_x, K_eta] = %s\n', mat2str(K_aug, 6));
say('     K_x = %s（乘 x̂）\n', mat2str(KX, 6));
say('     K_eta = [kyita1, kyita2] = %s（kyita1 乘 eta1、kyita2 乘 eta2）\n', ...
    mat2str(K_eta, 6));
say('  eig(A_aug + B_aug K_aug) = %s\n', mat2str(sort(eig(d.design.A_aug + ...
    d.design.B_aug * K_aug)), 6));
say('  （状态反馈那对 -2±j + 内模那对 -1±j）\n\n');

say('【降维观测器】F = %g, G = %g, H = %g, M = %s, N = %s\n', ...
    F, G, H, mat2str(M.', 4), mat2str(N.', 4));
say('  误差极点 = %s\n\n', mat2str(sort(eig(F)), 6));

%% ==================== 闭环极点（含参考外系统） ====================
A_cl = d.design.A_cl;
z0_big = d.design.z0_big(:);
poles = eig(A_cl);
say('【闭环】状态 [x; z_obs; eta; w] 共 %d 维\n', numel(z0_big));
say('  eig(A_cl) = %s\n', mat2str(sort(poles), 6));
say('  分离原理 + 内模 + 外系统：{-2±j} ∪ {-1±j} ∪ {-5} ∪ {±j}\n\n');

%% ==================== 解析解 ====================
t = linspace(0, Tf, Npt).';
Z = zeros(Npt, numel(z0_big));
for k = 1:Npt
    Z(k, :) = (expm(A_cl * t(k)) * z0_big).';
end

X    = Z(:, 1:2);                 % 真实状态
Zobs = Z(:, 3);                   % 降维观测器内部状态
Eta  = Z(:, 4:5);                 % 内模状态
W    = Z(:, 6:7);                 % 参考外系统
R    = W * C_r.';                 % r = sin t
Y    = X * c.';
Xh   = Zobs * M.' + Y * N.';      % x̂ = M z + N y
U    = Xh * KX.' + Eta * K_eta.'; % u = K_x x̂ + K_eta eta
E    = Y - R;

tail = t > Tf - 5;
say('【仿真】Tf = %g s（约 %.1f 个参考周期），采样 %d 点\n', ...
    Tf, Tf / (2 * pi / w0f), Npt);
say('  x(0) = %s,  x̂(0) = [%g; %g]（z(0) = %g）\n', mat2str(x0.', 4), ...
    Xh(1, 1), Xh(1, 2), z0);
say('  u(0) = %.2f，max|u| = %.2f\n', U(1), max(abs(U)));
say('  max|e| 全程 = %.3e，末 5 s = %.3e（内模原理：e -> 0）\n', ...
    max(abs(E)), max(abs(E(tail))));
say('  max|y - r| 末 5 s = %.3e，max|y| = %.3f，max|r| = %.3f\n', ...
    max(abs(Y(tail) - R(tail))), max(abs(Y)), max(abs(R)));
say('  末段 x̂ 与 x 的最大偏差 = %.3e（观测器误差 e^{-5t} 早已衰减）\n', ...
    max(abs(Xh(tail, :) - X(tail, :)), [], 'all'));
say(['  内模状态末段幅值：max|eta1| = %.3f，max|eta2| = %.3f' ...
     '（自己"长"出 %g rad/s 的振荡）\n\n'], ...
    max(abs(Eta(tail, 1))), max(abs(Eta(tail, 2))), w0f);

% 稳态：y 与 r 的相位/幅值
yy = Y(tail);  rr = R(tail);
amp_ratio = max(abs(yy)) / max(abs(rr));
say('  末段幅值比 max|y|/max|r| = %.4f（1 表示幅值无差）\n', amp_ratio);

%% ==================== 传递函数与 Bode ====================
% 内模控制器 C_im(s) = K_eta (sI - A_c)^{-1} B_c = (kyita1 s + kyita2)/(s^2 + w0^2)
num_im = d.design.inter_model_num;
den_im = d.design.inter_model_den;

% 从 r 到 e / y 的闭环
A_ctrl = d.design.A_ctrl;  B_ctrl = d.design.B_ctrl;
C_e = d.design.C_e;  D_e = d.design.D_e;
C_y = [c, zeros(1, size(A_ctrl, 1) - numel(c))];

w = logspace(-1, 1.2, 800);
w = w(abs(w - w0f) > 1e-3);                 % 避开恰好落在极点上的点（会 Inf）
H_im = polyval(num_im, 1i * w) ./ polyval(den_im, 1i * w);
H_er = freqresp_ss(A_ctrl, B_ctrl, C_e, D_e, w);
H_yr = freqresp_ss(A_ctrl, B_ctrl, C_y, 0, w);

e_at_w0 = abs(d.design.e_over_r_at_omega0);
say('【频域】内模控制器 C_im(s) = (%g s %+g)/(s^2 %+g)\n', num_im(1), num_im(2), w0f^2);
say('  |C_im(j%g)| -> 无穷（分母为零、分子不为零）\n', w0f);
say('  闭环 |e/r(j%g)| = %.2e（应为 0，即该频率上完全跟踪）\n', w0f, e_at_w0);
say('  闭环 |y/r(j%g)| = %.4f（应为 1）\n\n', w0f, ...
    abs(freqresp_ss(A_ctrl, B_ctrl, C_y, 0, w0f)));

%% ==================== 出图 ====================
% ---- 图 1：跟踪 ----
fig1 = figure('Position', [60 60 1000 560], 'Visible', 'off');
tiledlayout(fig1, 2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
nexttile;
plot(t, R, 'LineWidth', 1.5); hold on;
plot(t, Y, '--', 'LineWidth', 1.9);
grid on; ylabel('y');
legend({'r(t) = sin t', 'y(t) = x_1(t)'}, 'Location', 'best');
title('参考跟踪（降维观测器 + 内模）');
nexttile;
plot(t, E, 'LineWidth', 1.6); hold on; yline(0, ':');
grid on; xlabel('t [s]'); ylabel('e = y - r');
title(sprintf('跟踪误差（末 5 s max|e| = %.1e）', max(abs(E(tail)))));
export_figure(fig1, outDir, 'ex_06_tracking');

% ---- 图 2：状态与估计 ----
fig2 = figure('Position', [60 60 1000 620], 'Visible', 'off');
tiledlayout(fig2, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
nexttile;
plot(t, X(:, 1), 'LineWidth', 1.7); hold on;
plot(t, Xh(:, 1), '--', 'LineWidth', 1.5);
grid on; xlabel('t [s]'); ylabel('x_1 = 位置');
legend({'x_1(t)', 'x̂_1(t)'}, 'Location', 'best');
title('位置（x̂_1 = y，直接量到）');
nexttile;
plot(t, X(:, 2), 'LineWidth', 1.7); hold on;
plot(t, Xh(:, 2), '--', 'LineWidth', 1.5);
grid on; xlabel('t [s]'); ylabel('x_2 = 速度');
legend({'x_2(t)', 'x̂_2(t)'}, 'Location', 'best');
title('速度（由降维观测器估计）');
nexttile;
plot(t, U, 'LineWidth', 1.7); grid on;
xlabel('t [s]'); ylabel('u');
title(sprintf('控制量（u(0) = %.1f，max|u| = %.1f）', U(1), max(abs(U))));
nexttile;
plot(t, X(:, 2) - Xh(:, 2), 'LineWidth', 1.6);
grid on; xlabel('t [s]'); ylabel('x_2 - x̂_2');
title('速度估计误差（极点 -5）');
export_figure(fig2, outDir, 'ex_06_states');

% ---- 图 3：内模状态 ----
fig3 = figure('Position', [60 60 1000 460], 'Visible', 'off');
tiledlayout(fig3, 1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
nexttile;
plot(t, Eta(:, 1), 'LineWidth', 1.7); hold on;
plot(t, Eta(:, 2), '--', 'LineWidth', 1.5);
grid on; xlabel('t [s]'); ylabel('\eta');
legend({'\eta_1', '\eta_2'}, 'Location', 'best');
title('内模状态：把误差积分成 \omega_0 的振荡');
nexttile;
idx = t > Tf / 2;
plot(Y(idx), Eta(idx, 2), 'LineWidth', 1.3); grid on;
xlabel('y'); ylabel('\eta_2');
title('末段相图（Y–\eta_2 平面，收敛成椭圆）');
export_figure(fig3, outDir, 'ex_06_internal_model');

% ---- 图 4：Bode ----
fig4 = figure('Position', [60 60 1050 720], 'Visible', 'off');
tiledlayout(fig4, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
semilogx(w, 20 * log10(abs(H_im)), 'LineWidth', 1.7); hold on;
xline(w0f, 'r--', '\omega_0');
grid on; ylim([-40 60]); xlabel('\omega [rad/s]'); ylabel('|C_{im}| [dB]');
title('内模控制器幅频：\omega_0 处无穷大');

nexttile;
semilogx(w, unwrap(angle(H_im)) * 180 / pi, 'LineWidth', 1.7); hold on;
xline(w0f, 'r--', '\omega_0');
grid on; xlabel('\omega [rad/s]'); ylabel('\angle C_{im} [deg]');
title('内模控制器相频');

nexttile;
semilogx(w, 20 * log10(abs(H_er)), 'LineWidth', 1.7); hold on;
xline(w0f, 'r--', '\omega_0');
grid on; ylim([-80 20]); xlabel('\omega [rad/s]'); ylabel('|e/r| [dB]');
title('闭环 e/r：\omega_0 处为零点，误差被完全压掉');

nexttile;
semilogx(w, 20 * log10(abs(H_yr)), 'LineWidth', 1.7); hold on;
xline(w0f, 'r--', '\omega_0');
grid on; ylim([-40 10]); xlabel('\omega [rad/s]'); ylabel('|y/r| [dB]');
title('闭环 y/r：\omega_0 处 0 dB（幅值完全跟上）');

sgtitle('实验六 内模控制器与闭环频率特性');
export_figure(fig4, outDir, 'ex_06_bode');

% ---- 图 5：极点分布 ----
fig5 = figure('Position', [60 60 560 520], 'Visible', 'off');
pl = @(v) v(:);
plot(real(poles), imag(poles), 'o', 'MarkerSize', 9, 'LineWidth', 1.6); hold on;
grid on; xline(0, 'k:'); yline(0, 'k:');
xlabel('Re'); ylabel('Im');
title('实验六 闭环极点（含参考外系统）');
legend({'eig(A_{cl})'}, 'Location', 'southwest');
text(0.05, 1.05, '\pmj（参考外系统）', 'FontSize', 9);
export_figure(fig5, outDir, 'ex_06_poles');

%% ==================== 存数据 ====================
save(fullfile(outDir, 'ex_06_data.mat'), 'A', 'B', 'c', 'x0', 'K_aug', 'KX', 'K_eta', ...
     'F', 'G', 'H', 'M', 'N', 'Ac', 'Bc', 'A_r', 'C_r', 'A_cl', 'z0_big', ...
     't', 'X', 'Xh', 'Y', 'R', 'U', 'E', 'Eta', 'W', ...
     'w', 'H_im', 'H_er', 'H_yr', 'w0f');
fclose(logFid);

fprintf(1, 'K_aug = %s   eig(A_cl) = %s\n', mat2str(round(K_aug, 6)), ...
        mat2str(round(sort(eig(A_cl)), 5)));
fprintf(1, 'max|e| 末 5 s = %.2e, max|u| = %.2f\n', max(abs(E(tail))), max(abs(U)));
fprintf(1, '|e/r(j w0)| = %.2e, |y/r(j w0)| = %.4f\n', e_at_w0, ...
        abs(freqresp_ss(A_ctrl, B_ctrl, C_y, 0, w0f)));
fprintf(1, 'figures + data -> %s\n', outDir);

%% ==================== 局部函数 ====================
function H = freqresp_ss(A, B, C, D, w)
%FREQRESP_SS  H(jw) = C (jw I - A)^{-1} B + D（不依赖 Control System Toolbox）
%   SISO 时返回 1 x numel(w) 的行向量；w 为标量时返回标量。
w = reshape(w, 1, []);
H = zeros(1, numel(w));
for k = 1:numel(w)
    H(k) = C * ((1i * w(k) * eye(size(A)) - A) \ B) + D;
end
if numel(w) == 1
    H = H(1);
end
end
