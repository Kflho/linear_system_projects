%% ===================== 实验四：内模原理（两个任务） =====================
%
%   任务一（ex_04_1_sim.slx）：倒立摆输出调节，内模 1/(s^2+4)，跟踪 r = 0.1cos2t
%   任务二（ex_04_2_sim.slx）：渐近跟踪 + 干扰抑制，
%       对象 x' = [[0,1],[-2,2]]x + [0;1]u + E d，d = 1(t)，内模 1/(s(s^2+4))
%
%   设计在 ex_04_design.py 里（算法全在 Python 仓的 linear_system_utils）：
%       1) cal_internal_model 按模型实际的 Ac/Bc 拼增广系统
%       2) 对增广系统极点配置得 K_aug = [K_x, K_eta]
%       3) 内模原理：闭环稳定 ⇒ e -> 0，两个"零点"判据可离线验
%
%   本脚本：复核设计量 → 跑两个 Simulink 模型并与增广闭环解析解对照 → 出图 → 存数据
%
%   运行：
%       matlab -batch "cd('experiments'); run('src/ex_04/ex_04_main.m')"

clear; close all; clc;

thisDir = fileparts(mfilename('fullpath'));
srcDir  = fileparts(thisDir);
expRoot = fileparts(srcDir);
addpath(fullfile(srcDir, 'common'));
addpath(thisDir);

outDir = fullfile(expRoot, 'bin', 'data', 'ex_04');
if ~exist(outDir, 'dir'), mkdir(outDir); end
logFid = fopen(fullfile(outDir, 'ex_04_log.txt'), 'w', 'n', 'UTF-8');
say = @(varargin) fprintf(logFid, varargin{:});

json1 = fullfile(thisDir, 'ex_04_1_design.json');
json2 = fullfile(thisDir, 'ex_04_2_design.json');
if ~exist(json1, 'file') || ~exist(json2, 'file')
    error('找不到设计 JSON，请先运行：python experiments\\src\\ex_04\\ex_04_design.py');
end
d1 = jsondecode(fileread(json1));
d2 = jsondecode(fileread(json2));

Tf1 = 15;
Tf2 = 25;

%% ==================== 任务一：倒立摆 + 内模 1/(s^2+4) ====================
A1 = d1.A; B1 = d1.B; C1 = d1.C; D1 = d1.D; c1 = d1.c(:).';
Av = d1.A_v; Bv = d1.B_v; Cv = d1.C_v; Dv = d1.D_v;
w0 = d1.Initial_v(:); rg = d1.r(:).';
Kx1 = d1.k_x; ky2 = d1.kyita2; ky1 = d1.kyita1; wfb = d1.w;
K1 = [Kx1, ky2, ky1];                     % 增广状态顺序 [x(4); a; b]
Ac1 = d1.design.Ac; Bc1 = d1.design.Bc;

say('================ 任务一：倒立摆的输出调节（内模原理） ================\n');
say('【对象】x'' = A x + B u,  y = c x = [1 0 0 0] x\n');
say('【内模】a'' = e - %g b,  b'' = a     （1/(s^2+4)，e = y - r）\n', -wfb);
say('【控制】u = k_x x + kyita2 a + kyita1 b\n');
say('  K_aug = [k_x, kyita2, kyita1] = %s\n', mat2str(K1, 8));
say('  期望极点 = %s\n', mat2str(sort(complex([d1.design.poles.real], [d1.design.poles.imag])), 6));
say('  增广系统阶数 = %d，能控 rank = %d\n', d1.design.n_aug, d1.design.ctrb_rank);

% 独立复核：闭环极点 + 两个零点判据
Aaug1 = [A1, zeros(4, 2); Bc1 * c1, Ac1];
Baug1 = [B1; zeros(2, 1)];
Acl1 = Aaug1 + Baug1 * K1;
poles1 = sort(eig(Acl1));
say('  eig(A_aug + B_aug K) = %s\n', mat2str(poles1, 6));
Bref1 = [zeros(4, 1); -Bc1];              % r 以 -Bc 进内模
Caug1 = [c1, 0, 0];
zr1 = abs(Caug1 * ((1i * 2 * eye(6) - Acl1) \ Bref1) - 1);
say('  零点判据 |e/r(j2)| = %.2e（内模原理要求为 0）\n\n', zr1);

% ---- 仿真 ----
s1 = struct('ok', false, 'err', '');
try
    [t1, X1, U1, R1, Y1, E1] = ex_04_1_run_sim(d1, Tf1);
    s1.ok = true;
catch err
    s1.err = err.message;
    say('  Simulink 运行失败：%s\n\n', err.message);
end

if s1.ok
    say('【仿真】Tf = %g s，采样 %d 点，步长中位数 %.4g s\n', Tf1, numel(t1), median(diff(t1)));

    % 解析解：把外系统并进闭环
    Abig = [Acl1, Bref1 * rg; zeros(2, 6), Av];
    z0 = [zeros(6, 1); w0];
    n = numel(t1);
    Xa = zeros(n, 6); Wa = zeros(n, 2);
    for k = 1:n
        zz = expm(Abig * t1(k)) * z0;
        Xa(k, :) = zz(1:6).';
        Wa(k, :) = zz(7:8).';
    end
    Ua = (K1 * Xa.').';
    Ra = (rg * Wa.').';
    Ya = (c1 * Xa(:, 1:4).').';
    eX = max(abs(X1 - Xa(:, 1:4)), [], 'all');
    eU = max(abs(U1 - Ua), [], 'all');
    eY = max(abs(Y1 - Ya), [], 'all');
    say('  与解析解 expm([Ac Bref*r; 0 A_v] t) 的最大偏差：状态 %.2e，控制 %.2e，输出 %.2e\n', eX, eU, eY);

    tail = t1 > Tf1 - 3;
    say('  max|r| = %.4f，max|y| = %.4f，max|u| = %.4f\n', max(abs(R1)), max(abs(Y1)), max(abs(U1)));
    say('  max|y-r| 全程 = %.3e，末段 = %.3e\n', max(abs(Y1 - R1)), max(abs(Y1(tail) - R1(tail))));
    say('  末段 max|x1 - c*x| = %.2e（To Workspace 的 x_sim 与 c*x 是否一致）\n\n', ...
        max(abs(Y1(tail) - (X1(tail, :) * c1.'))));
end

%% ==================== 任务二：跟踪 + 干扰抑制 ====================
Av2 = d2.A_v; rg2 = d2.r(:).';
Ad = d2.A_d; Bd = d2.B_d; Cd = d2.C_d; Dd = d2.D_d; dstart = d2.d_start;
g1 = d2.gain1; g2c = d2.gain2; g6 = d2.gain6; g7 = d2.gain7;
K2 = [d2.k_x1, d2.k_x2, d2.kyita3, d2.kyita2, d2.kyita1];
Ac2 = d2.design.Ac; Bc2 = d2.design.Bc;
A2 = [0 1; -2 2];  B2 = [0; 1];  C2 = [1 0];   % 对象真身（模型里是积分链实现的）
E2 = d2.design.E_model;

say('================ 任务二：渐近跟踪 + 干扰抑制（内模原理） ================\n');
say('【对象】x'' = [0 1; -2 2] x + [0;1] u + E d,  y = x1,  d = 1(t)\n');
say('  模型里 d 的两路注入共用 gain1 -> E = [1;1]；任务书 E = [1;2]（见 references）\n');
say('【内模】p'' = e - 4 q,  q'' = p,  s'' = q     （1/(s(s^2+4))：既管 r 的 cos 也管 d 的阶跃）\n');
say('【控制】u = kyita3 p + kyita2 q + kyita1 s + k_x1 x1 + k_x2 x2\n');
say('  K_aug = [k_x1, k_x2, kyita3, kyita2, kyita1] = %s\n', mat2str(K2, 8));
say('  期望极点 = %s\n', mat2str(sort(complex([d2.design.poles.real], [d2.design.poles.imag])), 6));
say('  增广系统阶数 = %d，能控 rank = %d\n', d2.design.n_aug, d2.design.ctrb_rank);

Aaug2 = [A2, zeros(2, 3); Bc2 * C2, Ac2];
Baug2 = [B2; zeros(3, 1)];
Acl2 = Aaug2 + Baug2 * K2;
poles2 = sort(eig(Acl2));
say('  eig(A_aug + B_aug K) = %s\n', mat2str(poles2, 6));
Bref2 = [zeros(2, 1); -Bc2];
Bd2 = [E2; zeros(3, 1)];
Caug2 = [C2, 0, 0, 0];
zr2 = abs(Caug2 * ((1i * 2 * eye(5) - Acl2) \ Bref2) - 1);
zd2 = abs(Caug2 * ((-Acl2) \ Bd2));
say('  零点判据 |e/r(j2)| = %.2e     |d->e(0)| = %.2e（都应 0）\n\n', zr2, zd2);

s2 = struct('ok', false, 'err', '');
try
    [t2, X1b, X2b, U2, R2, Y2, E2b] = ex_04_2_run_sim(d2, Tf2);
    s2.ok = true;
catch err
    s2.err = err.message;
    say('  Simulink 运行失败：%s\n\n', err.message);
end

if s2.ok
    say('【仿真】Tf = %g s，采样 %d 点，步长中位数 %.4g s\n', Tf2, numel(t2), median(diff(t2)));

    % 解析解：闭环 + 参考外系统 + 常值干扰
    Abig2 = [Acl2, Bref2 * rg2, Bd2;
             zeros(2, 5), Av2, zeros(2, 1);
             zeros(1, 5), zeros(1, 2), Ad];
    z02 = [zeros(5, 1); w0; dstart];
    n2 = numel(t2);
    Za = zeros(n2, 8);
    for k = 1:n2
        Za(k, :) = (expm(Abig2 * t2(k)) * z02).';
    end
    Ua2 = (K2 * Za(:, 1:5).').';
    Ra2 = (rg2 * Za(:, 6:7).').';
    Ya2 = Za(:, 1);
    eU2 = max(abs(U2 - Ua2), [], 'all');
    eY2 = max(abs(Y2 - Ya2), [], 'all');
    say('  与解析解的最大偏差：控制 %.2e，输出 %.2e\n', eU2, eY2);

    tail2 = t2 > Tf2 - 5;
    say('  max|r| = %.4f，max|y| = %.4f，max|u| = %.4f，max|x2| = %.4f\n', ...
        max(abs(R2)), max(abs(Y2)), max(abs(U2)), max(abs(X2b)));
    say('  max|y-r| 全程 = %.3e，末段 = %.3e\n', max(abs(Y2 - R2)), max(abs(Y2(tail2) - R2(tail2))));
    say('  x2 末段均值 = %.4f（解析 %.4f；cos 段不是整周期，均值略偏 -1 是正常的）\n\n', ...
        mean(X2b(tail2)), mean(Za(tail2, 2)));
end

%% ==================== 出图 ====================
if s1.ok
    fig1 = figure('Position', [60 60 1000 560], 'Visible', 'off');
    tiledlayout(fig1, 2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
    nexttile;
    plot(t1, R1, 'LineWidth', 1.4); hold on; plot(t1, Y1, '--', 'LineWidth', 1.8);
    grid on; ylabel('y  [m]');
    legend({'r(t) = 0.1cos2t', 'y(t)'}, 'Location', 'best');
    title('任务一 输出调节（内模 1/(s^2+4)）');
    nexttile;
    plot(t1, E1, 'LineWidth', 1.4); hold on; yline(0, ':');
    grid on; xlabel('t [s]'); ylabel('e = y - r  [m]');
    title(sprintf('跟踪误差（末段 max|e| = %.1e）', max(abs(E1(tail)))));
    export_figure(fig1, outDir, 'ex_04_1_tracking');

    lbl1 = {'x_1 = s  [m]', 'x_2 = ds/dt  [m/s]', 'x_3 = \theta  [rad]', 'x_4 = d\theta/dt  [rad/s]'};
    fig2 = figure('Position', [60 60 1000 620], 'Visible', 'off');
    tiledlayout(fig2, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    for i = 1:4
        nexttile;
        plot(t1, X1(:, i), 'LineWidth', 1.6); hold on;
        if s1.ok, plot(t1, Xa(:, i), '--', 'LineWidth', 1.1); end
        grid on; xlabel('t [s]'); ylabel(lbl1{i});
        if i == 1, legend({'Simulink', '解析解'}, 'Location', 'best'); end
    end
    sgtitle('任务一 状态响应（u = Kx + K_\eta\eta）');
    export_figure(fig2, outDir, 'ex_04_1_states');

    fig3 = figure('Position', [60 60 1000 480], 'Visible', 'off');
    tiledlayout(fig3, 1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    nexttile;
    plot(t1, U1, 'LineWidth', 1.6); hold on;
    if s1.ok, plot(t1, Ua, '--', 'LineWidth', 1.1); end
    grid on; xlabel('t [s]'); ylabel('u');
    legend({'Simulink', '解析解'}, 'Location', 'best');
    title('控制量');
    nexttile;
    plot(t1, Xa(:, 5), 'LineWidth', 1.4); hold on;
    plot(t1, Xa(:, 6), 'LineWidth', 1.4);
    grid on; xlabel('t [s]'); ylabel('内模状态');
    legend({'a（kyita2 那级）', 'b（kyita1 那级）'}, 'Location', 'best');
    title('内模自己"长"出 2 rad/s 的振荡');
    export_figure(fig3, outDir, 'ex_04_1_control');
end

if s2.ok
    fig4 = figure('Position', [60 60 1000 560], 'Visible', 'off');
    tiledlayout(fig4, 2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
    nexttile;
    plot(t2, R2, 'LineWidth', 1.4); hold on; plot(t2, Y2, '--', 'LineWidth', 1.8);
    grid on; ylabel('y = x_1');
    legend({'r(t) = 0.1cos2t', 'y(t) = x_1(t)'}, 'Location', 'best');
    title('任务二 跟踪 + 干扰抑制（d = 1(t) 从 t=0 起）');
    nexttile;
    plot(t2, E2b, 'LineWidth', 1.4); hold on; yline(0, ':');
    grid on; xlabel('t [s]'); ylabel('e = y - r');
    title(sprintf('跟踪误差（末段 max|e| = %.1e）', max(abs(E2b(tail2)))));
    export_figure(fig4, outDir, 'ex_04_2_tracking');

    fig5 = figure('Position', [60 60 1000 480], 'Visible', 'off');
    tiledlayout(fig5, 2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
    nexttile;
    plot(t2, X1b, 'LineWidth', 1.6); grid on; ylabel('x_1 = y');
    title('状态：x_1 跟参考走');
    nexttile;
    plot(t2, X2b, 'LineWidth', 1.6); hold on; yline(-1, '--');
    grid on; xlabel('t [s]'); ylabel('x_2 = dx_1/dt');
    title('状态：x_2 稳态被干扰顶到 -1，所以 x_1 才能不动');
    legend({'x_2(t)', '稳态 -1'}, 'Location', 'best');
    export_figure(fig5, outDir, 'ex_04_2_states');

    fig6 = figure('Position', [60 60 1000 480], 'Visible', 'off');
    tiledlayout(fig6, 1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    nexttile;
    plot(t2, U2, 'LineWidth', 1.6); hold on;
    if s2.ok, plot(t2, Ua2, '--', 'LineWidth', 1.1); end
    grid on; xlabel('t [s]'); ylabel('u');
    legend({'Simulink', '解析解'}, 'Location', 'best');
    title('控制量');
    nexttile;
    plot(t2, X2b, 'LineWidth', 1.2); hold on;
    plot(t2, U2, 'LineWidth', 1.2);
    grid on; xlabel('t [s]');
    legend({'x_2', 'u'}, 'Location', 'best');
    title('u 与 x_2 的量级');
    export_figure(fig6, outDir, 'ex_04_2_control');
end

%% ==================== 存数据 ====================
if s1.ok
    save(fullfile(outDir, 'ex_04_1_data.mat'), 'A1', 'B1', 'c1', 'Av', 'w0', 'rg', ...
         'K1', 'Ac1', 'Bc1', 'poles1', 't1', 'X1', 'U1', 'R1', 'Y1', 'E1', 'eX', 'eU', 'eY');
end
if s2.ok
    save(fullfile(outDir, 'ex_04_2_data.mat'), 'A2', 'B2', 'C2', 'E2', 'Ac2', 'Bc2', ...
         'K2', 'poles2', 't2', 'X1b', 'X2b', 'U2', 'R2', 'Y2', 'E2b', 'eU2', 'eY2');
end
fclose(logFid);

if s1.ok
    fprintf(1, '任务一 K_aug = %s\n', mat2str(round(K1, 6)));
    fprintf(1, '      eig(A_cl) = %s\n', mat2str(round(poles1, 6)));
    fprintf(1, '      max|y-r| 末段 = %.3e, sim vs analytic: X %.2e U %.2e\n', ...
        max(abs(Y1(tail) - R1(tail))), eX, eU);
end
if s2.ok
    fprintf(1, '任务二 K_aug = %s\n', mat2str(round(K2, 6)));
    fprintf(1, '      eig(A_cl) = %s\n', mat2str(round(poles2, 6)));
    fprintf(1, '      max|y-r| 末段 = %.3e, sim vs analytic: U %.2e Y %.2e\n', ...
        max(abs(Y2(tail2) - R2(tail2))), eU2, eY2);
end
fprintf(1, 'figures + data -> %s\n', outDir);
