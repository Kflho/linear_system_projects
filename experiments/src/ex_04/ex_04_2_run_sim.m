function [t, X1, X2, U, R, Y, ERR, mdlFile] = ex_04_2_run_sim(d, Tf, maxStep)
%EX_04_2_RUN_SIM  实验四-2：渐近跟踪 + 干扰抑制，跑 ex_04_2_sim.slx。
%
%   [t,X1,X2,U,R,Y,ERR,mdlFile] = ex_04_2_run_sim(d, Tf, maxStep)
%
%   d 是 ex_04_design.py 写出的 ex_04_2_design.json，键名就是模型变量名：
%   A_v,B_v,C_v,D_v,Initial_v,r / A_d,B_d,C_d,D_d,d_start /
%   gain1,gain2,gain6,gain7 / k_x1,k_x2,kyita1,kyita2,kyita3。
%
%   模型结构（手搭，对象是积分链）：
%       State-Space（A_v..）  w' = A_v w，参考发生器，输入口空着
%       Gain r                r = 0.1 w_1
%       State-Space1（A_d..） 干扰发生器 x_d' = A_d x_d，x_d(0) = d_start -> d = 1(t)
%       Sum  '|-+'            e = y - r          （y = x1，从 Mux 第二路看）
%       Sum1 '|++'            p' = e + gain6*q   （gain6 = -4，内模反馈）
%       Integrator/Integrator2/Integrator1   p -> q -> s 三级积分
%       Sum4 '|++'            v'  = gain1*d + z          （v = x1）
%       Sum3 '|++++'          z'  = gain1*d + u + gain2*v + gain7*z   （z = x2）
%       Sum2 '|+++++'         u = kyita3*p + kyita1*s + kx2*z + kx1*v + kyita2*q
%
%   ⚠ 对象那两路差分里 d 的系数都用了 gain1（模型现状 ⇒ E = [1;1]）；
%     任务书是 E = [1;2]。内模含 1/s，对任意常值干扰都能压到零，结论不受影响；
%     要严格对齐任务书，在 z' 那一路上单独再串一个 x2 的增益即可（设计不用重算）。
%
%   只在内存里改模型（close_system 丢弃）：
%       1) 变量塞 base；
%       2) Gain 全部改 Matrix(K*u)；
%       3) 两个 State-Space 的输入口都没接线 → 各补一个 Constant 0；
%          参考发生器初值写死成 0 → 显式设成 Initial_v（否则 r ≡ 0）；
%          干扰发生器初值用 d_start（本来就是变量，顺手也设一次）。
%
%   Tf 默认 25 s（≈ 8 个参考周期，够看清 d 的阶跃），maxStep 默认 Tf/2000。

if nargin < 2 || isempty(Tf), Tf = 25; end
if nargin < 3 || isempty(maxStep), maxStep = Tf / 2000; end

mdlFile = fullfile(fileparts(mfilename('fullpath')), 'ex_04_2_sim.slx');
if ~exist(mdlFile, 'file')
    error('ex_04_2_run_sim:noModel', '找不到 Simulink 模型：%s', mdlFile);
end
[~, mdl] = fileparts(mdlFile);
if bdIsLoaded(mdl), close_system(mdl, 0); end

load_system(mdlFile);
cleanup = onCleanup(@() close_system(mdl, 0)); %#ok<NASGU>

% ---- 变量进 base --------------------------------------------------------
flds = {'A_v', 'B_v', 'C_v', 'D_v', 'Initial_v', 'r', ...
        'A_d', 'B_d', 'C_d', 'D_d', 'd_start', ...
        'gain1', 'gain2', 'gain6', 'gain7', ...
        'k_x1', 'k_x2', 'kyita1', 'kyita2', 'kyita3'};
for i = 1:numel(flds)
    if ~isfield(d, flds{i})
        error('ex_04_2_run_sim:missingVar', 'ex_04_2_design.json 里缺变量 %s', flds{i});
    end
    assignin('base', flds{i}, d.(flds{i}));
end

% ---- Gain 全部矩阵乘 ----------------------------------------------------
gains = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Gain');
for i = 1:numel(gains)
    set_param(gains{i}, 'Multiplication', 'Matrix(K*u)');
end

% ---- State-Space：补自治输入、设初值 -------------------------------------
ssBlks = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'StateSpace');
for i = 1:numel(ssBlks)
    name = get_param(ssBlks{i}, 'Name');
    ph = get_param(ssBlks{i}, 'PortHandles');
    if get_param(ph.Inport(1), 'Line') == -1
        zeroName = ['Zero_' name];
        if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', zeroName))
            pos = get_param(ssBlks{i}, 'Position');
            add_block('simulink/Sources/Constant', [mdl '/' zeroName], 'Value', '0', ...
                      'Position', [pos(1) - 60, pos(2), pos(1) - 20, pos(2) + 30]);
        end
        add_line(mdl, [zeroName '/1'], [name '/1'], 'autorouting', 'on');
    end
    switch get_param(ssBlks{i}, 'A')
        case 'A_v'
            set_param(ssBlks{i}, 'X0', mat2str(d.Initial_v(:), 16));
        case 'A_d'
            set_param(ssBlks{i}, 'X0', mat2str(double(d.d_start), 16));
    end
end

% ---- 求解器 -------------------------------------------------------------
set_param(mdl, 'StopTime', num2str(Tf), ...
               'Solver', 'ode45', 'RelTol', '1e-8', 'AbsTol', '1e-10', ...
               'MaxStep', num2str(maxStep));

% ---- 仿真 ---------------------------------------------------------------
out = sim(mdl);
want = {'ry_sim', 'u_sim', 'ktita2', 'x_x2_sim'};
[logs, ~] = slx_get_logs(out, want);
[t, RY] = slx_unpack(logs{1});
[~, U] = slx_unpack(logs{2});
[t3, ER] = slx_unpack(logs{3});
[t4, X2] = slx_unpack(logs{4});
if numel(t) ~= numel(t3)
    ER = interp1(t3, ER, t, 'pchip');
    X2 = interp1(t4, X2, t, 'pchip');
end
R = RY(:, 1);
Y = RY(:, end);       % Mux 第 2 路是 v = x1 = y
ERR = ER(:, 1);
U = reshape(U, numel(t), []);
X2 = reshape(X2, numel(t), []);
X1 = Y;               % x1 就是输出
end

% ------------------------------------------------------------------------
function [logs, names] = slx_get_logs(out, want)
if isa(out, 'Simulink.SimulationOutput')
    names = cellstr(out.who);
    missing = setdiff(want, names);
    if ~isempty(missing)
        error('ex_04_2_run_sim:noLog', '模型里缺 To Workspace 输出：%s（现有：%s）', ...
            strjoin(missing, ', '), strjoin(names.', ', '));
    end
    logs = cellfun(@(n) out.get(n), want, 'UniformOutput', false);
else
    missing = want(~isfield(out, want));
    if ~isempty(missing)
        error('ex_04_2_run_sim:noLog', '模型里缺 To Workspace 输出：%s', strjoin(missing, ', '));
    end
    logs = cellfun(@(n) out.(n), want, 'UniformOutput', false);
    names = fieldnames(out);
end
end

function [t, Y] = slx_unpack(v)
if isa(v, 'timeseries')
    t = v.Time;
    Y = reshape(v.Data, numel(t), []);
elseif isstruct(v) && isfield(v, 'time')
    t = v.time;
    Y = reshape(v.signals.values, numel(t), []);
else
    error('ex_04_2_run_sim:logFormat', '不认识的 To Workspace 输出格式。');
end
end
