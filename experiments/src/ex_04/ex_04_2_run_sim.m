function [t, X1, X2, U, R, Y, ERR, mdlFile] = ex_04_2_run_sim(d, Tf, maxStep)
%EX_04_2_RUN_SIM  实验四-2：渐近跟踪 + 干扰抑制，跑 ex_04_2_sim.slx。
%
%   [t,X1,X2,U,R,Y,ERR,mdlFile] = ex_04_2_run_sim(d, Tf, maxStep)
%
%   d 是 ex_04_design.py 写出的 ex_04_2_design.json，键名就是模型变量名：
%   A_v,B_v,C_v,D_v,v_start,r / A_d,B_d,C_d,D_d,d_start /
%   gain1,gain2,gain6,gain7 / k_x1,k_x2,kyita1,kyita2,kyita3。
%
%   模型结构（手搭，对象是积分链）：
%       State-Space（A_v..）  w' = A_v w + B_v*0，初值 v_start   -> 参考发生器
%       Gain r                r = 0.1 w_1
%       State-Space1（A_d..） 干扰发生器 x_d' = A_d x_d，x_d(0) = d_start -> d = 1(t)
%       Sum  '|-+'            e = y - r          （y = x1，从 Mux 第二路看）
%       Sum1 '|++'            p' = e + gain6*q   （gain6 = -4，内模反馈）
%       Integrator/Integrator2/Integrator1   p -> q -> s 三级积分
%       Sum4 '|++'            v'  = d + z                        （v = x1；d 直接进）
%       Sum3 '|++++'          z'  = gain1*d + u + gain2*v + gain7*z（z = x2；gain1 = 2）
%       Sum2 '|+++++'         u = kyita3*p + kyita1*s + kx2*z + kx1*v + kyita2*q
%
%   d 拆成两路后 E = [1;2]，与任务书一致（设计里 E 不出现，K 不受影响）。
%   模型自带的两个 Constant 0 已接好自治输入，初值也是变量，所以这里只塞变量 + 设 Gain 乘法方式。
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
flds = {'A_v', 'B_v', 'C_v', 'D_v', 'v_start', 'r', ...
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

% ---- State-Space 输入口检查（模型里已各接一个 Constant 0）-----------------
ssBlks = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'StateSpace');
for i = 1:numel(ssBlks)
    ph = get_param(ssBlks{i}, 'PortHandles');
    if get_param(ph.Inport(1), 'Line') == -1
        error('ex_04_2_run_sim:unconnected', ...
            'State-Space「%s」的输入口没接线（自治的话接一个 Constant 0）。', ...
            get_param(ssBlks{i}, 'Name'));
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
