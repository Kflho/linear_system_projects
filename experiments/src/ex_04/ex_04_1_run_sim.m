function [t, X, U, R, Y, ERR, mdlFile] = ex_04_1_run_sim(d, Tf, maxStep)
%EX_04_1_RUN_SIM  实验四-1：倒立摆 + 内模 1/(s^2+4)，跑 ex_04_1_sim.slx。
%
%   [t,X,U,R,Y,ERR,mdlFile] = ex_04_1_run_sim(d, Tf, maxStep)
%
%   d 是 ex_04_design.py 写出的 ex_04_1_design.json（jsondecode 后的结构体），
%   键名就是模型变量名：A,B,C,D,c,A_v,B_v,C_v,D_v,Initial_v,r,k_x,kyita1,kyita2,w。
%
%   模型结构（手搭）：
%       State-Space（A_v..）  w' = A_v w            -> 参考发生器，输入口空着
%       Gain r  [0.1 0]       r = r*w
%       State-Space1（A..D）  x' = A x + B u，C = eye(4) 输出全部状态
%       Gain c  [1 0 0 0]     y = c*x
%       Sum  '|-+'            e = y - r
%       Sum1 '|++'            a' = e + w*b        （w = -4，内模反馈）
%       Integrator/Integrator1  a、b 两级积分：b' = a
%       Sum2 '|+++'           u = kyita2*a + kyita1*b + k_x*x
%
%   只在内存里改模型（跑完 close_system(...,0)，磁盘 .slx 不动）：
%       1) 把变量塞进 base 工作区；
%       2) 所有 Gain 改成 Matrix(K*u)（1x4、1x2 的增益乘列向量）；
%       3) 参考发生器的输入口没接线（w' = A_v w 自治）→ 补一个 Constant 0；
%          它的初值在模型里写死成 0 → 这里显式设成 Initial_v（否则 r ≡ 0）。
%
%   Tf 默认 15 s（≈ 4.8 个参考周期），maxStep 默认 Tf/2000。

if nargin < 2 || isempty(Tf), Tf = 15; end
if nargin < 3 || isempty(maxStep), maxStep = Tf / 2000; end

mdlFile = fullfile(fileparts(mfilename('fullpath')), 'ex_04_1_sim.slx');
if ~exist(mdlFile, 'file')
    error('ex_04_1_run_sim:noModel', '找不到 Simulink 模型：%s', mdlFile);
end
[~, mdl] = fileparts(mdlFile);
if bdIsLoaded(mdl), close_system(mdl, 0); end

load_system(mdlFile);
cleanup = onCleanup(@() close_system(mdl, 0)); %#ok<NASGU>

% ---- 变量进 base --------------------------------------------------------
flds = {'A', 'B', 'C', 'D', 'c', 'A_v', 'B_v', 'C_v', 'D_v', ...
        'Initial_v', 'r', 'k_x', 'kyita1', 'kyita2', 'w'};
for i = 1:numel(flds)
    if ~isfield(d, flds{i})
        error('ex_04_1_run_sim:missingVar', 'ex_04_1_design.json 里缺变量 %s', flds{i});
    end
    assignin('base', flds{i}, d.(flds{i}));
end

% ---- Gain 全部矩阵乘 ----------------------------------------------------
gains = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Gain');
for i = 1:numel(gains)
    set_param(gains{i}, 'Multiplication', 'Matrix(K*u)');
end

% ---- State-Space：补自治输入、设参考初值 ---------------------------------
ssBlks = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'StateSpace');
for i = 1:numel(ssBlks)
    name = get_param(ssBlks{i}, 'Name');
    ph = get_param(ssBlks{i}, 'PortHandles');
    if get_param(ph.Inport(1), 'Line') == -1          % 输入口空着就补 0
        zeroName = ['Zero_' name];
        if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', zeroName))
            pos = get_param(ssBlks{i}, 'Position');
            add_block('simulink/Sources/Constant', [mdl '/' zeroName], 'Value', '0', ...
                      'Position', [pos(1) - 60, pos(2), pos(1) - 20, pos(2) + 30]);
        end
        add_line(mdl, [zeroName '/1'], [name '/1'], 'autorouting', 'on');
    end
    if strcmp(get_param(ssBlks{i}, 'A'), 'A_v')       % 参考发生器初值
        set_param(ssBlks{i}, 'X0', mat2str(d.Initial_v(:), 16));
    end
end

% ---- 求解器 -------------------------------------------------------------
set_param(mdl, 'StopTime', num2str(Tf), ...
               'Solver', 'ode45', 'RelTol', '1e-8', 'AbsTol', '1e-10', ...
               'MaxStep', num2str(maxStep));

% ---- 仿真 ---------------------------------------------------------------
out = sim(mdl);
want = {'x_sim', 'u_sim', 'ry_sim', 'ktita2'};
[logs, names] = slx_get_logs(out, want);
[t, X] = slx_unpack(logs{1});
[~, U] = slx_unpack(logs{2});
[t2, RY] = slx_unpack(logs{3});
[t3, ER] = slx_unpack(logs{4});
if numel(t) ~= numel(t2)
    RY = interp1(t2, RY, t, 'pchip');
    ER = interp1(t3, ER, t, 'pchip');
end
R = RY(:, 1);
Y = RY(:, end);
ERR = ER(:, 1);
U = reshape(U, numel(t), []);
end

% ------------------------------------------------------------------------
function [logs, names] = slx_get_logs(out, want)
%SLX_GET_LOGS  取 To Workspace 变量（R2024b 返回 SimulationOutput）
if isa(out, 'Simulink.SimulationOutput')
    names = cellstr(out.who);
    missing = setdiff(want, names);
    if ~isempty(missing)
        error('ex_04_1_run_sim:noLog', '模型里缺 To Workspace 输出：%s（现有：%s）', ...
            strjoin(missing, ', '), strjoin(names.', ', '));
    end
    logs = cellfun(@(n) out.get(n), want, 'UniformOutput', false);
else
    missing = want(~isfield(out, want));
    if ~isempty(missing)
        error('ex_04_1_run_sim:noLog', '模型里缺 To Workspace 输出：%s', strjoin(missing, ', '));
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
    error('ex_04_1_run_sim:logFormat', '不认识的 To Workspace 输出格式。');
end
end
