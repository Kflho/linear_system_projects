function [t, X, U, R, Y, mdlFile] = ex_03_run_sim(d, Tf, maxStep)
%EX_03_RUN_SIM  给手搭的 ex_03_sim.slx 灌参数并运行，返回参考/状态/控制/输出。
%
%   [t,X,U,R,Y,mdlFile] = ex_03_run_sim(d, Tf, maxStep)
%
%   d 是 ex_03_design.py 写出的 ex_03_design.json（jsondecode 之后的结构体），
%   里面正好是模型用的变量名：A,B,C,D,c,A_v,B_v,C_v,D_v,Initial_v,r,k_x,k_v。
%
%   模型由使用者手搭（按实验指导书图）：
%       State-Space1  x' = A x + B u, y = C x + D u    （C = eye(4)，输出全部状态）
%       Gain4 (c)     y = c x
%       Mux           ry = [r; y]          -> To Workspace: ry_sim
%       Gain3 (k_x)   u = k_x x + k_v w    -> To Workspace: u_sim
%       Gain2 (k_v)
%       State-Space   w' = A_v w + B_v 0   （参考发生器，输入口空着）
%       State-Space1  -> To Workspace: x_sim
%
%   本函数只在内存里动模型，跑完 close_system(...,0)，磁盘上的 .slx 不会被改：
%       1) 把上面那些变量放进 base 工作区；
%       2) 把所有 Gain 的乘法方式设成 Matrix(K*u)（1x4、1x2 的增益乘列向量时
%          Element-wise 会报维数错）；
%       3) 参考发生器是自治的（w' = A_v w），但模型里它的输入口没接线 ——
%          Simulink 会直接报错。这里补一个 Constant 0 接上（第 3 点是模型可以
%          改进的地方，见 bin/references/ex_03.md）。
%
%   Tf 默认 10 s（覆盖 3 个参考周期 2pi/2 ≈ 3.14 s 还要多），maxStep 默认 Tf/2000。

if nargin < 2 || isempty(Tf), Tf = 10; end
if nargin < 3 || isempty(maxStep), maxStep = Tf / 2000; end

mdlFile = fullfile(fileparts(mfilename('fullpath')), 'ex_03_sim.slx');
if ~exist(mdlFile, 'file')
    error('ex_03_run_sim:noModel', '找不到 Simulink 模型：%s', mdlFile);
end
[~, mdl] = fileparts(mdlFile);
if bdIsLoaded(mdl), close_system(mdl, 0); end

load_system(mdlFile);
cleanup = onCleanup(@() close_system(mdl, 0)); %#ok<NASGU>

% ---- 变量放进 base 工作区 ------------------------------------------------
flds = {'A', 'B', 'C', 'D', 'c', 'A_v', 'B_v', 'C_v', 'D_v', 'Initial_v', ...
        'r', 'k_x', 'k_v'};
for i = 1:numel(flds)
    if ~isfield(d, flds{i})
        error('ex_03_run_sim:missingVar', 'ex_03_design.json 里缺变量 %s', flds{i});
    end
    assignin('base', flds{i}, d.(flds{i}));
end

% ---- 所有 Gain 改成矩阵乘 ----------------------------------------------
gains = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Gain');
for i = 1:numel(gains)
    set_param(gains{i}, 'Multiplication', 'Matrix(K*u)');
end

% ---- 参考发生器的输入口：没接线就补一个 Constant 0 ----------------------
ssBlks = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'StateSpace');
for i = 1:numel(ssBlks)
    if ~strcmp(get_param(ssBlks{i}, 'A'), 'A_v')
        continue;                                  % 只处理外系统那个模块
    end
    ph = get_param(ssBlks{i}, 'PortHandles');
    if get_param(ph.Inport(1), 'Line') == -1
        zeroName = 'Zero_v_in';
        if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', zeroName))
            pos = get_param(ssBlks{i}, 'Position');
            add_block('simulink/Sources/Constant', [mdl '/' zeroName], ...
                      'Value', '0', 'Position', [pos(1) - 60, pos(2), pos(1) - 20, pos(2) + 30]);
        end
        add_line(mdl, [zeroName '/1'], [get_param(ssBlks{i}, 'Name') '/1'], ...
                 'autorouting', 'on');
    end
end

% ---- 求解器设置 ---------------------------------------------------------
set_param(mdl, 'StopTime', num2str(Tf), ...
               'Solver', 'ode45', 'RelTol', '1e-8', 'AbsTol', '1e-10', ...
               'MaxStep', num2str(maxStep));

% ---- 仿真 ---------------------------------------------------------------
out = sim(mdl);

want = {'x_sim', 'u_sim', 'ry_sim'};
if isa(out, 'Simulink.SimulationOutput')
    names = cellstr(out.who);
    missing = setdiff(want, names);
    if ~isempty(missing)
        error('ex_03_run_sim:noLog', ...
            '模型里缺 To Workspace 输出：%s（现有：%s）。', ...
            strjoin(missing, ', '), strjoin(names.', ', '));
    end
    xlog = out.get('x_sim');
    ulog = out.get('u_sim');
    rylog = out.get('ry_sim');
else
    missing = want(~isfield(out, want));
    if ~isempty(missing)
        error('ex_03_run_sim:noLog', '模型里缺 To Workspace 输出：%s', strjoin(missing, ', '));
    end
    xlog = out.x_sim;
    ulog = out.u_sim;
    rylog = out.ry_sim;
end

[t, X] = unpack_log(xlog);
[~, U] = unpack_log(ulog);
[t2, RY] = unpack_log(rylog);
if numel(t) ~= numel(t2)
    RY = interp1(t2, RY, t, 'pchip');
end
R = RY(:, 1);              % Mux 的第 1 路是 r
Y = RY(:, end);            % 第 2 路是 y
U = reshape(U, numel(t), []);
end

% ------------------------------------------------------------------------
function [t, Y] = unpack_log(v)
%UNPACK_LOG  把 To Workspace 的输出（Timeseries 或 struct）统一成 t,Y。
if isa(v, 'timeseries')
    t = v.Time;
    Y = reshape(v.Data, numel(t), []);
elseif isstruct(v) && isfield(v, 'time')
    t = v.time;
    Y = reshape(v.signals.values, numel(t), []);
else
    error('ex_03_run_sim:logFormat', '不认识的 To Workspace 输出格式。');
end
end
