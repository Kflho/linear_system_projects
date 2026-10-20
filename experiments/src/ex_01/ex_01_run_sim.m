function [t, X, U, mdlFile] = ex_01_run_sim(A, B, C, D, K, x0, Tf, maxStep)
%EX_01_RUN_SIM  给手搭的 ex_01_sim.slx 灌参数并运行，返回状态/控制响应。
%
%   [t,X,U,mdlFile] = ex_01_run_sim(A,B,C,D,K,x0,Tf,maxStep)
%
%   模型由使用者手搭（按实验指导书图）：State-Space 用变量 A,B,C,D，Gain 用
%   变量 K，To Workspace 输出 x_sim / u_sim。本函数只在内存里：
%       1) 把 A,B,C,D,K,x0 变量放到 base 工作区；
%       2) 把 State-Space 的 X0 设成 x0，把 Gain 的乘法方式改成 Matrix(K*u)
%          （Element-wise 对 2x4 的 K 与 4x1 的 x 会报维数错）；
%       3) 跑 sim，取 x_sim / u_sim。
%   跑完 close_system(...,0) 丢弃改动，磁盘上的 .slx 不会被改。
%
%   注意：C 必须是 eye(4)，这样 State-Space 输出就是四个状态本身。
%   maxStep 用于限制步长（默认 Tf/1000），保证与解析解对照时采样够密。

if nargin < 8 || isempty(maxStep), maxStep = Tf / 1000; end
if nargin < 7 || isempty(Tf), Tf = 10; end

mdlFile = fullfile(fileparts(mfilename('fullpath')), 'ex_01_sim.slx');
if ~exist(mdlFile, 'file')
    error('ex_01_run_sim:noModel', '找不到 Simulink 模型：%s', mdlFile);
end
[~, mdl] = fileparts(mdlFile);
if bdIsLoaded(mdl), close_system(mdl, 0); end

load_system(mdl);
cleanup = onCleanup(@() close_system(mdl, 0));

% ---- 变量放进 base 工作区 ---------------------------------------------
assignin('base', 'A', A);
assignin('base', 'B', B);
assignin('base', 'C', C);
assignin('base', 'D', D);
assignin('base', 'K', K);
assignin('base', 'x0', x0(:));

% ---- 定位模块 ----------------------------------------------------------
ssBlk = find_system(mdl, 'BlockType', 'StateSpace');
if isempty(ssBlk)
    error('ex_01_run_sim:noSS', '模型里找不到 State-Space 模块。');
end
ssBlk = ssBlk{1};

gBlk = find_system(mdl, 'BlockType', 'Gain');
if isempty(gBlk)
    error('ex_01_run_sim:noGain', '模型里找不到 Gain 模块。');
end
gBlk = gBlk{1};

% ---- 注入参数（只改内存） ---------------------------------------------
set_param(ssBlk, 'A', 'A', 'B', 'B', 'C', 'C', 'D', 'D', ...
                 'X0', mat2str(x0(:), 16));
set_param(gBlk, 'Gain', 'K', 'Multiplication', 'Matrix(K*u)');

set_param(mdl, 'StopTime', num2str(Tf), ...
               'Solver', 'ode45', 'RelTol', '1e-8', 'AbsTol', '1e-10', ...
               'MaxStep', num2str(maxStep));

% ---- 仿真 --------------------------------------------------------------
out = sim(mdl);

% R2024b 的 sim 返回 Simulink.SimulationOutput 对象，用 who/get 取变量
if isa(out, 'Simulink.SimulationOutput')
    names = out.who;
    if ~ismember('x_sim', names) || ~ismember('u_sim', names)
        error('ex_01_run_sim:noLog', ...
            '模型里没找到 x_sim / u_sim 两个 To Workspace 输出（现有：%s）。', ...
            strjoin(names(:).', ', '));
    end
    xlog = out.get('x_sim');
    ulog = out.get('u_sim');
else
    if ~isfield(out, 'x_sim') || ~isfield(out, 'u_sim')
        error('ex_01_run_sim:noLog', ...
            '模型里没找到 x_sim / u_sim 两个 To Workspace 输出。');
    end
    xlog = out.x_sim;
    ulog = out.u_sim;
end

[t, X] = unpack_log(xlog);
[~, U] = unpack_log(ulog);
if numel(t) ~= size(U, 1)
    U = interp1(ulog.Time, ulog.Data, t, 'pchip');
end
end

% ---------------------------------------------------------------------
function [t, Y] = unpack_log(v)
%UNPACK_LOG  把 To Workspace 的输出（Timeseries 或 struct）统一成 t,Y。
if isa(v, 'timeseries')
    t = v.Time;
    Y = reshape(v.Data, numel(t), []);
elseif isstruct(v) && isfield(v, 'time')
    t = v.time;
    Y = reshape(v.signals.values, numel(t), []);
else
    error('ex_01_run_sim:logFormat', '不认识的 To Workspace 输出格式。');
end
end
