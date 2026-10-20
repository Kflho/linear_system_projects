function [t, X, Y, U, mdlFile] = ex_02_run_sim(A, B, C, D, K, H, v, Tf, maxStep)
%EX_02_RUN_SIM  给手搭的 ex_02_sim.slx 灌参数并运行（解耦控制实验）。
%
%   [t,X,Y,U] = ex_02_run_sim(A,B,C,D,K,H,v,Tf,maxStep)
%
%   模型结构（使用者按实验图手搭）：
%       v1,v2 ──► Mux ──► H ──┐
%                             ├─► Sum ──► State-Space ──┬─► x_sim
%              State-Space ──► K ───────┘               └─► C ──► y_sim
%
%   本函数只在内存里做四件事：
%     1) A,B,C,D,K,H 以及阶跃幅值 v1_gain/v2_gain 放进 base 工作区；
%     2) 三个增益模块的乘法方式改成 Matrix(K*u)（Element-wise 会报维数错）；
%     3) State-Space 的 C 改成 eye(4)（这样输出就是四个状态，y 由 C 增益另算），
%        X0 置零（阶跃响应从静止开始）；
%     4) 在 Sum 输出上挂一个 u_log（模型里 u_sim 接的是 Hv，不是真控制量
%        u = Kx + Hv）。
%   跑完 close_system(...,0) 丢弃改动，磁盘上的 .slx 不会被改。

if nargin < 9 || isempty(maxStep), maxStep = Tf / 1000; end
if nargin < 8 || isempty(Tf), Tf = 10; end

mdlFile = fullfile(fileparts(mfilename('fullpath')), 'ex_02_sim.slx');
if ~exist(mdlFile, 'file')
    error('ex_02_run_sim:noModel', '找不到 Simulink 模型：%s', mdlFile);
end
[~, mdl] = fileparts(mdlFile);
if bdIsLoaded(mdl), close_system(mdl, 0); end

load_system(mdl);
cleanup = onCleanup(@() close_system(mdl, 0));   %#ok<NASGU>

n = size(A, 1);
m = size(B, 2);

% ---- 变量 ------------------------------------------------------------
assignin('base', 'A', A);
assignin('base', 'B', B);
assignin('base', 'C', C);
assignin('base', 'D', D);
assignin('base', 'K', K);
assignin('base', 'H', H);
assignin('base', 'v1_start', 0);
assignin('base', 'v2_start', 0);
assignin('base', 'v1_gain', v(1));
assignin('base', 'v2_gain', v(2));

% ---- 增益模块 --------------------------------------------------------
gains = find_system(mdl, 'BlockType', 'Gain');
for i = 1:numel(gains)
    set_param(gains{i}, 'Multiplication', 'Matrix(K*u)');
end

% ---- 状态空间模块 ----------------------------------------------------
ssBlk = find_system(mdl, 'BlockType', 'StateSpace');
if isempty(ssBlk)
    error('ex_02_run_sim:noSS', '模型里找不到 State-Space 模块。');
end
ssBlk = ssBlk{1};
% 输出取四个状态（C = I4），所以 D 必须是 4 x m 的零阵，而不是模型的 2x2
Dss = zeros(n, m);
set_param(ssBlk, 'A', 'A', 'B', 'B', ...
                 'C', mat2str(eye(n)), 'D', mat2str(Dss), ...
                 'X0', mat2str(zeros(n, 1)));

% ---- 挂一个 u_log 取真正的 u = Kx + Hv -------------------------------
sumBlk = find_system(mdl, 'BlockType', 'Sum');
if isempty(sumBlk)
    error('ex_02_run_sim:noSum', '模型里找不到 Sum 模块（u = Kx + Hv）。');
end
sumBlk = sumBlk{1};
add_block('simulink/Sinks/To Workspace', [mdl '/u_log'], ...
    'VariableName', 'u_log', 'SaveFormat', 'Timeseries', ...
    'Position', [520 380 600 420]);
add_line(mdl, [get_param(sumBlk, 'Name') '/1'], 'u_log/1', 'autorouting', 'on');

set_param(mdl, 'StopTime', num2str(Tf), ...
               'Solver', 'ode45', 'RelTol', '1e-8', 'AbsTol', '1e-10', ...
               'MaxStep', num2str(maxStep));

% ---- 仿真 ------------------------------------------------------------
out = sim(mdl);

[t, X]  = get_log(out, 'x_sim', n);
[~, Y]  = get_log(out, 'y_sim', 2);
[~, U]  = get_log(out, 'u_log', m);
end

% ---------------------------------------------------------------------
function [t, Y] = get_log(out, name, ncol)
if isa(out, 'Simulink.SimulationOutput')
    names = out.who;
    if ~ismember(name, names)
        error('ex_02_run_sim:noLog', ...
            '模型输出里没有 %s（现有：%s）。', name, strjoin(names(:).', ', '));
    end
    v = out.get(name);
else
    v = out.(name);
end

if isa(v, 'timeseries')
    t = v.Time;
    Y = reshape(v.Data, numel(t), []);
elseif isstruct(v) && isfield(v, 'time')
    t = v.time;
    Y = reshape(v.signals.values, numel(t), []);
else
    error('ex_02_run_sim:logFormat', '不认识的 To Workspace 输出格式。');
end
Y = reshape(Y, numel(t), ncol);
end
