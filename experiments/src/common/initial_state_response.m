function [t, X, U] = initial_state_response(A, B, K, x0, tfinal, N)
%INITIAL_STATE_RESPONSE  状态反馈闭环 u = Kx 的初始状态响应（解析解）。
%
%   [t, X, U] = initial_state_response(A, B, K, x0, tfinal, N)
%
%   闭环系统  xdot = (A + B*K) x,  u = K x,  x(0) = x0：
%
%       x(t) = expm((A+BK) t) x0
%       u(t) = K x(t)              （代数关系，别写成积分）
%
%   实验一第 (5) 问就是要求用 expm(A*t) 算这条响应曲线。

if nargin < 6 || isempty(N),      N = 800;  end
if nargin < 5 || isempty(tfinal), tfinal = 10; end

n  = size(A, 1);
Ac = A + B * K;
t  = linspace(0, tfinal, N).';

X = zeros(N, n);
for k = 1:N
    X(k, :) = (expm(Ac * t(k)) * x0(:)).';
end
U = X * K.';
end
