function root = project_paths()
%PROJECT_PATHS  把 src/ 下所有目录加入 MATLAB 搜索路径，返回工程根目录。
%
%   root = project_paths()
%
%   调用后即可直接使用 common/ 下的工具函数（controllability_index、
%   pole_placement_duan、initial_state_response、export_figure）。

here = fileparts(mfilename('fullpath'));   % .../src/common
srcRoot = fileparts(here);                 % .../src
root = fileparts(srcRoot);                 % 工程根目录

addpath(here);
addpath(genpath(srcRoot));
end
