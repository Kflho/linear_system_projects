function fname = export_figure(fig, outDir, name, res)
%EXPORT_FIGURE  把图窗保存为 PNG（同时保存 .fig 便于后续修改）。
%
%   fname = export_figure(fig, outDir, name)
%   fname = export_figure(fig, outDir, name, resolution)

if nargin < 4 || isempty(res), res = 200; end
if ~exist(outDir, 'dir'), mkdir(outDir); end

fname = fullfile(outDir, [name '.png']);
exportgraphics(fig, fname, 'Resolution', res);
savefig(fig, fullfile(outDir, [name '.fig']));
end
