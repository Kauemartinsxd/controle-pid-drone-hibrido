% refaz_figuras_sync.m
% Regera, em TEMA CLARO e com fontes maiores, as quatro figuras do capitulo
% de sincronizacao da dissertacao a partir dos .mat ja salvos:
%   fig_sync_hist100.png, fig_sync_hist20.png  (histogramas do intervalo)
%   fig_sync_xp_20_100.png                      (compara_taxa_20_vs_100)
%   fig_sync_nl_taxas.png                       (NL_taxa_sensores_tabela)
% Copia o resultado para dissertacao-mestrado-ita/03_Sincronizacao/figuras.

here = fileparts(mfilename('fullpath')); addpath(here);
xpV  = fullfile(fileparts(here), 'xplane', 'voos');
dest = 'C:\Users\kaue\Documents\dissertacao\dissertacao\dissertacao-mestrado-ita\03_Sincronizacao\figuras';
set(groot, 'defaultAxesFontSize', 11, 'defaultTextFontSize', 11);

%% histogramas
H = {'XP_missao_20260916_155243_LQRYmirko_psiNovo_oval_confirma_taxa_reload.mat', 'fig_sync_hist100.png', 100; ...
     'XP_missao_20260916_155734_LQRYmirko_psiNovo_oval_confirma_taxa_reload_20Hz.mat', 'fig_sync_hist20.png', 20};
for k = 1:2
    S = load(fullfile(xpV, H{k,1})); tx = S.voo.t_xplane(:);
    novo = [true; diff(tx) > 0]; dtx = diff(tx(novo)); f_ef = (nnz(novo)-1)/(tx(end)-tx(1));
    fig = figure('Color', 'w', 'Position', [100 100 900 420]); claro(fig);
    histogram(1e3*dtx, 0:1:max(50, ceil(1e3*max(dtx))), 'FaceColor', [0.2 0.45 0.7]);
    xlabel('intervalo entre leituras novas do X-Plane [ms]'); ylabel('amostras'); grid on
    title(sprintf('T_s de I/O pedido: %d Hz  |  mediana %.1f ms  |  %.1f Hz efetivo', H{k,3}, 1e3*median(dtx), f_ef));
    exportgraphics(fig, fullfile(dest, H{k,2}), 'Resolution', 150); close(fig);
end

%% comparacao X-Plane 100 vs 20 Hz e varredura NL (scripts existentes, forcando tema claro)
CONF_tema_claro = true; %#ok<NASGU>
compara_taxa_20_vs_100
copyfile(fullfile(xpV, 'LQRYmirko_taxa_20Hz_vs_100Hz_20260916.png'), fullfile(dest, 'fig_sync_xp_20_100.png'));
close all
CONF_arquivos = {};
NL_taxa_sensores_tabela
d = dir(fullfile(xpV, 'NL_taxa_sensores_LQRYmirko_*.png')); [~, j] = max([d.datenum]);
copyfile(fullfile(xpV, d(j).name), fullfile(dest, 'fig_sync_nl_taxas.png'));
close all
fprintf('figuras regeradas em %s\n', dest);

function claro(fig)
% forca tema claro (MATLAB >= R2025a tem fig.Theme; antes disso ja e claro)
try, fig.Theme = 'light'; catch, end
end
