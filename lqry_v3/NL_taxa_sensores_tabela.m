% NL_taxa_sensores_tabela.m
% Tabela + figura da varredura de taxa de I/O no modelo NL (NL_taxa_sensores_lqry).
% Le os .mat listados em CONF_arquivos (ou, se vazio, os NL_missao_*_taxaIO_*.mat
% mais recentes de cada taxa em xplane/voos).

here = fileparts(mfilename('fullpath'));
xpV  = fullfile(fileparts(here), 'xplane', 'voos');
R2D = 180/pi; T_MAX = 75;
if ~exist('CONF_arquivos', 'var') || isempty(CONF_arquivos)
    rots = {'continuo', '100Hz', '50Hz', '20Hz', '10Hz', '5Hz'}; CONF_arquivos = {};
    for r = rots
        d = dir(fullfile(xpV, ['NL_missao_*_taxaIO_' r{1} '.mat']));
        if ~isempty(d), [~, j] = max([d.datenum]); CONF_arquivos{end+1} = fullfile(xpV, d(j).name); end %#ok<SAGROW>
    end
end
n = numel(CONF_arquivos);
res = struct('rotulo', {}, 'voo', {}, 'm', {});
for k = 1:n
    S = load(CONF_arquivos{k}); voo = S.voo;
    tk = regexp(CONF_arquivos{k}, 'taxaIO_(\w+)\.mat$', 'tokens'); rot = tk{1}{1};
    Y = voo.Y; U = voo.U; t = voo.t(:);
    W = voo.WPs; cap = 0;
    for w = 1:size(W,1), cap = cap + (min(hypot(Y(11,:) - W(w,1), Y(12,:) - W(w,2))) <= voo.R_accept); end
    k_dep = find(abs(Y(5,:))*R2D > 60, 1);  k_st = find(Y(14,:)*R2D > 18.5, 1);
    m = struct('fim', t(end), 'cap', cap, ...
        't_dep', NaN, 't_est', NaN);
    if ~isempty(k_dep), m.t_dep = t(k_dep); end
    if ~isempty(k_st),  m.t_est = t(k_st);  end
    jan = t <= T_MAX; Y = Y(:, jan); U = U(jan, :);
    m.h_rms = rms(Y(8,:) - 600); m.h_min = min(Y(8,:)); m.h_max = max(Y(8,:));
    m.VT_rms = rms(Y(1,:) - 15); m.VT_min = min(Y(1,:)); m.VT_max = max(Y(1,:));
    m.phi_max = max(abs(Y(5,:)))*R2D; m.th_min = min(Y(6,:))*R2D; m.th_max = max(Y(6,:))*R2D;
    m.al_max = max(Y(14,:))*R2D; m.de_min = min(U(:,2))*R2D; m.de_max = max(U(:,2))*R2D;
    m.thr_min = min(U(:,1)); m.thr_max = max(U(:,1));
    m.thr_fora = 100*mean(U(:,1) < 0 | U(:,1) > 1);
    m.sup_sat = 100*mean(max(abs(U(:,2:4)), [], 2)*R2D > 15);
    res(k) = struct('rotulo', rot, 'voo', voo, 'm', m);
end
col = @(c) strjoin(cellfun(@(x) sprintf('%10s', x), c, 'UniformOutput', false), ' | ');
lin  = @(lab, fmt, f)      [sprintf('%-30s | ', lab) col(arrayfun(@(r) sprintf(fmt, r.m.(f)), res, 'UniformOutput', false))];
lin2 = @(lab, fmt, f1, f2) [sprintf('%-30s | ', lab) col(arrayfun(@(r) sprintf(fmt, r.m.(f1), r.m.(f2)), res, 'UniformOutput', false))];
L = {};
L{end+1} = [sprintf('%-30s | ', 'metrica') col({res.rotulo})];
L{end+1} = repmat('-', 1, numel(L{1}));
L{end+1} = sprintf('(oval GUI 15 m/s, LQRy do Mirko psi novo, planta NL Ana, motor tau 0,3 s, curso 15 deg; metricas ate %d s)', T_MAX);
L{end+1} = lin('capturas (de 6)', '%d', 'cap');
L{end+1} = lin('fim [s]', '%.1f', 'fim');
L{end+1} = lin('estol (alpha>18,5) em t [s]', '%.1f', 't_est');
L{end+1} = lin('departure (|phi|>60) em t [s]', '%.1f', 't_dep');
L{end+1} = lin('h RMS [m]', '%.2f', 'h_rms');
L{end+1} = lin2('h min..max [m]', '%.1f..%.1f', 'h_min', 'h_max');
L{end+1} = lin('VT RMS [m/s]', '%.2f', 'VT_rms');
L{end+1} = lin2('VT min..max [m/s]', '%.1f..%.1f', 'VT_min', 'VT_max');
L{end+1} = lin('phi max [deg]', '%.1f', 'phi_max');
L{end+1} = lin2('theta min..max [deg]', '%.1f..%.1f', 'th_min', 'th_max');
L{end+1} = lin('alpha max [deg]', '%.1f', 'al_max');
L{end+1} = lin2('de cmd min..max [deg]', '%+.1f..%+.1f', 'de_min', 'de_max');
L{end+1} = lin2('manete cmd min..max', '%.2f..%.2f', 'thr_min', 'thr_max');
L{end+1} = lin('manete fora de [0,1] [%]', '%.0f', 'thr_fora');
L{end+1} = lin('superficie cmd > 15 deg [%]', '%.0f', 'sup_sat');
fprintf('\n%s\n', L{:});
stamp = datestr(now, 'yyyymmdd_HHMMSS');
fid = fopen(fullfile(xpV, ['NL_taxa_sensores_LQRYmirko_' stamp '.txt']), 'w'); fprintf(fid, '%s\n', L{:}); fclose(fid);

fig = figure('Color', 'w', 'Position', [100 100 950 950]); try, fig.Theme = 'light'; catch, end
tl = tiledlayout(5, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
cores = lines(n); ax = gobjects(1,5);
ax(1) = nexttile; hold on; for k = 1:n, plot(res(k).voo.t, res(k).voo.Y(1,:), 'Color', cores(k,:)); end; ylabel('V_T [m/s]'); yline(15,'k:'); grid on; legend({res.rotulo}, 'Location', 'eastoutside');
ax(2) = nexttile; hold on; for k = 1:n, plot(res(k).voo.t, res(k).voo.Y(8,:), 'Color', cores(k,:)); end; ylabel('h [m]'); yline(600,'k:'); grid on; ylim([560 640]);
ax(3) = nexttile; hold on; for k = 1:n, plot(res(k).voo.t, res(k).voo.Y(5,:)*R2D, 'Color', cores(k,:)); end; ylabel('\phi [deg]'); grid on; ylim([-90 90]);
ax(4) = nexttile; hold on; for k = 1:n, plot(res(k).voo.t, res(k).voo.Y(14,:)*R2D, 'Color', cores(k,:)); end; ylabel('\alpha [deg]'); yline(18.5,'r:'); grid on; ylim([-10 30]);
ax(5) = nexttile; hold on; for k = 1:n, plot(res(k).voo.t, res(k).voo.U(:,1), 'Color', cores(k,:)); end; ylabel('manete cmd'); yline(0,'k:'); yline(1,'k:'); grid on; ylim([-1 2]); xlabel('t [s]');
linkaxes(ax, 'x'); xlim(ax(1), [0 90]);
title(tl, 'LQRy do Mirko (\psi Hold novo), planta NL: taxa de I/O do controlador (motor \tau 0,3 s)');
exportgraphics(fig, fullfile(xpV, ['NL_taxa_sensores_LQRYmirko_' stamp '.png']), 'Resolution', 150);
fprintf('salvo: %s\n', fullfile(xpV, ['NL_taxa_sensores_LQRYmirko_' stamp '.{txt,png}']));
