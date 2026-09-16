% compara_taxa_20_vs_100.m
% Compara os dois voos de confirmacao do LQRy do Mirko (psi Hold novo) no
% gemeo v2, 2026-09-16: mesma missao (oval GUI, 15 m/s), mesmos ganhos,
% so a janela de I/O com o X-Plane muda (Ts_io 0,01 s vs 0,05 s).
% Saida: tabela no console + txt + figura sobreposta em xplane/voos.

xpV = 'C:\Users\kaue\Documents\dissertacao\dissertacao\controle-pid-drone-hibrido\xplane\voos';
F = {fullfile(xpV, 'XP_missao_20260916_155243_LQRYmirko_psiNovo_oval_confirma_taxa_reload.mat'), ...
     fullfile(xpV, 'XP_missao_20260916_155734_LQRYmirko_psiNovo_oval_confirma_taxa_reload_20Hz.mat')};
nome = {'100 Hz', '20 Hz'};
R2D = 180/pi;
L = {};
L{end+1} = sprintf('%-34s | %12s | %12s', 'metrica', nome{1}, nome{2});
L{end+1} = repmat('-', 1, 64);
M = struct();
% Janela de comparacao: o voo a 100 Hz engatou 77 s apos o load do .acf e o
% estoque do motor eletrico acabou nos ultimos ~8 s (VT cai, manete enrola
% apos os 78 s). Metricas so ate T_MAX, onde os dois voos estao com motor.
T_MAX = 75;
L{end+1} = sprintf('(metricas de voo calculadas ate t = %.0f s; taxa no voo inteiro)', T_MAX);
for k = 1:2
    S = load(F{k}); voo = S.voo; Y = voo.Y; U = voo.U; t = voo.t(:);
    jan = t <= T_MAX; Y = Y(:, jan); U = U(jan, :); t = t(jan);
    m = struct();
    m.Ts_io   = voo.Ts_io;
    m.dur     = t(end);
    m.h_min   = min(Y(8,:)); m.h_max = max(Y(8,:)); m.h_rms = rms(Y(8,:) - voo.cfg.XP_msl0);
    m.VT_min  = min(Y(1,:)); m.VT_max = max(Y(1,:)); m.VT_rms = rms(Y(1,:) - voo.cfg.VT);
    m.phi_max = max(abs(Y(5,:)))*R2D;
    m.th_min  = min(Y(6,:))*R2D; m.th_max = max(Y(6,:))*R2D;
    m.al_max  = max(Y(14,:))*R2D;
    m.de_min  = min(U(:,2))*R2D; m.de_max = max(U(:,2))*R2D;
    m.da_max  = max(abs(U(:,3)))*R2D; m.dr_max = max(abs(U(:,4)))*R2D;
    m.thr_min = min(U(:,1)); m.thr_max = max(U(:,1));
    m.thr_fora = 100*mean(U(:,1) < 0 | U(:,1) > 1);
    m.sup_sat  = 100*mean(abs(U(:,2))*R2D > 15 | abs(U(:,3))*R2D > 15 | abs(U(:,4))*R2D > 15);
    % periodo do ciclo-limite de VT (cruzamentos de zero de VT - media, apos 20 s)
    mm = t > 20; v = detrend(Y(1,mm)); tt = t(mm);
    zc = tt([false; diff(sign(v(:))) ~= 0]);
    if numel(zc) > 4, m.T_ciclo = 2*median(diff(zc)); else, m.T_ciclo = NaN; end
    % taxa real
    tx = voo.t_xplane(:); novo = [true; diff(tx) > 0]; dtx = diff(tx(novo));
    m.f_ef = (nnz(novo)-1)/(tx(end)-tx(1)); m.dt_med = 1e3*median(dtx);
    dts = sort(dtx); m.dt_p99 = 1e3*dts(max(1, round(0.99*numel(dts)))); m.dt_max = 1e3*max(dtx);
    m.rep = 100*(1 - nnz(novo)/numel(tx));
    % capturas: distancia minima a cada WP
    W = voo.WPs; d = zeros(1, size(W,1));
    for w = 1:size(W,1), d(w) = min(hypot(Y(11,:) - W(w,1), Y(12,:) - W(w,2))); end
    m.dist = d;
    M(k).m = m; M(k).voo = voo;
end
f = @(fmt, a, b) sprintf(['%-34s | %12s | %12s'], '', sprintf(fmt, a), sprintf(fmt, b));
row = @(lab, fmt, fld) sprintf('%-34s | %12s | %12s', lab, sprintf(fmt, M(1).m.(fld)), sprintf(fmt, M(2).m.(fld)));
L{end+1} = row('Ts_io [s]', '%.2f', 'Ts_io');
L{end+1} = row('duracao [s] (fim automatico)', '%.1f', 'dur');
L{end+1} = sprintf('%-34s | %12s | %12s', 'h [m] min..max', sprintf('%.1f..%.1f', M(1).m.h_min, M(1).m.h_max), sprintf('%.1f..%.1f', M(2).m.h_min, M(2).m.h_max));
L{end+1} = row('h RMS em torno de 600 [m]', '%.2f', 'h_rms');
L{end+1} = sprintf('%-34s | %12s | %12s', 'VT [m/s] min..max', sprintf('%.1f..%.1f', M(1).m.VT_min, M(1).m.VT_max), sprintf('%.1f..%.1f', M(2).m.VT_min, M(2).m.VT_max));
L{end+1} = row('VT RMS em torno de 15 [m/s]', '%.2f', 'VT_rms');
L{end+1} = row('periodo do ciclo de VT [s]', '%.1f', 'T_ciclo');
L{end+1} = row('phi max [deg]', '%.1f', 'phi_max');
L{end+1} = sprintf('%-34s | %12s | %12s', 'theta [deg] min..max', sprintf('%.1f..%.1f', M(1).m.th_min, M(1).m.th_max), sprintf('%.1f..%.1f', M(2).m.th_min, M(2).m.th_max));
L{end+1} = row('alpha max [deg] (estol 18,5)', '%.1f', 'al_max');
L{end+1} = sprintf('%-34s | %12s | %12s', 'de cmd [deg] min..max', sprintf('%+.1f..%+.1f', M(1).m.de_min, M(1).m.de_max), sprintf('%+.1f..%+.1f', M(2).m.de_min, M(2).m.de_max));
L{end+1} = row('|da| max cmd [deg]', '%.1f', 'da_max');
L{end+1} = row('|dr| max cmd [deg]', '%.1f', 'dr_max');
L{end+1} = sprintf('%-34s | %12s | %12s', 'manete cmd min..max', sprintf('%.2f..%.2f', M(1).m.thr_min, M(1).m.thr_max), sprintf('%.2f..%.2f', M(2).m.thr_min, M(2).m.thr_max));
L{end+1} = row('manete cmd fora de [0,1] [%]', '%.0f', 'thr_fora');
L{end+1} = row('superficie cmd > 15 deg [%]', '%.0f', 'sup_sat');
L{end+1} = sprintf('%-34s | %12s | %12s', 'dist min aos WPs [m]', mat2str(round(M(1).m.dist(1:end-1),1)), mat2str(round(M(2).m.dist(1:end-1),1)));
L{end+1} = repmat('-', 1, 64);
L{end+1} = row('taxa efetiva de leitura [Hz]', '%.1f', 'f_ef');
L{end+1} = row('dt entre leituras: mediana [ms]', '%.1f', 'dt_med');
L{end+1} = row('dt entre leituras: p99 [ms]', '%.1f', 'dt_p99');
L{end+1} = row('dt entre leituras: max [ms]', '%.1f', 'dt_max');
L{end+1} = row('leituras repetidas [%]', '%.1f', 'rep');
fprintf('\n%s\n', L{:});
fid = fopen(fullfile(xpV, 'LQRYmirko_taxa_20Hz_vs_100Hz_20260916.txt'), 'w'); fprintf(fid, '%s\n', L{:}); fclose(fid);

%% figura sobreposta
fig = figure('Color', 'w', 'Position', [100 100 900 900]);
tl = tiledlayout(5, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
cor = {[0 0.45 0.74], [0.85 0.33 0.1]};
ax1 = nexttile; hold on; for k = 1:2, plot(M(k).voo.t, M(k).voo.Y(1,:), 'Color', cor{k}); end; ylabel('V_T [m/s]'); yline(15, 'k:'); grid on; legend(nome, 'Location', 'best');
ax2 = nexttile; hold on; for k = 1:2, plot(M(k).voo.t, M(k).voo.Y(8,:), 'Color', cor{k}); end; ylabel('h MSL [m]'); yline(600, 'k:'); grid on;
ax3 = nexttile; hold on; for k = 1:2, plot(M(k).voo.t, M(k).voo.Y(5,:)*R2D, 'Color', cor{k}); end; ylabel('\phi [deg]'); grid on;
ax4 = nexttile; hold on; for k = 1:2, plot(M(k).voo.t, M(k).voo.Y(14,:)*R2D, 'Color', cor{k}); end; ylabel('\alpha [deg]'); yline(18.5, 'r:'); grid on;
ax5 = nexttile; hold on; for k = 1:2, plot(M(k).voo.t, M(k).voo.U(:,1), 'Color', cor{k}); end; ylabel('manete cmd'); yline(0, 'k:'); yline(1, 'k:'); grid on; xlabel('t [s]');
linkaxes([ax1 ax2 ax3 ax4 ax5], 'x');
title(tl, 'LQRy do Mirko (\psi Hold novo) no gemeo v2 — oval GUI 15 m/s: laco X-Plane a 100 Hz vs 20 Hz (2026-09-16)');
exportgraphics(fig, fullfile(xpV, 'LQRYmirko_taxa_20Hz_vs_100Hz_20260916.png'), 'Resolution', 120);
fprintf('figura salva em %s\n', fullfile(xpV, 'LQRYmirko_taxa_20Hz_vs_100Hz_20260916.png'));
