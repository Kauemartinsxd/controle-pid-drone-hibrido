function fig = plot_guiagem_NL(t, Y, U, WPs, R_accept, titulo, arq)
% PLOT_GUIAGEM_NL  Trajetoria por waypoints + series temporais (h, V_T, psi, phi, theta/alpha,
% de, manete) de uma corrida do guiagem_NL. Y = 14 canais do modelo
% [VT p q r phi theta psi h beta t xN xE psi_abs alpha]; U = [manete de da dr] (rad).
if nargin < 6, titulo = ''; end
if nargin < 7, arq = ''; end
R2D = 180/pi; cor = [0.85 0.33 0.10];
fig = figure('Color', 'w', 'Position', [60 60 1200 640], 'Tag', 'guiagem_NL'); try, fig.Theme = 'light'; catch, end
tl = tiledlayout(fig, 4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, titulo, 'FontWeight', 'bold');
% --- trajetoria (N para cima, E para a direita) ---
ax = nexttile(tl, 1, [4 1]); hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
tc = linspace(0, 2*pi, 100);
for k = 1:size(WPs, 1)
    fill(ax, WPs(k,2) + R_accept*cos(tc), WPs(k,1) + R_accept*sin(tc), [0.92 0.92 0.97], 'EdgeColor', [0.6 0.6 0.8], 'LineStyle', '--');
    text(ax, WPs(k,2) + 0.3*R_accept, WPs(k,1) + 0.3*R_accept, sprintf('WP%d\n%g m, %g m/s', k, WPs(k,3), WPs(k,4)), 'FontSize', 8, 'Color', [0.2 0.2 0.5]);
end
plot(ax, [0; WPs(:,2)], [0; WPs(:,1)], ':', 'Color', [0.5 0.5 0.5]);
plot(ax, Y(12,:), Y(11,:), '-', 'Color', cor, 'LineWidth', 1.4);
plot(ax, 0, 0, 'o', 'MarkerFaceColor', [0.2 0.7 0.2], 'MarkerEdgeColor', 'k', 'MarkerSize', 8);
xlabel(ax, 'E [m]'); ylabel(ax, 'N [m]'); title(ax, 'trajetoria (engate na origem, proa 0)');
% --- series ---
s = {8, 'h [m]', 1; 1, 'V_T [m/s]', 1; 7, '\psi [deg]', R2D; 5, '\phi [deg]', R2D};
for k = 1:4
    ax = nexttile(tl, 2*k); hold(ax, 'on'); grid(ax, 'on');
    plot(ax, t, Y(s{k,1}, :)*s{k,3}, 'Color', cor, 'LineWidth', 1.1);
    ylabel(ax, s{k,2});
    if k == 4, xlabel(ax, 't [s]'); end
end
% figura 2: atitude e comandos
fig2 = figure('Color', 'w', 'Position', [80 80 900 520], 'Tag', 'guiagem_NL'); try, fig2.Theme = 'light'; catch, end
tl2 = tiledlayout(fig2, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact'); title(tl2, titulo, 'FontWeight', 'bold');
ax = nexttile(tl2); plot(ax, t, Y(6,:)*R2D, 'Color', cor, 'LineWidth', 1.1); hold(ax, 'on'); plot(ax, t, Y(14,:)*R2D, 'Color', [0 0.45 0.74], 'LineWidth', 1.1); yline(ax, 18.5, 'r--'); grid(ax, 'on'); ylabel(ax, '[deg]'); legend(ax, '\theta', '\alpha', 'estol 18,5', 'Location', 'best');
ax = nexttile(tl2); plot(ax, t, U(:,2)*R2D, 'Color', cor, 'LineWidth', 1.1); hold(ax, 'on'); plot(ax, t, U(:,3)*R2D, 'Color', [0 0.45 0.74]); plot(ax, t, U(:,4)*R2D, 'Color', [0.47 0.67 0.19]); yline(ax, 15, 'r--'); yline(ax, -15, 'r--'); grid(ax, 'on'); ylabel(ax, 'superficies [deg]'); legend(ax, '\delta_e', '\delta_a', '\delta_r', 'Location', 'best');
ax = nexttile(tl2); plot(ax, t, U(:,1), 'Color', cor, 'LineWidth', 1.1); grid(ax, 'on'); ylabel(ax, 'manete [-]'); ylim(ax, [0 1]); xlabel(ax, 't [s]');
if ~isempty(arq)
    exportgraphics(fig, arq, 'Resolution', 130);
    exportgraphics(fig2, strrep(arq, '.png', '_comandos.png'), 'Resolution', 130);
end
end
