function plot_XP_missao(voo, vooFile)
%PLOT_XP_MISSAO Figuras de uma missao por waypoints do DH no X-Plane.
%
% Uso:
%   plot_XP_missao(voo)            % apos XP_missao (struct no workspace)
%   plot_XP_missao(voo, vooFile)   % tambem salva PNGs ao lado do .mat
%   v = load('...\XP_missao_*.mat'); plot_XP_missao(v.voo)   % re-plot
%
% Fig 1: trajetoria 2D (Leste x Norte) com WPs, circulos de R_accept e
%        marcas de troca de wp_idx. Fig 2: series temporais.
% (Adaptacao 2D do plot3d_voo_xplane.m do PIPER-1-6 do Julio.)

    if nargin < 2, vooFile = ''; end
    Y = voo.Y; U = voo.U; t = voo.t;
    xN = Y(11,:); xE = Y(12,:);
    WPs = voo.WPs; R = voo.R_accept;
    nWP = size(WPs,1);
    % logs da guiagem rodam a Ts=0.05 (chart), tout no passo do solver
    % (0.005) — eixo de tempo proprio p/ wp_idx/dist
    t_wp = linspace(t(1), t(end), numel(voo.wp_idx))';

    %% Fig 1 — trajetoria 2D
    f1 = figure('Name', 'XP_missao — trajetoria 2D', 'Color', 'w', ...
        'Position', [80 80 720 720]);
    hold on; axis equal; grid on;
    th = linspace(0, 2*pi, 90);
    for i = 1:nWP
        fill(WPs(i,2) + R*cos(th), WPs(i,1) + R*sin(th), [0.9 0.95 1], ...
            'EdgeColor', [0.3 0.5 0.9], 'LineStyle', '--', 'FaceAlpha', 0.5);
        plot(WPs(i,2), WPs(i,1), 'b^', 'MarkerFaceColor', 'b', 'MarkerSize', 7);
        text(WPs(i,2)+18, WPs(i,1)+18, sprintf('WP%d', i), 'Color', 'b', 'FontWeight', 'bold');
    end
    plot(xE, xN, 'k-', 'LineWidth', 1.4);
    plot(xE(1), xN(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 9);
    plot(xE(end), xN(end), 'rs', 'MarkerFaceColor', 'r', 'MarkerSize', 9);
    % marca as trocas de waypoint sobre a trajetoria (mapeia t_wp -> t)
    if isfield(voo, 'wp_idx') && ~isempty(voo.wp_idx)
        isw = find(diff(voo.wp_idx(:)) > 0);
        it = max(1, min(numel(xE), round(interp1(t, 1:numel(t), t_wp(isw)))));
        plot(xE(it), xN(it), 'mo', 'MarkerFaceColor', 'm', 'MarkerSize', 6);
    end
    xlabel('Leste [m]'); ylabel('Norte [m]');
    title(sprintf('DH no X-Plane — missao por waypoints (R_{accept} = %.0f m)', R));

    %% Fig 2 — series temporais
    f2 = figure('Name', 'XP_missao — estados', 'Color', 'w', ...
        'Position', [820 80 720 780]);
    subplot(5,1,1);
    plot(t, Y(1,:), 'b', 'LineWidth', 1.1); hold on;
    if isfield(voo.cfg, 'VT_ref'), yline(voo.cfg.VT_ref, 'r--'); end
    grid on; ylabel('V_T [m/s]');
    title('Missao por waypoints — cascata PID (ganhos da dissertacao)');
    subplot(5,1,2);
    plot(t, Y(8,:), 'b', 'LineWidth', 1.1); hold on;
    stairs(t_wp, WPs(max(round(voo.wp_idx(:)),1), 3), 'r--');
    grid on; ylabel('h MSL [m]'); legend('h', 'h_{WP}', 'Location', 'best');
    subplot(5,1,3);
    plot(t, rad2deg(Y(5,:)), 'Color', [0 0.6 0]); hold on;
    plot(t, rad2deg(Y(6,:)), 'Color', [0.85 0.4 0]);
    grid on; ylabel('[deg]'); legend('\phi', '\theta', 'Location', 'best');
    subplot(5,1,4);
    plot(t_wp, voo.dist_wp, 'b', 'LineWidth', 1.1); hold on;
    yline(R, 'r--');
    yyaxis right; stairs(t_wp, voo.wp_idx, 'm-'); ylabel('wp idx');
    yyaxis left; grid on; ylabel('dist WP [m]');
    subplot(5,1,5);
    plot(t, U(:,1), 'b', 'LineWidth', 1.1); hold on;
    plot(t, rad2deg(U(:,2))/25, 'Color', [0.85 0.4 0]);
    grid on; ylabel('thr | de/25deg'); xlabel('t [s]');
    legend('\delta_T', '\delta_e (norm)', 'Location', 'best');

    %% PNGs ao lado do .mat (registro da campanha)
    if ~isempty(vooFile)
        [p, n] = fileparts(vooFile);
        try
            exportgraphics(f1, fullfile(p, [n '_traj.png']), 'Resolution', 110);
            exportgraphics(f2, fullfile(p, [n '_series.png']), 'Resolution', 110);
            fprintf('PNGs salvos: %s_traj.png / _series.png\n', fullfile(p, n));
        catch ME
            fprintf('plot_XP_missao: falha ao salvar PNG (%s)\n', ME.message);
        end
    end
end
