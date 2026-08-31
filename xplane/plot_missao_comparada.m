function plot_missao_comparada(vooXP, vooNL, vooFileXP)
%PLOT_MISSAO_COMPARADA  Mesma missao: X-Plane (gemeo) x modelo NL (Ana).
%
% Sobrepoe a trajetoria 2D (no referencial da PROA DE ENGATE, comum aos
% dois mundos) e as series h/VT/psi. Chamada automaticamente pelo
% XP_missao (secao 9); tambem funciona avulsa:
%   X = load('...XP_missao_*.mat'); N = load('...NL_missao_*_autoNL.mat');
%   plot_missao_comparada(X.voo, N.voo, '...XP_missao_*.mat')
%
% Cores seguem o tema do MATLAB (claro/escuro).

    if nargin < 3, vooFileXP = ''; end

    % XP: trajetoria NE -> referencial de engate (rotacao por psi_engate)
    cp = cosd(vooXP.psi_engate); sp = sind(vooXP.psi_engate);
    xN = vooXP.Y(11,:); xE = vooXP.Y(12,:);
    frenteXP  = xN*cp + xE*sp;
    direitaXP = xE*cp - xN*sp;
    % NL: engate na origem com psi=0 -> ja esta no referencial certo
    frenteNL  = vooNL.Y(11,:);
    direitaNL = vooNL.Y(12,:);
    WPs = vooNL.WPs;                    % [frente direita alt vel]
    R = vooNL.R_accept;

    f = figure('Name', 'Missao comparada — X-Plane x NL', 'Position', [60 60 1180 640]);
    escuro = false;
    try, escuro = strcmpi(f.Theme.BaseColorStyle, 'dark'); catch, end
    if escuro
        cXP = [1.00 0.60 0.20]; cNL = [0.40 0.65 1.00];
        cFill = [0.16 0.22 0.34]; cEdge = [0.45 0.62 0.95]; cRef = [0.7 0.7 0.7];
    else
        set(f, 'Color', 'w');
        cXP = [0.85 0.33 0.10]; cNL = [0.00 0.45 0.74];
        cFill = [0.90 0.95 1.00]; cEdge = [0.30 0.50 0.90]; cRef = [0.45 0.45 0.45];
    end

    % --- trajetoria 2D ---
    subplot(3,2,[1 3 5]); hold on; axis equal; grid on;
    th = linspace(0, 2*pi, 90);
    for i = 1:size(WPs,1)
        fill(WPs(i,2) + R*cos(th), WPs(i,1) + R*sin(th), cFill, ...
            'EdgeColor', cEdge, 'LineStyle', '--', 'FaceAlpha', 0.5);
        text(WPs(i,2)+15, WPs(i,1)+15, sprintf('WP%d', i), ...
            'Color', cEdge, 'FontWeight', 'bold');
    end
    hNL = plot(direitaNL, frenteNL, '-', 'Color', cNL, 'LineWidth', 1.4);
    hXP = plot(direitaXP, frenteXP, '-', 'Color', cXP, 'LineWidth', 1.2);
    plot(0, 0, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 9);
    xlabel('a direita do engate [m]'); ylabel('a frente do engate [m]');
    title('Trajetoria: mesma missao nos dois mundos');
    legend([hNL hXP], {'NL (modelo da Ana)', 'X-Plane (gemeo v1.1)'}, 'Location', 'best');

    % --- series ---
    tX = vooXP.t; tN = vooNL.t;
    subplot(3,2,2); hold on; grid on;
    plot(tN, vooNL.Y(8,:), '-', 'Color', cNL, 'LineWidth', 1.2);
    plot(tX, vooXP.Y(8,:), '-', 'Color', cXP, 'LineWidth', 1.0);
    ylabel('h [m]'); title('X-Plane (laranja) x NL (azul)');
    subplot(3,2,4); hold on; grid on;
    plot(tN, vooNL.Y(1,:), '-', 'Color', cNL, 'LineWidth', 1.2);
    plot(tX, vooXP.Y(1,:), '-', 'Color', cXP, 'LineWidth', 1.0);
    ylabel('V_T [m/s]');
    subplot(3,2,6); hold on; grid on;
    plot(tN, rad2deg(vooNL.Y(7,:)), '-', 'Color', cNL, 'LineWidth', 1.2);
    plot(tX, rad2deg(vooXP.Y(7,:)), '-', 'Color', cXP, 'LineWidth', 1.0);
    ylabel('\psi rel. engate [deg]'); xlabel('t [s]');

    if ~isempty(vooFileXP)
        [p, n] = fileparts(vooFileXP);
        try
            exportgraphics(f, fullfile(p, [n '_compNL.png']), 'Resolution', 120);
            fprintf('Comparacao salva: %s_compNL.png\n', fullfile(p, n));
        catch e
            fprintf('plot_missao_comparada: falha ao salvar PNG (%s)\n', e.message);
        end
    end
end
