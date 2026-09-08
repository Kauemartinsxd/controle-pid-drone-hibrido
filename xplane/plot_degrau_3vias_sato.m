% plot_degrau_3vias_sato.m
% =============================================================
% Figura "estilo Sato" (dissertacao do Sato, Figs. de validacao):
% a MESMA manobra de profundor sobreposta painel a painel nos tres
% mundos, com dados de VOO reais do X-Plane:
%
%   - modelo da Ana (nao linear, simulado com trimagem_DH +
%     dyn_rigidbody_DH, degrau a partir do trim exato);
%   - .acf ORIGINAL (medido em voo: XP_ident_theta_20260831_125617,
%     ponto de operacao pitch0 2 / de_op 2 / thr 0.5);
%   - gemeo v1.1  (medido em voo: XP_ident_theta_20260830_170637 =
%     R pitch 1.47 ft, fit conjunto wn 6.25 / zeta 0.61).
%
% Paineis: de absoluto (estatica: cada mundo opera noutro nivel de
% profundor) + respostas Delta q / Delta theta / Delta VT (dinamica;
% deriva do ponto de operacao removida pela corrida baseline, mesmo
% metodo da identificacao / XP_retune_Ctheta).
%
% Gera: voos/EQUIV_sato_degrau_p2.png (degrau +2) e _m2.png (-2).
% =============================================================

xpDir = fileparts(mfilename('fullpath'));
repo  = fileparts(xpDir);
addpath(fullfile(repo, 'utilitarios'));

FN_ORIG  = fullfile(xpDir, 'voos', 'XP_ident_theta_20260831_125617.mat');
FN_GEMEO = fullfile(xpDir, 'voos', 'XP_ident_theta_20260830_170637.mat');

So = load(FN_ORIG);
Sg = load(FN_GEMEO);

% ---- trim exato do modelo da Ana a 12 m/s (nivelado, 600 m) ----
% (trimagem_DH usa fsolve/Optimization Toolbox, ausente nesta maquina;
%  mesmo residuo [udot wdot qdot] minimizado por fminsearch)
[Xe, Ue] = local_trim(12, 600, 0);
fprintf('Trim Ana: alpha %.2f deg | de %+.2f deg | thr %.3f\n', ...
    rad2deg(atan(Xe(3)/Xe(1))), rad2deg(Ue(2)), Ue(1));

t_grid = (0:0.05:1.5)';           % janela de validade do curto periodo
t_pre  = (-0.5:0.05:0)';          % trecho pre-degrau (contexto no painel de de)

for dstep = +2                     % resposta antissimetrica equivale p/ -2
    % ---- modelo da Ana: degrau de profundor a partir do trim ----
    Ustep = @(t) Ue + [0; deg2rad(dstep)*(t>=0); 0; 0; 0; 0; 0];
    [tnl, Xnl] = ode45(@(t,X) dyn_rigidbody_DH(t, X, Ustep(t)), t_grid, Xe);
    VTnl = sqrt(Xnl(:,1).^2 + Xnl(:,3).^2);
    qnl  = rad2deg(Xnl(:,5));
    thnl = rad2deg(Xnl(:,8));

    % ---- X-Plane: combinacao antissimetrica (run(+2) - run(-2))/2 —
    %      cancela deriva/fugoide de modo comum (1a ordem) e estima a
    %      resposta linear ao degrau de +2 (mesmo espirito do fit
    %      conjunto do XP_retune_Ctheta, aqui direto nos dados)
    [dq_o, dth_o, dVT_o] = local_resp_anti(So, t_grid);
    [dq_g, dth_g, dVT_g] = local_resp_anti(Sg, t_grid);

    % ---- figura ----
    f = figure('Color','w','Position',[80 40 860 940], 'Visible','off');
    try, f.Theme = 'light'; catch, end   % ignora tema escuro do desktop
    tl = tiledlayout(4,1,'TileSpacing','compact','Padding','compact');

    cA = [0 0.447 0.741];         % Ana  (azul, como na fig. 3 vias)
    cO = [0.45 0.45 0.45];        % original (cinza tracejado)
    cG = [0.850 0.325 0.098];     % gemeo (laranja)

    % 1) profundor ABSOLUTO — a estatica: cada mundo noutro nivel
    nexttile; hold on; grid on;
    tt = [t_pre; t_grid];
    stairs(tt, rad2deg(Ue(2)) + dstep*(tt>=0), '-',  'Color', cA, 'LineWidth', 1.4);
    stairs(tt, So.de_op       + dstep*(tt>=0), '--', 'Color', cO, 'LineWidth', 1.4);
    stairs(tt, Sg.de_op       + dstep*(tt>=0), '-',  'Color', cG, 'LineWidth', 1.4);
    ylabel('\delta_e [{\circ}]'); ylim([-1 12]); xlim([t_pre(1) t_grid(end)]);
    title(sprintf(['Degrau de profundor %+d° em malha aberta — mesma manobra nos três mundos\n' ...
        '(\\delta_e absoluto: os níveis de operação diferem; era o trim que não batia)'], dstep), ...
        'Interpreter','tex');
    legend({'modelo da Ana (\delta_e trim +7,6°)', ...
            '.acf ORIGINAL (\delta_e op +2°)', ...
            'gêmeo v1.1 (\delta_e op +5°)'}, ...
            'Location','east', 'Interpreter','tex');

    % trecho pre-degrau nulo p/ alinhar o eixo x com o painel de de
    pad = @(y) [zeros(numel(t_pre),1); y(:)];

    % 2) resposta de q
    nexttile; hold on; grid on;
    plot(tt, pad(qnl - qnl(1)), '-',  'Color', cA, 'LineWidth', 1.6);
    plot(tt, pad(dq_o),         '--', 'Color', cO, 'LineWidth', 1.4);
    plot(tt, pad(dq_g),         '-',  'Color', cG, 'LineWidth', 1.6);
    ylabel('\Delta q [{\circ}/s]'); xlim([t_pre(1) t_grid(end)]);
    legend({'modelo da Ana (\omega_n 6,43, \zeta 0,55)', ...
            '.acf ORIGINAL (sobreamortecido)', ...
            'gêmeo v1.1 (\omega_n 6,25, \zeta 0,61)'}, ...
            'Location','northeast', 'Interpreter','tex');

    % 3) resposta de theta
    nexttile; hold on; grid on;
    plot(tt, pad(thnl - thnl(1)), '-',  'Color', cA, 'LineWidth', 1.6);
    plot(tt, pad(dth_o),          '--', 'Color', cO, 'LineWidth', 1.4);
    plot(tt, pad(dth_g),          '-',  'Color', cG, 'LineWidth', 1.6);
    ylabel('\Delta\theta [{\circ}]'); xlim([t_pre(1) t_grid(end)]);

    % 4) resposta de VT
    nexttile; hold on; grid on;
    plot(tt, pad(VTnl - VTnl(1)), '-',  'Color', cA, 'LineWidth', 1.6);
    plot(tt, pad(dVT_o),          '--', 'Color', cO, 'LineWidth', 1.4);
    plot(tt, pad(dVT_g),          '-',  'Color', cG, 'LineWidth', 1.6);
    ylabel('\Delta V_T [m/s]'); xlabel('t [s]'); xlim([t_pre(1) t_grid(end)]);

    fn = fullfile(xpDir, 'voos', 'EQUIV_sato_degrau_3vias.png');
    exportgraphics(f, fn, 'Resolution', 130);
    close(f);
    fprintf('Figura salva: %s\n', fn);
end

function [Xe, Ue] = local_trim(Ve, he, gammae)
    montaX = @(y) [Ve*cos(y(1)); 0; Ve*sin(y(1)); 0; 0; 0; 0; ...
                   y(1)+gammae; 0; 0; 0; -he; 0; 0];
    montaU = @(y) [y(2); y(3); 0; 0; 0; 0; 0];
    resid  = @(y) subsref(dyn_rigidbody_DH(0, montaX(y), montaU(y)), ...
                          struct('type','()','subs',{{[1 3 5]}}));
    J = @(y) sum(resid(y).^2);
    y0 = [deg2rad(14.4), 0.284, deg2rad(7.6)];   % chute = valores conhecidos
    opt = optimset('MaxFunEvals',5e4,'MaxIter',5e4,'TolFun',1e-16,'TolX',1e-14);
    y  = fminsearch(J, y0, opt);
    y  = fminsearch(J, y, opt);                  % refina (residuo ~1e-12)
    fprintf('Trim fminsearch: residuo J = %.3e\n', J(y));
    Xe = montaX(y); Ue = montaU(y);
end

function [dq, dth, dVT] = local_resp_anti(S, t_grid)
    % resposta linear ao degrau de +2 via (run(+2) - run(-2))/2 —
    % cancela em 1a ordem a deriva/fugoide de modo comum das corridas
    ip = find([S.runs.step_deg] == +2, 1);
    im = find([S.runs.step_deg] == -2, 1);
    Dp = S.runs(ip).D; Dm = S.runs(im).D;
    % D = [t_wall q(rad/s) theta(deg) VT(m/s) h t_xp]
    g  = @(D,c) interp1(D(:,1), D(:,c), t_grid, 'linear', 'extrap');
    dq  = rad2deg(g(Dp,2) - g(Dm,2))/2;
    dth = (g(Dp,3) - g(Dm,3))/2;
    dVT = (g(Dp,4) - g(Dm,4))/2;
    % referencia na 1a amostra pos-comando
    dq = dq - dq(1); dth = dth - dth(1); dVT = dVT - dVT(1);
end
