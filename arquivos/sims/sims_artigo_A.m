%% sims_artigo_A — itens do artigo: polos/estabilidade (Sec. II), simulacao
%  em malha aberta (Sec. II) e PID NL x linear nas 3 manobras (Sec. IV).
%  Saidas: Artigo\figs\*.png + Artigo\dados_sims_A.txt (tabelas p/ LaTeX).

run('C:\Users\kaue\Documents\PID_DH\controle-pid-drone-hibrido\DH_inicializacao.m');
artigo = 'C:\Users\kaue\Documents\PID_DH\Artigo';
figdir = fullfile(artigo, 'figs');
fid = fopen(fullfile(artigo, 'dados_sims_A.txt'), 'w');
R2D = 180/pi;
the = Xe(8); dEe = Ue(2); dTe = Ue(1);

%% ================= A1: polos e modos =================
fprintf(fid, '===== POLOS (linearizacao em Ve=12, he=600) =====\n');
grupos = {'LONGITUDINAL', eig(A_long); 'LATERO-DIRECIONAL', eig(A_lat)};
for g = 1:2
    fprintf(fid, '--- %s ---\n', grupos{g,1});
    e = grupos{g,2};
    % agrupa: pares complexos + reais
    usados = false(size(e));
    for k = 1:numel(e)
        if usados(k), continue; end
        if abs(imag(e(k))) > 1e-9
            par = find(abs(e - conj(e(k))) < 1e-9 & ~usados & (1:numel(e))' ~= k, 1);
            usados(k) = true; if ~isempty(par), usados(par) = true; end
            wn = abs(e(k)); z = -real(e(k))/wn;
            fprintf(fid, 'par  %+.4f +/- %.4fi   wn=%.4f rad/s  zeta=%.3f\n', ...
                real(e(k)), abs(imag(e(k))), wn, z);
        else
            usados(k) = true;
            fprintf(fid, 'real %+.6f            tau=%s s\n', real(e(k)), ...
                ternario(abs(real(e(k)))>1e-9, sprintf('%.1f', 1/abs(real(e(k)))), 'inf'));
        end
    end
end
% matrizes em LaTeX
fprintf(fid, '\n===== MATRIZES (LaTeX) =====\n');
nomes = {'A_long', A_long; 'B_long', B_long; 'A_lat', A_lat; 'B_lat', B_lat};
for k = 1:4
    M = nomes{k,2};
    fprintf(fid, '%s = \\begin{bmatrix}\n', nomes{k,1});
    for i = 1:size(M,1)
        fprintf(fid, '%s \\\\\n', strjoin(compose('%.4g', M(i,:)), ' & '));
    end
    fprintf(fid, '\\end{bmatrix}\n');
end

%% ================= A2: malha aberta =================
odef = @(t,x) dyn_rigidbody_DH(t, x, Ue);
opts = odeset('RelTol', 1e-7, 'AbsTol', 1e-9);
[t0s, x0s] = ode45(odef, [0 300], Xe, opts);                 % trim exato
Xp = Xe; Xp(8) = Xp(8) + deg2rad(1); Xp(7) = Xp(7) + deg2rad(5);
[t1s, x1s] = ode45(odef, [0 300], Xp, opts);                 % perturbado

sinaisMA = {
  @(x) sqrt(sum(x(:,1:3).^2,2)), 'Airspeed V_T', 'm/s';
  @(x) x(:,8)*R2D,               '\theta',       'deg';
  @(x) x(:,7)*R2D,               '\phi',         'deg';
  @(x) -x(:,12),                 'Altitude',     'm'};
fig = figure('Color','w'); try, set(fig,'Theme','light'); catch, end
set(fig,'Units','normalized','Position',[0.1 0.1 0.8 0.75]);
tl = tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
title(tl, 'Open-loop nonlinear simulation with commands frozen at trim', 'FontWeight','bold');
for k = 1:4
    ax = nexttile(tl); hold(ax,'on'); grid(ax,'on');
    plot(ax, t0s, sinaisMA{k,1}(x0s), '-',  'Color',[0 0.447 0.741], 'LineWidth',1.4);
    plot(ax, t1s, sinaisMA{k,1}(x1s), '--', 'Color',[0.85 0.325 0.098], 'LineWidth',1.2);
    title(ax, sinaisMA{k,2}); ylabel(ax, sinaisMA{k,3});
    if k>=3, xlabel(ax,'Time [s]'); end
    if k==1, legend(ax, {'Initialized at trim', ...
        'Perturbed (\Delta\theta = 1\circ, \Delta\phi = 5\circ)'}, 'Location','best'); end
end
exportgraphics(fig, fullfile(figdir,'fig_malha_aberta.png'), 'Resolution',300, 'BackgroundColor','white');
close(fig);
fprintf(fid, '\n===== MALHA ABERTA =====\n');
fprintf(fid, 'trim exato: max|dVT|=%.3g  max|dtheta|=%.3g deg  max|dh|=%.3g m (300 s)\n', ...
    max(abs(sqrt(sum(x0s(:,1:3).^2,2))-12)), max(abs(x0s(:,8)-the))*R2D, max(abs(-x0s(:,12)-600)));
fprintf(fid, 'perturbado: VT final=%.4g  theta final=%.4g deg  phi final=%.4g deg  h final=%.4g m\n', ...
    sqrt(sum(x1s(end,1:3).^2)), x1s(end,8)*R2D, x1s(end,7)*R2D, -x1s(end,12));

%% ================= A3: PID NL x linear (3 manobras) =================
mans = {'manobra_long','long'; 'manobra_latero','latero'; 'manobra_agressiva','agressiva'};
for m = 1:3
    eval(mans{m,1});
    outNL = sim('modelo_NL_DH_CL', 'StopTime', '120');
    outL  = sim('modelo_linear_DH_CL', 'StopTime', '120', 'ReturnWorkspaceOutputs','on');

    tN = outNL.tout; YN = outNL.Y.signals.values; UN = outNL.U.signals.values;
    tL = outL.get('tout');
    YLo = outL.get('Y_long_lin').signals.values;   % [u alpha q theta h VT] (deltas)
    YLa = outL.get('Y_lat_lin').signals.values;    % [beta p r phi psi] (deltas)
    ULo = outL.get('U_long_lin').signals.values;   % [throttle elevator] (deltas)
    ULa = outL.get('U_lat_lin').signals.values;    % [aileron rudder] (deltas)

    % o modelo linear pode logar deltas OU valores absolutos (trim ja somado);
    % detecta pelo valor inicial e soma o trim apenas se vier em delta.
    offH  = 600*(abs(mean(YLo(1:5,5)))  < 300);
    offV  = 12 *(abs(mean(YLo(1:5,6)))  < 6);
    offTh = the*(abs(mean(YLo(1:5,4)))  < the/2);
    offE  = dEe*(abs(mean(ULo(1:5,2)))  < dEe/2);
    paineis = {
      % {titulo, unid, yNL, yLIN}
      {'Altitude','m',      YN(:,15),        offH + YLo(:,5)}
      {'Airspeed V_T','m/s',YN(:,1),         offV + YLo(:,6)}
      {'\theta','deg',      YN(:,9)*R2D,     (offTh + YLo(:,4))*R2D}
      {'\psi','deg',        YN(:,10)*R2D,    YLa(:,5)*R2D}
      {'\phi','deg',        YN(:,8)*R2D,     YLa(:,4)*R2D}
      {'Elevator','deg',    UN(:,2)*R2D,     (offE + ULo(:,2))*R2D}
    };
    fig = figure('Color','w'); try, set(fig,'Theme','light'); catch, end
    set(fig,'Units','normalized','Position',[0.05 0.05 0.9 0.85]);
    tl = tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
    tits = struct('long','longitudinal maneuver (h +10 m)', ...
                  'latero','lateral-directional maneuver (\psi +15 deg)', ...
                  'agressiva','aggressive combined maneuver (h +30 m and \psi +40 deg)');
    title(tl, ['Cascade PID: nonlinear vs. linear model - ' tits.(mans{m,2})], ...
        'FontWeight','bold', 'Interpreter','tex');
    for k = 1:6
        p = paineis{k};
        ax = nexttile(tl); hold(ax,'on'); grid(ax,'on');
        plot(ax, tN, p{3}, '-',  'Color',[0 0.447 0.741], 'LineWidth',1.4);
        plot(ax, tL, p{4}, '--', 'Color',[0.85 0.325 0.098], 'LineWidth',1.3);
        title(ax, p{1}); ylabel(ax, p{2}); xlim(ax,[0 120]);
        if k>=5, xlabel(ax,'Time [s]'); end
        if k==1, legend(ax, {'Nonlinear model','Linear model'}, 'Location','best'); end
    end
    exportgraphics(fig, fullfile(figdir, sprintf('fig_nl_lin_%s.png', mans{m,2})), ...
        'Resolution',300, 'BackgroundColor','white');
    close(fig);
    fprintf('manobra %s ok\n', mans{m,2});
    fprintf(fid, 'NLxLIN %s: max|dh(NL-L)|=%.3g m  max|dtheta|=%.3g deg  max|dphi|=%.3g deg\n', ...
        mans{m,2}, max(abs(interp1(tL,offH+YLo(:,5),tN,'linear','extrap')-YN(:,15))), ...
        max(abs(interp1(tL,(offTh+YLo(:,4))*R2D,tN,'linear','extrap')-YN(:,9)*R2D)), ...
        max(abs(interp1(tL,YLa(:,4)*R2D,tN,'linear','extrap')-YN(:,8)*R2D)));
end
fclose(fid);
fprintf('sims A concluidas.\n');
close all

function s = ternario(c, a, b)
    if c, s = a; else, s = b; end
end
