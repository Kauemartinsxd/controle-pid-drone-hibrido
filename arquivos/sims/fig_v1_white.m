%% fig_v1_white — re-exporta as figuras v1 (campanha 10/ago) em fundo branco
%  P4/P5 de arquivos/PENDENCIAS.md: mesmas curvas, cores de serie e layout
%  dos assets do deck (tema escuro); fundo branco, 300 dpi, rotulos em
%  ingles. Sobrescreve arquivos/figs com os MESMOS nomes. Tambem regenera
%  fig_eq_comb.png (P5) na missao combinada com o contexto do LQRy v1.

raiz   = 'C:\Users\kaue\Documents\PID_DH\HIL_PID\Matlab';
pR     = fullfile(raiz,'Dados_mat_ROBUSTEZ');
pC     = fullfile(raiz,'Dados_mat_comparacao');
figdir = 'C:\Users\kaue\Documents\PID_DH\controle-pid-drone-hibrido\arquivos\figs';
R2D    = 180/pi;

% cores das series (mantidas do deck)
cRef = [0.54 0.54 0.54]; cPID = [89 166 255]/255; cLQR = [255 115 77]/255;
cWN  = [242 193 78]/255; cWE  = [93 211 158]/255; cWD  = [180 142 173]/255;
cEvt = [0.55 0.55 0.55]; cLbl = [0.25 0.25 0.25]; cNota = [0.45 0.45 0.45];

%% ---------- 1) fig_v1_comb — missao combinada PID x LQRy (8 paineis) ----------
S  = load(fullfile(raiz,'Dados_mat_SIL_LQRY_devfix','Plot_caso4.mat')); o = S.out_sim;
tL = o.tout;
refVT  = o.VT_NL.signals.values(:,1);
refPSI = o.psi_NL.signals.values(:,2);
refH   = o.H_NL.signals.values(:,1);
Lq = struct('t',tL, 'VT',o.VT_NL.signals.values(:,2), 'psi',o.psi_NL.signals.values(:,1), ...
    'h',o.H_NL.signals.values(:,2), 'th',o.theta_NL.signals.values(:,2), ...
    'phi',o.phi_NL.signals.values(:,2), 'elev',o.elev_NL.signals.values(:,1), ...
    'thr',o.Throttle_NL.signals.values(:,1), 'ail',o.ail_NL.signals.values(:,1));
Pm = load(fullfile(pC,'pid_missao.mat')); Pm = Pm.P;
Pd = struct('t',Pm.t, 'VT',Pm.VT, 'psi',Pm.psi*R2D, 'h',Pm.h, 'th',Pm.theta*R2D, ...
    'phi',Pm.phi*R2D, 'elev',interp1(Pm.tU,Pm.U(:,2),Pm.t)*R2D, ...
    'thr',interp1(Pm.tU,Pm.U(:,1),Pm.t), 'ail',interp1(Pm.tU,Pm.U(:,3),Pm.t)*R2D);

paineis = {
 'VT',  'V_T [m/s]',    refVT,  5,            {'V_T: 12 \rightarrow 15.2 m/s'}, ''
 'phi', '\phi [deg]',   [],     [],           {}, 'no direct command — banks to track \psi'
 'psi', '\psi [deg]',   refPSI, [50 100 150], {'\psi_{ref} +5\circ','\psi_{ref} -5\circ','\psi_{ref} 0\circ'}, ''
 'elev','elevator [deg]', [],   [],           {}, ''
 'h',   'h [m]',        refH,   [80 160 240], {'h_{ref} +5 m','h_{ref} -5 m','h_{ref} 600 m'}, ''
 'thr', 'throttle [-]', [],     [],           {}, ''
 'th',  '\theta [deg]', [],     [],           {}, 'no direct command — responds to V_T and h'
 'ail', 'aileron [deg]',[],     [],           {}, ''
};
fig = figure('Color','w','Units','pixels','Position',[40 40 1500 950],'Visible','off');
try, fig.Theme = 'light'; catch, end
tl  = tiledlayout(fig,4,2,'TileSpacing','compact','Padding','compact');
for p = 1:size(paineis,1)
    ax = nexttile(tl); hold(ax,'on'); grid(ax,'on'); set(ax,'FontSize',10.5);
    temref = ~isempty(paineis{p,3});
    if temref
        plot(ax, tL, paineis{p,3}, '--', 'Color', cRef, 'LineWidth', 1.2);
    end
    plot(ax, Pd.t, Pd.(paineis{p,1}), '-', 'Color', cPID, 'LineWidth', 1.4);
    plot(ax, Lq.t, Lq.(paineis{p,1}), '-', 'Color', cLQR, 'LineWidth', 1.4);
    ylabel(ax, paineis{p,2}); xlim(ax,[0 300]);
    if p >= 7, xlabel(ax,'Time [s]'); end
    yl = ylim(ax);
    ev = paineis{p,4}; rot = paineis{p,5};
    for i = 1:numel(ev)
        xline(ax, ev(i), ':', 'Color', cEvt, 'LineWidth', 1.0);
        text(ax, ev(i)+3, yl(2)-0.07*diff(yl), rot{i}, ...
            'Color', cLbl, 'FontSize', 9, 'VerticalAlignment','top');
    end
    ylim(ax, yl);
    if ~isempty(paineis{p,6})
        text(ax, 297, yl(1)+0.08*diff(yl), paineis{p,6}, 'Color', cNota, ...
            'FontSize', 8.5, 'FontAngle','italic', 'HorizontalAlignment','right');
    end
    if p == 1
        legend(ax, {'ref','PID','LQRy'}, 'Location','east','FontSize',9);
    end
end
exportgraphics(fig, fullfile(figdir,'fig_v1_comb.png'), 'Resolution',300,'BackgroundColor','white');
close(fig); fprintf('fig_v1_comb.png ok.\n');

%% ---------- 2) robustez: sato / satoL (4 paineis) ----------
sinais = @(X) struct('t',X.VT_NL.time, 'vt',X.VT_NL.signals.values, ...
    'ps',X.psi_NL.signals.values, 'hh',X.H_NL.signals.values, ...
    'th',X.theta_NL.signals.values);
figcfg = {'sato','fig_v1_rob_sato.png'; 'satoL','fig_v1_rob_satoL.png'};
for f = 1:2
    cen = figcfg{f,1};
    P = load(fullfile(pR,sprintf('PID_%s.mat',cen)));  A = sinais(P.S);
    L = load(fullfile(pR,sprintf('LQR_%s.mat',cen)));  B = sinais(L.out_sim);
    fig = figure('Color','w','Units','pixels','Position',[50 50 1100 970],'Visible','off');
    try, fig.Theme = 'light'; catch, end
    tl = tiledlayout(fig,4,1,'TileSpacing','compact','Padding','compact');
    sn = {'vt',2,1,'V_T [m/s]'; 'ps',1,2,'\psi [deg]'; 'hh',2,1,'h [m]'; 'th',2,0,'\theta [deg]'};
    for p = 1:4
        ax = nexttile(tl); hold(ax,'on'); grid(ax,'on'); set(ax,'FontSize',11);
        fld = sn{p,1}; col = sn{p,2}; rcol = sn{p,3};
        if rcol > 0
            plot(ax, A.t, A.(fld)(:,rcol), '--', 'Color', cRef, 'LineWidth', 1.2);
        end
        plot(ax, A.t, A.(fld)(:,col), '-', 'Color', cPID, 'LineWidth', 1.5);
        plot(ax, B.t, B.(fld)(:,col), '-', 'Color', cLQR, 'LineWidth', 1.5);
        ylabel(ax, sn{p,4}); xlim(ax,[0 300]);
        if p == 1, legend(ax,{'ref','PID','LQRy'},'Location','east'); end
        if p == 4, xlabel(ax,'Time [s]'); end
    end
    exportgraphics(fig, fullfile(figdir,figcfg{f,2}), 'Resolution',300,'BackgroundColor','white');
    close(fig); fprintf('%s ok.\n', figcfg{f,2});
end

%% ---------- 3) robustez: vento (perfil + 4 paineis) ----------
P = load(fullfile(pR,'PID_vento.mat'));  A = sinais(P.S);
L = load(fullfile(pR,'LQR_vento.mat'));  B = sinais(L.out_sim);
V = load(fullfile(pR,'perfil_vento.mat'));
fig = figure('Color','w','Units','pixels','Position',[50 50 1100 1150],'Visible','off');
try, fig.Theme = 'light'; catch, end
tl = tiledlayout(fig,5,1,'TileSpacing','compact','Padding','compact');
ax = nexttile(tl); hold(ax,'on'); grid(ax,'on'); set(ax,'FontSize',11);
plot(ax,V.t_w,V.W_w(:,1),'-','Color',cWN,'LineWidth',1.5);
plot(ax,V.t_w,V.W_w(:,2),'-','Color',cWE,'LineWidth',1.5);
plot(ax,V.t_w,V.W_w(:,3),'-','Color',cWD,'LineWidth',1.5);
ylabel(ax,'wind [m/s]'); xlim(ax,[0 300]); ylim(ax,[-4 5]);
legend(ax,{'W_N (tailwind)','W_E (gusts)','W_D (gusts)'},'Location','east');
sn = {'vt',2,1,'V_T [m/s]'; 'ps',1,2,'\psi [deg]'; 'hh',2,1,'h [m]'; 'th',2,0,'\theta [deg]'};
for p = 1:4
    ax = nexttile(tl); hold(ax,'on'); grid(ax,'on'); set(ax,'FontSize',11);
    fld = sn{p,1}; col = sn{p,2}; rcol = sn{p,3};
    if rcol > 0
        plot(ax, A.t, A.(fld)(:,rcol), '--', 'Color', cRef, 'LineWidth', 1.2);
    end
    plot(ax, A.t, A.(fld)(:,col), '-', 'Color', cPID, 'LineWidth', 1.5);
    plot(ax, B.t, B.(fld)(:,col), '-', 'Color', cLQR, 'LineWidth', 1.5);
    ylabel(ax, sn{p,4}); xlim(ax,[0 300]);
    if p == 1, legend(ax,{'ref','PID','LQRy'},'Location','east'); end
    if p == 4, xlabel(ax,'Time [s]'); end
end
exportgraphics(fig, fullfile(figdir,'fig_v1_rob_vento.png'), 'Resolution',300,'BackgroundColor','white');
close(fig); fprintf('fig_v1_rob_vento.png ok.\n');

%% ---------- 4) fig_v1_acopla — varredura de acoplamento (so PID) ----------
c5 = [93 211 158]/255; c15 = [242 193 78]/255; c30 = [89 166 255]/255; cAH = [89 166 255]/255;
E = load(fullfile(raiz,'Dados_mat_experimentos','estudo_acoplamento_roll.mat'));
pex = @(o) struct('t', o.Y.time, 'phi', o.Y.signals.values(:,8)*R2D, ...
    'th', o.Y.signals.values(:,9)*R2D, 'h', o.Y.signals.values(:,15), ...
    'el', interp1(o.U.time, o.U.signals.values(:,2), o.Y.time)*R2D);
P5  = pex(E.rodadas(1).out); P15 = pex(E.rodadas(2).out); P30 = pex(E.rodadas(3).out);
Pb  = pex(E.rodadas(5).out); PA  = pex(E.rodadas(6).out);
pre = @(S,f) mean(S.(f)(S.t < 19));
d5  = struct('th',P5.th -pre(P5,'th'),  'h',P5.h -pre(P5,'h'),  'el',P5.el -pre(P5,'el'));
d15 = struct('th',P15.th-pre(P15,'th'), 'h',P15.h-pre(P15,'h'), 'el',P15.el-pre(P15,'el'));
d30 = struct('th',P30.th-pre(P30,'th'), 'h',P30.h-pre(P30,'h'), 'el',P30.el-pre(P30,'el'));
dA  = struct('th',PA.th-interp1(Pb.t,Pb.th,PA.t), 'h',PA.h-interp1(Pb.t,Pb.h,PA.t), ...
             'el',PA.el-interp1(Pb.t,Pb.el,PA.t));
fig = figure('Color','w','Units','pixels','Position',[40 40 1500 900],'Visible','off');
try, fig.Theme = 'light'; catch, end
tl = tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
paineis = {
  '\phi [deg]',            {P5.t, P5.phi; P15.t, P15.phi; P30.t, P30.phi; PA.t, PA.phi};
  '\Delta\theta [deg]',    {P5.t, d5.th;  P15.t, d15.th;  P30.t, d30.th;  PA.t, dA.th};
  '\Delta h [m]',          {P5.t, d5.h;   P15.t, d15.h;   P30.t, d30.h;   PA.t, dA.h};
  '\Delta elevator [deg]', {P5.t, d5.el;  P15.t, d15.el;  P30.t, d30.el;  PA.t, dA.el};
};
estilos = {c5,'-'; c15,'-'; c30,'-'; cAH,'--'};
for p = 1:4
    ax = nexttile(tl); hold(ax,'on'); grid(ax,'on'); set(ax,'FontSize',11);
    for s = 1:4
        plot(ax, paineis{p,2}{s,1}, paineis{p,2}{s,2}, estilos{s,2}, ...
            'Color', estilos{s,1}, 'LineWidth', 1.5);
    end
    ylabel(ax, paineis{p,1}); xlim(ax,[0 200]);
    if p >= 3, xlabel(ax,'Time [s]'); end
    if p == 1
        legend(ax, {'\phi = 5\circ (ThetaHold)','\phi = 15\circ (ThetaHold)', ...
            '\phi = 30\circ (ThetaHold)','\phi = 30\circ (AltitudeHold, 15.2 m/s)'}, ...
            'Location','southeast','FontSize',9);
    end
end
exportgraphics(fig, fullfile(figdir,'fig_v1_acopla.png'), 'Resolution',300,'BackgroundColor','white');
close(fig); fprintf('fig_v1_acopla.png ok.\n');

%% ---------- 5) fig_eq_comb — ablacao de equalizacao, missao combinada ----------
% PID como entregue (PID_nom) x PID equalizado (pid2_comb_equiparado) com o
% LQRy v1 (LQR_nom) como contexto; ref = degraus crus do rig LQRy.
cAsis = [0 0.4470 0.7410]; cEq = [0.4660 0.6740 0.1880]; cCtx = [0.8500 0.3250 0.0980];
Pa = load(fullfile(pR,'PID_nom.mat'));            A = sinais(Pa.S);
Pe = load(fullfile(pC,'pid2_comb_equiparado.mat')); Bq = sinais(Pe.S);
Lc = load(fullfile(pR,'LQR_nom.mat'));            C = sinais(Lc.out_sim);
fig = figure('Color','w','Units','pixels','Position',[60 60 760 640],'Visible','off');
try, fig.Theme = 'light'; catch, end
tl = tiledlayout(fig,3,1,'TileSpacing','compact','Padding','compact');
sn = {'vt',2,1,'V_T [m/s]'; 'ps',1,2,'\psi [deg]'; 'hh',2,1,'h [m]'};
for p = 1:3
    ax = nexttile(tl); hold(ax,'on'); grid(ax,'on'); set(ax,'FontSize',10);
    fld = sn{p,1}; col = sn{p,2}; rcol = sn{p,3};
    plot(ax, C.t, C.(fld)(:,rcol), ':', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.1);
    plot(ax, A.t, A.(fld)(:,col),  '-', 'Color', cAsis, 'LineWidth', 1.4);
    plot(ax, Bq.t, Bq.(fld)(:,col),'-', 'Color', cEq,   'LineWidth', 1.4);
    plot(ax, C.t, C.(fld)(:,col),  '--','Color', cCtx,  'LineWidth', 1.2);
    ylabel(ax, sn{p,4}); xlim(ax,[0 300]);
    if p == 1
        legend(ax, {'ref','PID (as delivered)','PID (equalized)','LQRy (context)'}, ...
            'Location','east','FontSize',8);
    end
    if p == 3, xlabel(ax,'Time [s]'); end
end
exportgraphics(fig, fullfile(figdir,'fig_eq_comb.png'), 'Resolution',300,'BackgroundColor','white');
close(fig); fprintf('fig_eq_comb.png ok.\n');
fprintf('>>> figuras v1 em fundo branco prontas.\n');
