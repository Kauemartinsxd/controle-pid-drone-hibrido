%% sims_artigo_E — figuras do artigo (robustez, altitude única, acoplamento)
%  Estilo paper: fundo branco, 300 dpi, labels em inglês -> Artigo\figs\
%  Fontes de dados (tudo já simulado; este script só replota):
%    Dados_mat_ROBUSTEZ\{PID,LQR}_{nom,sato,satoL,pert,vento}.mat + perfil_vento.mat
%    Dados_mat_experimentos\estudo_acoplamento_roll.mat (PID) e _lqry.mat (LQRy)
%  Saídas: fig_alt_pid_lqry, fig_rob_{sato,satoL,pert,vento}, fig_acoplamento
%          + dados_sims_E.txt (métricas, incl. análise do elevador sólida x tracejada)

raiz   = 'C:\Users\kaue\Documents\PID_DH\HIL_PID\Matlab';
figdir = 'C:\Users\kaue\Documents\PID_DH\Artigo\figs';
txtout = 'C:\Users\kaue\Documents\PID_DH\Artigo\dados_sims_E.txt';
pastaR = fullfile(raiz,'Dados_mat_ROBUSTEZ');
pastaE = fullfile(raiz,'Dados_mat_experimentos');
cd(raiz);

cRef = [0.45 0.45 0.45];
cPID = [0 0.4470 0.7410];
cLQR = [0.8500 0.3250 0.0980];
LW = 1.4;

cen = {'nom','sato','satoL','pert','vento'};
D = struct();
for c = 1:numel(cen)
    p = load(fullfile(pastaR, sprintf('PID_%s.mat', cen{c}))); D.PID.(cen{c}) = p.S;
    l = load(fullfile(pastaR, sprintf('LQR_%s.mat', cen{c}))); D.LQR.(cen{c}) = l.out_sim;
end
V = load(fullfile(pastaR,'perfil_vento.mat'));
fid = fopen(txtout,'w');
fprintf(fid,'===== sims_artigo_E — %s =====\n\n', char(datetime('now')));

%% ---------- 1) Altitude única: ref x PID x LQRy (missão nominal) ----------
[tP,~,~,hhP] = sinais(D.PID.nom);
[tL,~,~,hhL] = sinais(D.LQR.nom);
fig = fig_paper([760 340]);
ax = axes(fig); hold(ax,'on'); grid(ax,'on');
plot(ax, tP, hhP(:,1), '--', 'Color', cRef, 'LineWidth', 1.1);
plot(ax, tP, hhP(:,2), '-',  'Color', cPID, 'LineWidth', LW);
plot(ax, tL, hhL(:,2), '-',  'Color', cLQR, 'LineWidth', LW);
xlabel(ax,'Time [s]'); ylabel(ax,'Altitude h [m]');
xlim(ax,[0 300]); ylim(ax,[594 606]);
legend(ax, {'Reference','PID','LQRy'}, 'Location','northeast');
salva(fig, fullfile(figdir,'fig_alt_pid_lqry.png'));

%% ---------- 2-5) Robustez: 4 figuras em tema claro ----------
% sato/satoL/pert: 4 painéis (VT, psi, h, theta); vento: 5 (perfil + 4)
rob = {'sato','fig_rob_sato.png'; 'satoL','fig_rob_satoL.png'; ...
       'pert','fig_rob_pert.png'};
for f = 1:size(rob,1)
    [tP,vtP,psP,hhP,thP] = sinais(D.PID.(rob{f,1}));
    [tL,vtL,psL,hhL,thL] = sinais(D.LQR.(rob{f,1}));
    fig = fig_paper([760 760]);
    tl = tiledlayout(fig,4,1,'TileSpacing','compact','Padding','compact');
    ax = eixo(tl);
    plot(ax,tP,vtP(:,1),'--','Color',cRef,'LineWidth',1.1);
    plot(ax,tP,vtP(:,2),'-','Color',cPID,'LineWidth',LW);
    plot(ax,tL,vtL(:,2),'-','Color',cLQR,'LineWidth',LW);
    ylabel(ax,'V_T [m/s]'); xlim(ax,[0 300]);
    legend(ax,{'Reference','PID','LQRy'},'Location','best');
    ax = eixo(tl);
    plot(ax,tP,psP(:,2),'--','Color',cRef,'LineWidth',1.1);
    plot(ax,tP,psP(:,1),'-','Color',cPID,'LineWidth',LW);
    plot(ax,tL,psL(:,1),'-','Color',cLQR,'LineWidth',LW);
    ylabel(ax,'\psi [deg]'); xlim(ax,[0 300]);
    ax = eixo(tl);
    plot(ax,tP,hhP(:,1),'--','Color',cRef,'LineWidth',1.1);
    plot(ax,tP,hhP(:,2),'-','Color',cPID,'LineWidth',LW);
    plot(ax,tL,hhL(:,2),'-','Color',cLQR,'LineWidth',LW);
    ylabel(ax,'h [m]'); xlim(ax,[0 300]);
    ax = eixo(tl);
    plot(ax,tP,thP(:,2),'-','Color',cPID,'LineWidth',LW);
    plot(ax,tL,thL(:,2),'-','Color',cLQR,'LineWidth',LW);
    ylabel(ax,'\theta [deg]'); xlabel(ax,'Time [s]'); xlim(ax,[0 300]);
    salva(fig, fullfile(figdir,rob{f,2}));
end

% vento: perfil no topo
[tP,vtP,psP,hhP,thP] = sinais(D.PID.vento);
[tL,vtL,psL,hhL,thL] = sinais(D.LQR.vento);
fig = fig_paper([760 900]);
tl = tiledlayout(fig,5,1,'TileSpacing','compact','Padding','compact');
ax = eixo(tl);
plot(ax,V.t_w,V.W_w(:,1),'-','LineWidth',LW);
plot(ax,V.t_w,V.W_w(:,2),'-','LineWidth',LW);
plot(ax,V.t_w,V.W_w(:,3),'-','LineWidth',LW);
ylabel(ax,'Wind [m/s]'); xlim(ax,[0 300]); ylim(ax,[-4 5]);
legend(ax,{'w_N (tailwind)','w_E (lateral gusts)','w_D (vertical gusts)'},'Location','east');
ax = eixo(tl);
plot(ax,tP,vtP(:,1),'--','Color',cRef,'LineWidth',1.1);
plot(ax,tP,vtP(:,2),'-','Color',cPID,'LineWidth',LW);
plot(ax,tL,vtL(:,2),'-','Color',cLQR,'LineWidth',LW);
ylabel(ax,'V_T [m/s]'); xlim(ax,[0 300]);
legend(ax,{'Reference','PID','LQRy'},'Location','best');
ax = eixo(tl);
plot(ax,tP,psP(:,2),'--','Color',cRef,'LineWidth',1.1);
plot(ax,tP,psP(:,1),'-','Color',cPID,'LineWidth',LW);
plot(ax,tL,psL(:,1),'-','Color',cLQR,'LineWidth',LW);
ylabel(ax,'\psi [deg]'); xlim(ax,[0 300]);
ax = eixo(tl);
plot(ax,tP,hhP(:,1),'--','Color',cRef,'LineWidth',1.1);
plot(ax,tP,hhP(:,2),'-','Color',cPID,'LineWidth',LW);
plot(ax,tL,hhL(:,2),'-','Color',cLQR,'LineWidth',LW);
ylabel(ax,'h [m]'); xlim(ax,[0 300]);
ax = eixo(tl);
plot(ax,tP,thP(:,2),'-','Color',cPID,'LineWidth',LW);
plot(ax,tL,thL(:,2),'-','Color',cLQR,'LineWidth',LW);
ylabel(ax,'\theta [deg]'); xlabel(ax,'Time [s]'); xlim(ax,[0 300]);
salva(fig, fullfile(figdir,'fig_rob_vento.png'));

%% ---------- 6) Acoplamento com phi_ref explícito (inglês) ----------
P = load(fullfile(pastaE,'estudo_acoplamento_roll.mat'));    % rodadas (PID)
L = load(fullfile(pastaE,'estudo_acoplamento_lqry.mat'));    % rodadas_lqry
R2D = 180/pi; t_step = 20; T_end = 200;

getP = @(o) struct('t',o.Y.time, 'phi',o.Y.signals.values(:,8)*R2D, ...
    'th',o.Y.signals.values(:,9)*R2D, 'h',o.Y.signals.values(:,15), ...
    'el',o.U.signals.values(:,2)*R2D);
getL = @(o) struct('t',o.phi_NL.time, 'phi',o.phi_NL.signals.values(:,2), ...
    'th',o.theta_NL.signals.values(:,2), 'h',o.H_NL.signals.values(:,2), ...
    'el',o.elev_NL.signals.values(:,1));

sP_th = getP(P.rodadas(3).out);
bP_th = struct('t',sP_th.t, 'phi',zeros(size(sP_th.t)), ...
    'th',ones(size(sP_th.t))*mean(sP_th.th(sP_th.t<t_step-2)), ...
    'h', ones(size(sP_th.t))*mean(sP_th.h (sP_th.t<t_step-2)), ...
    'el',ones(size(sP_th.t))*mean(sP_th.el(sP_th.t<t_step-2)));
sP_ah = getP(P.rodadas(6).out);   bP_ah = getP(P.rodadas(5).out);
sL_th = getL(L.rodadas_lqry(4).out); bL_th = getL(L.rodadas_lqry(1).out);
sL_ah = getL(L.rodadas_lqry(8).out); bL_ah = getL(L.rodadas_lqry(7).out);

% phi_ref (pré-filtrada) da rodada LQRy TH30 — col 1 do phi_NL
phiref_t = L.rodadas_lqry(4).out.phi_NL.time;
phiref   = L.rodadas_lqry(4).out.phi_NL.signals.values(:,1);

pares = {
  {sP_th, bP_th, cPID, '-',  'PID (ThetaHold, trim)'}
  {sL_th, bL_th, cLQR, '-',  'LQRy (ThetaHold, trim)'}
  {sP_ah, bP_ah, cPID, '--', 'PID (AltitudeHold, 15.2 m/s)'}
  {sL_ah, bL_ah, cLQR, '--', 'LQRy (AltitudeHold, 15.2 m/s)'}
};
paineis = {'\phi [deg]','\Delta\theta [deg]','\Delta h [m]','\Delta elevator [deg]'};
campos  = {'phi','th','h','el'};

fig = fig_paper([900 700]);
tl = tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
for p = 1:4
    ax = eixo(tl);
    if p == 1
        plot(ax, phiref_t, phiref, ':', 'Color', cRef, 'LineWidth', 1.6);
    end
    for q = 1:4
        s = pares{q}{1}; b = pares{q}{2};
        y = s.(campos{p});
        if p > 1
            y = y - interp1(b.t, b.(campos{p}), s.t, 'linear', 'extrap');
        end
        plot(ax, s.t, y, pares{q}{4}, 'Color', pares{q}{3}, 'LineWidth', LW);
    end
    ylabel(ax, paineis{p}); xlim(ax, [0 T_end]);
    if p >= 3, xlabel(ax,'Time [s]'); end
    if p == 1
        legend(ax, [{'\phi^{ref} (prefiltered)'}, ...
            cellfun(@(c) c{5}, pares, 'UniformOutput', false)'], ...
            'Location','southeast', 'FontSize', 8);
    end
end
salva(fig, fullfile(figdir,'fig_acoplamento.png'));

%% ---------- 7) Item 6 da reunião: elevador sólida (trim) > tracejada (15,2) ----------
% Steady state do Delta elevador (últimos 20 s) por curva + previsão por pressão
% dinâmica: para o mesmo banco phi, DeltaL = (n-1)W; Delta_alpha ~ 1/qbar -> a
% razão dos incrementos entre 15,2 e 12 m/s deve ~ (12/15.2)^2.
ss = @(s,b) mean( s.el(s.t>T_end-20) - interp1(b.t,b.el,s.t(s.t>T_end-20),'linear','extrap') );
ssP_th = ss(sP_th,bP_th); ssP_ah = ss(sP_ah,bP_ah);
ssL_th = ss(sL_th,bL_th); ssL_ah = ss(sL_ah,bL_ah);
qratio = (12/15.2)^2;
fprintf(fid,'--- Acoplamento: Delta elevador em regime (últimos 20 s) ---\n');
fprintf(fid,'PID  ThetaHold@12: %.3f deg | AltitudeHold@15.2: %.3f deg | razão %.3f\n', ...
    ssP_th, ssP_ah, ssP_ah/ssP_th);
fprintf(fid,'LQRy ThetaHold@12: %.3f deg | AltitudeHold@15.2: %.3f deg | razão %.3f\n', ...
    ssL_th, ssL_ah, ssL_ah/ssL_th);
fprintf(fid,'Previsão por pressão dinâmica (12/15.2)^2 = %.3f\n', qratio);
fprintf(fid,['Leitura: em 15.2 m/s a mesma sustentação extra da curva (n=1/cos30==1.155)\n' ...
    'exige menos alpha (e menos elevador) porque qbar é %.0f%% maior.\n\n'], 100*(1/qratio-1));

% dip de theta por amplitude (escala 1-cos phi) — PID ThetaHold 5/15/30
for k = [1 2 3]   % rodadas ThetaHold 5/15/30 do PID (1..3 em estudo_acoplamento_roll)
    s = getP(P.rodadas(k).out);
    b = struct('t',s.t,'th',ones(size(s.t))*mean(s.th(s.t<t_step-2)));
    dth = s.th - b.th;
    j = s.t>t_step & s.t<t_step+30;
    fprintf(fid,'PID ThetaHold amp=%2d deg: dip dtheta = %+.4f deg (1-cos = %.4f)\n', ...
        P.rodadas(k).amp, min(dth(j)), 1-cosd(P.rodadas(k).amp));
end
fclose(fid);
type(txtout);
fprintf('>>> sims_artigo_E: figuras e métricas prontas.\n');

%% ---------- funções ----------
function [t, vt, ps, hh, th] = sinais(X)
    vt = X.VT_NL.signals.values;    % [ref atual]
    ps = X.psi_NL.signals.values;   % [atual ref]
    hh = X.H_NL.signals.values;     % [ref atual]
    th = X.theta_NL.signals.values; % [ref atual]
    t  = X.VT_NL.time;
end
function fig = fig_paper(sz)
    fig = figure('Color','w','Units','pixels','Position',[60 60 sz]);
    try, set(fig,'Theme','light'); catch, end
end
function ax = eixo(tl)
    ax = nexttile(tl); hold(ax,'on'); grid(ax,'on');
    set(ax,'FontSize',10,'LineWidth',0.8);
end
function salva(fig, arq)
    exportgraphics(fig, arq, 'Resolution', 300, 'BackgroundColor','white');
    close(fig); fprintf('%s ok.\n', arq);
end
