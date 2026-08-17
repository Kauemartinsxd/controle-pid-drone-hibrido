%% sims_artigo_C — regera (sem re-simular) as figuras de HIL do artigo, em
%  ingles e com o enquadramento neutro: determinismo, PID SIL x HIL e
%  comparacao PID x LQRy na mesma bancada.

raiz   = 'C:\Users\kaue\Documents\PID_DH\HIL_PID\Matlab';
mirko  = 'C:\Users\kaue\Documents\PID_DH\HIL_PID\referencia_mirko\Matlab';
figdir = 'C:\Users\kaue\Documents\PID_DH\Artigo\figs';
addpath(raiz); cd(raiz);
R2D = 180/pi;

cA = [0 0.4470 0.7410]; cB = [0.8500 0.3250 0.0980]; cR = [0.45 0.45 0.45];

paineis = {'theta_NL',2,1,'\theta','deg'; 'VT_NL',2,1,'Airspeed V_T','m/s';
           'H_NL',2,1,'Altitude','m';     'phi_NL',2,1,'\phi','deg';
           'elev_NL',1,0,'Elevator','deg';'Throttle_NL',1,0,'Throttle','-'};

%% ---------- 1. determinismo: mesmo codigo, dois MCUs ----------
A = load(fullfile(mirko,'Dados_mat_HIL','out_caso_HIL.mat'));  o1 = A.out_caso_1_HIL;
B = load(fullfile(raiz,'mirko_replica','out_replica_caso1.mat')); o2 = B.out_replica;

fig = figure('Color','w'); try, set(fig,'Theme','light'); catch, end
set(fig,'Units','normalized','Position',[0.05 0.05 0.9 0.85]);
tl = tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
title(tl, ['Bit-exact reproducibility: same generated code on two ' ...
    'microcontroller models (flight case 1)'], 'FontWeight','bold');
for k = 1:6
    v1 = o1.get(paineis{k,1}); v2 = o2.get(paineis{k,1});
    ax = painel(tl, paineis{k,4}, paineis{k,5});
    plot(ax, v1.time, v1.signals.values(:,paineis{k,2}), '-',  'Color',cA,'LineWidth',1.6);
    plot(ax, v2.time, v2.signals.values(:,paineis{k,2}), '--', 'Color',cB,'LineWidth',1.2);
    if k>=5, xlabel(ax,'Time [s]'); end
    if k==1, legend(ax, {'Execution A (STM32F411)','Execution B (STM32F303)'}, 'Location','best'); end
end
exportgraphics(fig, fullfile(figdir,'replica_caso1.png'), 'Resolution',300,'BackgroundColor','white');
close(fig);

%% ---------- 2. PID: SIL x HIL (caso 1) ----------
S = load(fullfile(raiz,'Dados_mat_SIL_PID','Plot_caso1.mat'));
H = load(fullfile(raiz,'Dados_mat_HIL_PID','out_caso_1_HIL_PID.mat'));
fH = fieldnames(H);
sS = extrai_sinais_PID(S.out_sim);  sH = extrai_sinais_PID(H.(fH{1}));

fig = figure('Color','w'); try, set(fig,'Theme','light'); catch, end
set(fig,'Units','normalized','Position',[0.05 0.05 0.9 0.85]);
tl = tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
title(tl, 'Cascade PID, flight case 1: SIL vs. HIL on the nonlinear model', ...
    'FontWeight','bold');
for k = 1:6
    v1 = sS.(paineis{k,1}); v2 = sH.(paineis{k,1});
    ax = painel(tl, paineis{k,4}, paineis{k,5});
    if paineis{k,3} > 0
        plot(ax, v1.time, v1.signals.values(:,paineis{k,3}), ':', 'Color',cR,'LineWidth',1.0);
    end
    plot(ax, v1.time, v1.signals.values(:,paineis{k,2}), '-',  'Color',cA,'LineWidth',1.6);
    plot(ax, v2.time, v2.signals.values(:,paineis{k,2}), '--', 'Color',cB,'LineWidth',1.2);
    if k>=5, xlabel(ax,'Time [s]'); end
    if k==1
        if paineis{k,3} > 0
            legend(ax, {'Reference','SIL','HIL'}, 'Location','best');
        else
            legend(ax, {'SIL','HIL'}, 'Location','best');
        end
    end
end
exportgraphics(fig, fullfile(figdir,'pid_sil_hil_caso1.png'), 'Resolution',300,'BackgroundColor','white');
close(fig);

%% ---------- 3. PID x LQRy na mesma bancada (caso 1) ----------
L = load(fullfile(raiz,'Dados_mat_HIL_LQRY_corrigido','out_caso_1_HIL_LQRY.mat'));
oL = L.out_sim;

fig = figure('Color','w'); try, set(fig,'Theme','light'); catch, end
set(fig,'Units','normalized','Position',[0.05 0.05 0.9 0.85]);
tl = tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
title(tl, ['Embedded comparison, flight case 1: cascade PID and LQRy ' ...
    'under identical conditions'], 'FontWeight','bold');
for k = 1:6
    vP = sH.(paineis{k,1});  vL = oL.get(paineis{k,1});
    ax = painel(tl, paineis{k,4}, paineis{k,5});
    leg = {};
    if paineis{k,3} > 0
        plot(ax, vP.time, vP.signals.values(:,paineis{k,3}), ':', 'Color',cR,'LineWidth',1.0);
        leg{end+1} = 'Reference';
    end
    plot(ax, vP.time, vP.signals.values(:,paineis{k,2}), '-',  'Color',cA,'LineWidth',1.4);
    plot(ax, vL.time, vL.signals.values(:,paineis{k,2}), '--', 'Color',cB,'LineWidth',1.4);
    leg = [leg {'PID (HIL)','LQRy (HIL)'}];
    if k>=5, xlabel(ax,'Time [s]'); end
    if k==1, legend(ax, leg, 'Location','best'); end
end
exportgraphics(fig, fullfile(figdir,'justo_caso1.png'), 'Resolution',300,'BackgroundColor','white');
close(fig);

fprintf('figuras C regeradas em ingles.\n');
close all

%% ---------- funcao local ----------
function ax = painel(tl, tit, ylab)
    ax = nexttile(tl); hold(ax,'on'); grid(ax,'on');
    title(ax, tit); ylabel(ax, ylab); xlim(ax,[0 300]);
end
