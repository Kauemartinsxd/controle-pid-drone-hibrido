%% sims_artigo_B — 3 cenarios de comparacao PID x LQRy em SIL (Sec. IV):
%  latero (phi doublet), longitudinal (theta doublet + VT), combinado (caso 1).
%  Mesmas referencias nos dois rigs (harness do Mirko intocado — so variaveis).
%  Saidas: Artigo\figs\fig_cen_*.png + Artigo\dados_sims_B.txt (metricas).

%% -------- PID: init + cenarios --------
run('C:\Users\kaue\Documents\PID_DH\controle-pid-drone-hibrido\DH_inicializacao.m');
raiz = 'C:\Users\kaue\Documents\PID_DH\HIL_PID\Matlab';
artigo = 'C:\Users\kaue\Documents\PID_DH\Artigo';
figdir = fullfile(artigo, 'figs');
cd(raiz); PID_H_params;
R2D = 180/pi;

% base comum (tudo desligado)
base = struct('VT_step_delta',0,'VT_step_t',5,'VT_pulse_delta',5, ...
    'VT_pulse_t',15,'VT_pulse_t2',50, ...
    'phi_step_init',0,'phi_step_final',0,'phi_step_t',1e9,'phi_step_t2',1e9,'phi_step_t3',1e9, ...
    'theta_step_init',0,'theta_step_final',0,'theta_step_t',1e9,'theta_step_t2',1e9,'theta_step_t3',1e9, ...
    'psi_ref_init',0,'psi_ref_final',0,'psi_ref_t',1e9,'psi_ref_t2',1e9,'psi_ref_t3',1e9, ...
    'h_step_init',0,'h_step_final',0,'h_step_t',1e9,'h_step_t2',1e9,'h_step_t3',1e9);

cen = struct();
cen.latero = base;
cen.latero.phi_step_final = deg2rad(5);
cen.latero.phi_step_t=50; cen.latero.phi_step_t2=100; cen.latero.phi_step_t3=150;
cen.long = base;
cen.long.VT_step_delta = 0.2;                        % 12.0 -> 12.2
cen.long.theta_step_final = deg2rad(5);
cen.long.theta_step_t=100; cen.long.theta_step_t2=150; cen.long.theta_step_t3=200;

nomes_cen = {'latero','long'};
VT_Throttle=1; phi_psi=1; att_alt=1;                 % modos: VelHold+PhiHold+ThetaHold

S_PID = struct();
for c = 1:2
    cfg = cen.(nomes_cen{c});
    fn = fieldnames(cfg);
    for k = 1:numel(fn), eval(sprintf('%s = cfg.%s;', fn{k}, fn{k})); end
    fprintf('PID cenario %s...\n', nomes_cen{c});
    out_sim = sim('CL_NL_DH_SIL_casos', 'StopTime', '300');
    S_PID.(nomes_cen{c}) = extrai_sinais_PID(out_sim);
end
P = load(fullfile(raiz,'Dados_mat_SIL_PID','Plot_caso1.mat'));
S_PID.comb = extrai_sinais_PID(P.out_sim);

%% -------- LQRy: cenarios no SIL dele (so variaveis) --------
work = fullfile(raiz, 'mirko_replica');
cd(work);
addpath(genpath('Control_NL_HIL')); addpath(genpath('Modelo_DH_HIL')); addpath(genpath('Dados_mat_SIL'));
load('Ganho_hold_theta.mat'); load('Ganho_hold_H.mat'); load('Ganho_hold_VT.mat');
load('Ganho_hold_phi.mat');   load('Ganho_hold_psi.mat'); load('Dados_Trim.mat');
Ts = 1/100; surfaces = 24; Ve = 12; he = 600; gammae = 0; R = 6371000;
coef_Ana = 1; coef_Sato = 0;
VT_Throttle=1; phi_psi=1; att_alt=1;
refPsi = 0; refAlt = 0;

S_LQR = struct();
% latero: phi doublet 5 deg, theta parado, VT em 12.0 (+ pulso fixo do harness)
refPhi = 5; reftheta = 0; refVel = 12.0;
fprintf('LQRy cenario latero...\n');
out_sim = sim('CL_NL_DH_SIL_manobras', 'StopTime', '300');
S_LQR.latero = out_sim;
% long: theta doublet 5 deg, phi/psi zero, VT 12.2
refPhi = 0; reftheta = 5; refVel = 12.2;
fprintf('LQRy cenario long...\n');
out_sim = sim('CL_NL_DH_SIL_manobras', 'StopTime', '300');
S_LQR.long = out_sim;
% comb = caso 1 dele (ja existente)
M = load(fullfile('C:\Users\kaue\Documents\PID_DH\HIL_PID\referencia_mirko\Matlab', ...
    'Dados_mat_SIL', 'Plot_caso1.mat'));
fM = fieldnames(M); S_LQR.comb = M.(fM{1});

%% -------- extracao uniforme + metricas + figuras --------
% getsig(S, nome, col): PID = struct, LQRy = SimulationOutput
getsig = @(S,nm) local_get(S,nm);

fid = fopen(fullfile(artigo,'dados_sims_B.txt'),'w');
fprintf(fid, '===== METRICAS 3 CENARIOS (SIL, refs identicas) =====\n');
fprintf(fid, 'overshoot [%%] | t_subida 10-90 [s] | t_acomod 2%% [s] | erro regime\n\n');

% janelas: {cenario, canal(nome *_NL), col_atual, col_ref, t0, t1, titulo, unidade}
JAN = {
 'latero','phi_NL',2,1, 50,100,'\phi','deg';
 'latero','VT_NL', 2,1, 15, 50,'V_T','m/s';
 'long',  'theta_NL',2,1,100,150,'\theta','deg';
 'long',  'VT_NL', 2,1, 15, 50,'V_T','m/s';
 'comb',  'phi_NL',2,1, 50,100,'\phi','deg';
 'comb',  'theta_NL',2,1,100,150,'\theta','deg';
 'comb',  'VT_NL', 2,1, 15, 50,'V_T','m/s';
};
LTX = {};
for j = 1:size(JAN,1)
    cenj = JAN{j,1};
    res = cell(1,2);
    for r = 1:2
        if r==1, S = S_PID.(cenj); else, S = S_LQR.(cenj); end
        v = local_get(S, JAN{j,2});
        t = v.time; y = v.signals.values(:,JAN{j,3}); ref = v.signals.values(:,JAN{j,4});
        res{r} = metricas_step(t, y, ref, JAN{j,5}, JAN{j,6});
    end
    fprintf(fid, '%-7s %-9s  PID: OS=%6.1f%%  tr=%5.2f  ts=%6.2f  ess=%+.3g   |   LQRy: OS=%6.1f%%  tr=%5.2f  ts=%6.2f  ess=%+.3g\n', ...
        cenj, JAN{j,7}, res{1}.os, res{1}.tr, res{1}.ts, res{1}.ess, ...
        res{2}.os, res{2}.tr, res{2}.ts, res{2}.ess);
    LTX{end+1} = sprintf('%s & $%s$ & %.1f & %.2f & %.2f & %.3g & %.1f & %.2f & %.2f & %.3g \\\\', ...
        cenj, JAN{j,7}, res{1}.os, res{1}.tr, res{1}.ts, res{1}.ess, ...
        res{2}.os, res{2}.tr, res{2}.ts, res{2}.ess); %#ok<SAGROW>
end
fprintf(fid, '\n===== LINHAS LaTeX =====\n');
for k = 1:numel(LTX), fprintf(fid, '%s\n', LTX{k}); end
fclose(fid);

% figuras: 6 paineis por cenario
tit = struct('latero','Scenario 1 - lateral-directional maneuver (\phi doublet, 5 deg)', ...
             'long','Scenario 2 - longitudinal maneuver (\theta doublet, 5 deg + V_T step)', ...
             'comb','Scenario 3 - combined maneuver (lateral + longitudinal)');
paineis = {'phi_NL',2,1,'\phi','deg'; 'theta_NL',2,1,'\theta','deg';
           'VT_NL',2,1,'Airspeed V_T','m/s';  'H_NL',2,0,'Altitude','m';
           'Throttle_NL',1,0,'Throttle','-'; 'elev_NL',1,0,'Elevator','deg'};
cRef=[0.45 0.45 0.45]; cP=[0 0.447 0.741]; cL=[0.85 0.325 0.098];
nomes3 = {'latero','long','comb'};
for c = 1:3
    fig = figure('Color','w'); try, set(fig,'Theme','light'); catch, end
    set(fig,'Units','normalized','Position',[0.05 0.05 0.9 0.85]);
    tl = tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
    title(tl, [tit.(nomes3{c}) ' - PID vs. LQRy (SIL)'], 'FontWeight','bold', 'Interpreter','tex');
    for k = 1:6
        vP = local_get(S_PID.(nomes3{c}), paineis{k,1});
        vL = local_get(S_LQR.(nomes3{c}), paineis{k,1});
        ax = nexttile(tl); hold(ax,'on'); grid(ax,'on'); leg = {};
        if paineis{k,3} > 0
            plot(ax, vP.time, vP.signals.values(:,paineis{k,3}), ':', 'Color',cRef,'LineWidth',1.1);
            leg{end+1} = 'Reference';
        end
        plot(ax, vP.time, vP.signals.values(:,paineis{k,2}), '-',  'Color',cP,'LineWidth',1.3);
        plot(ax, vL.time, vL.signals.values(:,paineis{k,2}), '--', 'Color',cL,'LineWidth',1.3);
        leg = [leg {'PID','LQRy'}];
        title(ax, paineis{k,4}); ylabel(ax, paineis{k,5}); xlim(ax,[0 300]);
        if k>=5, xlabel(ax,'Time [s]'); end
        if k==1, legend(ax, leg, 'Location','best'); end
    end
    exportgraphics(fig, fullfile(figdir, sprintf('fig_cen_%s.png', nomes3{c})), ...
        'Resolution',300,'BackgroundColor','white');
    close(fig);
end
fprintf('sims B concluidas.\n');
close all

%% -------- funcoes locais --------
function v = local_get(S, nm)
    if isstruct(S), v = S.(nm); else, v = S.get(nm); end
end

function m = metricas_step(t, y, ref, t0, t1)
    seg = t>=t0 & t<=t1; ts_ = t(seg); ys = y(seg); rs = ref(seg);
    y0 = ys(1); yf = mean(rs(ts_ > t1-5));      % alvo = ref (filtrada) no fim da janela
    dy = yf - y0;
    if abs(dy) < 1e-9, m = struct('os',0,'tr',0,'ts',0,'ess',0); return; end
    yn = (ys - y0)/dy;                          % normalizado 0->1
    i10 = find(yn >= 0.1, 1); i90 = find(yn >= 0.9, 1);
    if isempty(i10) || isempty(i90), m.tr = NaN; else, m.tr = ts_(i90)-ts_(i10); end
    m.os = max(0, (max(yn)-1)*100);
    fora = find(abs(yn-1) > 0.02, 1, 'last');
    if isempty(fora), m.ts = 0; else, m.ts = ts_(min(fora+1,numel(ts_))) - t0; end
    m.ess = mean(ys(ts_ > t1-5)) - yf;
end
