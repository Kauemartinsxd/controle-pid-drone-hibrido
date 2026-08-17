%% acoplamento_v2 — estudo de acoplamento com PID retunado x LQRy v2
%  Fase A: PID — roda estudo_acoplamento_roll.m (self-contained, retunado via repo)
%  Fase B: LQRy v2 — rig CL_NL_DH_v2_acopla (copia do v2_rob: phi step unico @20 s)
%  Saida: figs\fig_acoplamento.png + Artigo\dados_acoplamento_v2.txt
raiz = 'C:\Users\kaue\Documents\PID_DH\HIL_PID\Matlab';
figdir = 'C:\Users\kaue\Documents\PID_DH\Artigo\figs';
txt = 'C:\Users\kaue\Documents\PID_DH\Artigo\dados_acoplamento_v2.txt';

%% fase A — PID (6 rodadas, 200 s cada)
run(fullfile(raiz,'estudo_acoplamento_roll.m'));   % salva Dados_mat_experimentos\estudo_acoplamento_roll.mat
bdclose all;

%% fase B — LQRy v2
pid_root = 'C:\Users\kaue\Documents\PID_DH\controle-pid-drone-hibrido';
pp = strsplit(path, pathsep);
for k = find(startsWith(pp, pid_root)), rmpath(pp{k}); end
work = fullfile(raiz,'mirko_v2');
pm = strsplit(path, pathsep);
for k = find(startsWith(pm, raiz)), rmpath(pm{k}); end
addpath(work); cd(raiz);
load(fullfile(work,'Ganho_hold_theta.mat')); load(fullfile(work,'Ganho_hold_H.mat'));
load(fullfile(work,'Ganho_hold_VT.mat')); load(fullfile(work,'Ganho_hold_phi.mat'));
load(fullfile(work,'Ganho_hold_psi.mat')); load(fullfile(work,'Dados_Trim.mat'));
i = 2; Xe = Plantas(i).Xe; Ue = Plantas(i).Ue;
Ts = 1/100; surfaces = 24; Ve = Plantas(i).Ve; he = Plantas(i).He; gammae = 0; R = 6371000;
Variacao_Iner = 0; coef_Ana = 1; coef_Sato = 0;
wind_ts = timeseries(zeros(2,3), [0;300]);

rig = 'CL_NL_DH_v2_acopla';
bdclose(rig);
copyfile(fullfile(work,'CL_NL_DH_v2_rob.slx'), fullfile(work,[rig '.slx']), 'f');
load_system(rig);
% phi: fonte de degrau unico @20 s + switch4 no lado 1
hb = find_system(rig,'LookUnderMasks','all','BlockType','Step');
nfix = 0;
for k = 1:numel(hb)
  ph = get_param(hb{k},'PortHandles'); ln = get_param(ph.Outport(1),'Line');
  if ln==-1, continue; end
  db = get_param(ln,'DstBlockHandle');
  if strcmp(get_param(db(1),'Name'),'Manual Switch4')
    set_param(hb{k},'Time','20'); nfix = nfix+1;
  end
end
ms4 = find_system(rig,'LookUnderMasks','all','BlockType','ManualSwitch','Name','Manual Switch4');
set_param(ms4{1},'sw','1'); save_system(rig);
fprintf('rig acopla: %d step(s) de phi -> 20 s, switch4 lado 1\n', nfix);

% 4 rodadas: TH base/30 @trim, AH base/30 @15.2 (200 s)
cfgs = {'TH_b',1,12,0; 'TH_30',1,12,30; 'AH_b',0,15.2,0; 'AH_30',0,15.2,30};
VT_Throttle = 1; phi_psi = 1; refPsi = 0; reftheta = 0; refAlt = 0;
Rlq = struct();
for c = 1:4
  att_alt = cfgs{c,2}; refVel = cfgs{c,3}; refPhi = cfgs{c,4};
  fprintf('LQRy v2 acopla %s...\n', cfgs{c,1}); tic;
  out_sim = sim(rig,'StopTime','200');
  Rlq.(cfgs{c,1}) = out_sim; fprintf('  ok %.0fs\n', toc);
end
save(fullfile(raiz,'Dados_mat_experimentos','estudo_acoplamento_lqry_v2.mat'),'Rlq','-v7.3');

%% figura + metricas
P = load(fullfile(raiz,'Dados_mat_experimentos','estudo_acoplamento_roll.mat'));
R2D = 180/pi; t_step = 20; T_end = 200;
getP = @(o) struct('t',o.Y.time,'phi',o.Y.signals.values(:,8)*R2D, ...
  'th',o.Y.signals.values(:,9)*R2D,'h',o.Y.signals.values(:,15),'el',o.U.signals.values(:,2)*R2D);
getL = @(o) struct('t',o.phi_NL.time,'phi',o.phi_NL.signals.values(:,2), ...
  'th',o.theta_NL.signals.values(:,2),'h',o.H_NL.signals.values(:,2),'el',o.elev_NL.signals.values(:,1));
sP_th = getP(P.rodadas(3).out);
bP_th = struct('t',sP_th.t,'phi',zeros(size(sP_th.t)), ...
  'th',ones(size(sP_th.t))*mean(sP_th.th(sP_th.t<t_step-2)), ...
  'h', ones(size(sP_th.t))*mean(sP_th.h (sP_th.t<t_step-2)), ...
  'el',ones(size(sP_th.t))*mean(sP_th.el(sP_th.t<t_step-2)));
sP_ah = getP(P.rodadas(6).out);  bP_ah = getP(P.rodadas(5).out);
sL_th = getL(Rlq.TH_30); bL_th = getL(Rlq.TH_b);
sL_ah = getL(Rlq.AH_30); bL_ah = getL(Rlq.AH_b);
phiref_t = Rlq.TH_30.phi_NL.time; phiref = Rlq.TH_30.phi_NL.signals.values(:,1);

cRef=[0.45 0.45 0.45]; cPID=[0 0.4470 0.7410]; cLQR=[0.8500 0.3250 0.0980]; LW=1.4;
pares = { {sP_th,bP_th,cPID,'-'}, {sL_th,bL_th,cLQR,'-'}, {sP_ah,bP_ah,cPID,'--'}, {sL_ah,bL_ah,cLQR,'--'} };
labs = {'PID (ThetaHold, trim)','LQRy (ThetaHold, trim)','PID (AltitudeHold, 15.2 m/s)','LQRy (AltitudeHold, 15.2 m/s)'};
paineis = {'\phi [deg]','\Delta\theta [deg]','\Delta h [m]','\Delta elevator [deg]'};
campos = {'phi','th','h','el'};
fig = figure('Color','w','Units','pixels','Position',[60 60 900 700]); try, set(fig,'Theme','light'); catch, end
tl = tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
for p = 1:4
  ax = nexttile(tl); hold(ax,'on'); grid(ax,'on'); set(ax,'FontSize',10);
  if p==1, plot(ax,phiref_t,phiref,':','Color',cRef,'LineWidth',1.6); end
  for q = 1:4
    s = pares{q}{1}; b = pares{q}{2}; y = s.(campos{p});
    if p>1, y = y - interp1(b.t,b.(campos{p}),s.t,'linear','extrap'); end
    plot(ax,s.t,y,pares{q}{4},'Color',pares{q}{3},'LineWidth',LW);
  end
  ylabel(ax,paineis{p}); xlim(ax,[0 T_end]);
  if p>=3, xlabel(ax,'Time [s]'); end
  if p==1, legend(ax,[{'\phi^{ref}'},labs],'Location','southeast','FontSize',8); end
end
exportgraphics(fig,fullfile(figdir,'fig_acoplamento.png'),'Resolution',300,'BackgroundColor','white'); close(fig);

fid = fopen(txt,'w');
fprintf(fid,'===== acoplamento v2 (%s) — PID retunado x LQRy v2 =====\n', char(datetime));
ssf=@(s,b) mean(s.el(s.t>T_end-20) - interp1(b.t,b.el,s.t(s.t>T_end-20),'linear','extrap'));
ssP_th=ssf(sP_th,bP_th); ssP_ah=ssf(sP_ah,bP_ah); ssL_th=ssf(sL_th,bL_th); ssL_ah=ssf(sL_ah,bL_ah);
fprintf(fid,'ss Delta elev: PID TH=%.3f AH=%.3f (razao %.3f) | LQRy TH=%.3f AH=%.3f (razao %.3f) | (12/15.2)^2=%.3f\n', ...
  ssP_th,ssP_ah,ssP_ah/ssP_th, ssL_th,ssL_ah,ssL_ah/ssL_th, (12/15.2)^2);
dipf=@(s,b) min(s.th(s.t>t_step&s.t<t_step+40) - interp1(b.t,b.th,s.t(s.t>t_step&s.t<t_step+40),'linear','extrap'));
for k=1:3
  s=getP(P.rodadas(k).out);
  b=struct('t',s.t,'th',ones(size(s.t))*mean(s.th(s.t<t_step-2)));
  fprintf(fid,'PID TH amp=%2d: dip dtheta=%+.4f (1-cos=%.4f)\n',P.rodadas(k).amp,dipf(s,b),1-cosd(P.rodadas(k).amp));
end
fprintf(fid,'LQRy TH30: dip dtheta=%+.3f | dh(180s): PID=%.1f LQRy=%.1f\n', dipf(sL_th,bL_th), ...
  sP_th.h(find(sP_th.t>=200,1))-bP_th.h(1), interp1(sL_th.t,sL_th.h,200)-interp1(bL_th.t,bL_th.h,200));
dth_ss=@(s,b) mean(s.th(s.t>T_end-20)-interp1(b.t,b.th,s.t(s.t>T_end-20),'linear','extrap'));
fprintf(fid,'AH: dtheta_ss PID=%.3f LQRy=%.3f | dh_min AH: PID=%.2f LQRy=%.2f\n', ...
  dth_ss(sP_ah,bP_ah), dth_ss(sL_ah,bL_ah), min(sP_ah.h-interp1(bP_ah.t,bP_ah.h,sP_ah.t,'linear','extrap')), ...
  min(sL_ah.h-interp1(bL_ah.t,bL_ah.h,sL_ah.t,'linear','extrap')));
fclose(fid); type(txt);
fprintf('>>> acoplamento v2 completo.\n');
