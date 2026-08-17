% fig_acopla_v2 — pos-processamento do acoplamento v2 (dados ja simulados)
raiz = 'C:\Users\kaue\Documents\PID_DH\HIL_PID\Matlab';
figdir = 'C:\Users\kaue\Documents\PID_DH\Artigo\figs';
txt = 'C:\Users\kaue\Documents\PID_DH\Artigo\dados_acoplamento_v2.txt';
L = load(fullfile(raiz,'Dados_mat_experimentos','estudo_acoplamento_lqry_v2.mat')); Rlq = L.Rlq;
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
fig = figure('Color','w','Units','pixels','Position',[60 60 900 700],'Visible','off');
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
fprintf(fid,'LQRy TH30: dip dtheta=%+.3f | dh(200s): PID=%.1f LQRy=%.1f\n', dipf(sL_th,bL_th), ...
  interp1(sP_th.t,sP_th.h,200)-bP_th.h(1), interp1(sL_th.t,sL_th.h,200)-interp1(bL_th.t,bL_th.h,200));
dth_ss=@(s,b) mean(s.th(s.t>T_end-20)-interp1(b.t,b.th,s.t(s.t>T_end-20),'linear','extrap'));
fprintf(fid,'AH: dtheta_ss PID=%.3f LQRy=%.3f | dh_min AH: PID=%.2f LQRy=%.2f\n', ...
  dth_ss(sP_ah,bP_ah), dth_ss(sL_ah,bL_ah), min(sP_ah.h-interp1(bP_ah.t,bP_ah.h,sP_ah.t,'linear','extrap')), ...
  min(sL_ah.h-interp1(bL_ah.t,bL_ah.h,sL_ah.t,'linear','extrap')));
fclose(fid); type(txt);
disp('fig_acoplamento + metricas ok');
