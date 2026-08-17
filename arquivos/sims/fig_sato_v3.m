% fig_sato_v3 — cenario A com 3 curvas + marcador de divergencia + inset
raiz='C:\Users\kaue\Documents\PID_DH\HIL_PID\Matlab'; pR=fullfile(raiz,'Dados_mat_ROBUSTEZ');
figdir='C:\Users\kaue\Documents\PID_DH\Artigo\figs';
cRef=[0.45 0.45 0.45]; cPID=[0 0.4470 0.7410]; cLQR=[0.8500 0.3250 0.0980]; cLQ1=[0.4660 0.6740 0.1880]; LW=1.4;
P=load(fullfile(pR,'PID_sato.mat')); L2=load(fullfile(pR,'LQR2_sato_parcial.mat')); L1=load(fullfile(pR,'LQR_sato.mat'));
g=@(X) struct('t',X.VT_NL.time,'vt',X.VT_NL.signals.values,'ps',X.psi_NL.signals.values,'hh',X.H_NL.signals.values,'th',X.theta_NL.signals.values);
A=g(P.S); B=g(L2.out_sim); C=g(L1.out_sim);
fig=figure('Color','w','Units','pixels','Position',[60 60 760 800],'Visible','off');
tl=tiledlayout(fig,4,1,'TileSpacing','compact','Padding','compact');
sn={'vt',2,1,'V_T [m/s]';'ps',1,2,'\psi [deg]';'hh',2,1,'h [m]';'th',2,0,'\theta [deg]'};
for p=1:4
  ax=nexttile(tl); hold(ax,'on'); grid(ax,'on'); set(ax,'FontSize',10);
  f=sn{p,1}; col=sn{p,2}; rcol=sn{p,3};
  if rcol>0, plot(ax,A.t,A.(f)(:,rcol),'--','Color',cRef,'LineWidth',1.1); end
  plot(ax,A.t,A.(f)(:,col),'-','Color',cPID,'LineWidth',LW);
  plot(ax,C.t,C.(f)(:,col),'-','Color',cLQ1,'LineWidth',LW);
  plot(ax,B.t,B.(f)(:,col),'-','Color',cLQR,'LineWidth',LW);
  plot(ax,B.t(end),B.(f)(end,col),'x','Color',cLQR,'MarkerSize',10,'LineWidth',2);
  ylabel(ax,sn{p,4}); xlim(ax,[0 300]);
  if p==1, legend(ax,{'Ref','PID','LQRy (fixed-gain)','LQRy (scheduled)','sched. diverges'},'Location','east','FontSize',8); end
  if p==4, xlabel(ax,'Time [s]'); end
end
axi=axes(fig,'Position',[0.33 0.60 0.27 0.11]); hold(axi,'on'); grid(axi,'on');
plot(axi,A.t,A.ps(:,2),'--','Color',cRef);
plot(axi,C.t,C.ps(:,1),'-','Color',cLQ1,'LineWidth',1.2);
plot(axi,B.t,B.ps(:,1),'-','Color',cLQR,'LineWidth',1.2);
plot(axi,B.t(end),B.ps(end,1),'x','Color',cLQR,'MarkerSize',8,'LineWidth',2);
xlim(axi,[45 58]); title(axi,'45-58 s','FontSize',8); set(axi,'FontSize',8);
exportgraphics(fig,fullfile(figdir,'fig_rob_sato.png'),'Resolution',300,'BackgroundColor','white'); close(fig);
pk=B.ps(B.t>50,1); tk=B.t(B.t>50);
fprintf('psi(sched): 51s=%.1f 53s=%.1f 55s=%.1f\n', interp1(tk,pk,51), interp1(tk,pk,53), interp1(tk,pk,55));
j=C.t>12; fprintf('fixedgain sato: rmsVT=%.3f rmsPSI=%.3f rmsH=%.3f\n', ...
  rms(C.vt(j,2)-C.vt(j,1)), rms(C.ps(j,1)-C.ps(j,2)), rms(C.hh(j,2)-C.hh(j,1)));
disp('fig_rob_sato v3 ok');
