%% compute_metrics_v2 — metricas da recampanha (PID registro x LQRy v2)
raiz = 'C:\Users\kaue\Documents\PID_DH\HIL_PID\Matlab';
pR = fullfile(raiz,'Dados_mat_ROBUSTEZ'); pC = fullfile(raiz,'Dados_mat_comparacao');
fid = fopen('C:\Users\kaue\Documents\PID_DH\Artigo\dados_recampanha_v2.txt','w');
fprintf(fid,'===== RECAMPANHA v2 (%s) — PID registro (retunado+b0) x LQRy v2 (scheduled, cond 2) =====\n\n', char(datetime));

% ---------- A) robustez: RMS t>12 (missao) ----------
fprintf(fid,'--- A) Robustez missao: RMS/max t>12 s ---\n');
fprintf(fid,'%-7s %-5s | %-8s %-8s | %-8s %-8s | %-8s %-8s\n','cen','ctrl','rmsVT','maxVT','rmsPSI','maxPSI','rmsH','maxH');
cens = {'nom','sato','satoL','pert','vento'};
M = struct();
for c = 1:5
  for ct = {'PID','LQR2'}
    if strcmp(ct{1},'PID'), f = fullfile(pR,sprintf('PID_%s.mat',cens{c}));
    else, f = fullfile(pR,sprintf('LQR2_%s.mat',cens{c})); end
    if ~exist(f,'file')
      fprintf(fid,'%-7s %-5s | DIVERGE (sem arquivo completo)\n',cens{c},ct{1}); continue;
    end
    D = load(f);
    if isfield(D,'S'), X = D.S; else, X = D.out_sim; end
    vt = X.VT_NL.signals.values; ps = X.psi_NL.signals.values;
    hh = X.H_NL.signals.values;  t  = X.VT_NL.time;
    j = t > 12;
    eV = vt(j,2)-vt(j,1); eP = ps(j,1)-ps(j,2); eH = hh(j,2)-hh(j,1);
    m = [rms(eV) max(abs(eV)) rms(eP) max(abs(eP)) rms(eH) max(abs(eH))];
    M.(ct{1}).(cens{c}) = m;
    fprintf(fid,'%-7s %-5s | %-8.3f %-8.3f | %-8.3f %-8.3f | %-8.3f %-8.3f\n',cens{c},ct{1},m);
  end
end
fprintf(fid,'\n--- degradacao (RMS cen / RMS nom) ---\n');
for c = 2:5
  for ct = {'PID','LQR2'}
    if isfield(M.(ct{1}),cens{c})
      m = M.(ct{1}).(cens{c}); m0 = M.(ct{1}).nom;
      fprintf(fid,'%-7s %-5s | VT %-7.2f PSI %-7.2f H %-7.2f\n',cens{c},ct{1},m(1)/m0(1),m(3)/m0(3),m(5)/m0(5));
    end
  end
end

% ---------- B) metricas de degrau por eixo (canais de guiagem) ----------
% VT: degrau 12(11.62)->refVel @5s ; PSI: +5deg @50 ; H: +5m @80
fprintf(fid,'\n--- B) Degrau por eixo: OS[%%] tr[s] ts[s] ess[%%] ---\n');
fprintf(fid,'%-16s %-6s | OS      tr      ts      ess\n','run','canal');
runs = { ...
 'pid2_vel_comoesta','VT'; 'pid2_lat_comoesta','PSI'; 'pid2_long_comoesta','H'; ...
 'pid2_vel_equiparado','VT'; 'pid2_lat_equiparado','PSI'; 'pid2_long_equiparado','H'; ...
 'LQR2_ax_vel','VT'; 'LQR2_ax_lat','PSI'; 'LQR2_ax_long','H'};
for r = 1:size(runs,1)
  nm = runs{r,1}; ch = runs{r,2};
  if startsWith(nm,'pid2'), D = load(fullfile(pC,[nm '.mat'])); X = D.S;
  else, D = load(fullfile(pR,[nm '.mat'])); X = D.out_sim; end
  t = X.VT_NL.time;
  switch ch
    case 'VT',  y = X.VT_NL.signals.values(:,2);  yr = X.VT_NL.signals.values(:,1);
                t0 = 5;  tf = 80;  y0 = interp1(t,y,t0); yf = yr(end);
    case 'PSI', y = X.psi_NL.signals.values(:,1); yr = X.psi_NL.signals.values(:,2);
                t0 = 50; tf = 100; y0 = 0; yf = 5;
    case 'H',   y = X.H_NL.signals.values(:,2);   yr = X.H_NL.signals.values(:,1);
                t0 = 80; tf = 160; y0 = interp1(t,y,t0); yf = 605;
  end
  j = t>=t0 & t<=tf; tj = t(j); yj = y(j); A = yf - y0;
  OS = max(0, (max(yj*sign(A)) - yf*sign(A)) / abs(A) * 100);
  i10 = find((yj-y0)*sign(A) >= 0.1*abs(A), 1); i90 = find((yj-y0)*sign(A) >= 0.9*abs(A), 1);
  if ~isempty(i10) && ~isempty(i90), tr = tj(i90)-tj(i10); else, tr = NaN; end
  fora = abs(yj - yf) > 0.02*abs(A);
  if any(fora), ts = tj(find(fora,1,'last')) - t0; else, ts = 0; end
  ess = abs(yj(end) - yf) / abs(A) * 100;
  fprintf(fid,'%-16s %-6s | %-7.1f %-7.2f %-7.1f %-7.2f\n', nm, ch, OS, tr, ts, ess);
end

% ---------- C) compliance missao (comb): OS/ts/ess por canal ----------
fprintf(fid,'\n--- C) Compliance missao (comb, canais guiagem): OS ts ess ---\n');
combs = {'PID_nom','LQR2_nom'};
for r = 1:2
  D = load(fullfile(pR,[combs{r} '.mat']));
  if isfield(D,'S'), X = D.S; else, X = D.out_sim; end
  t = X.VT_NL.time;
  sets = { ...
   'VT', X.VT_NL.signals.values(:,2), 5, 80, interp1(t,X.VT_NL.signals.values(:,2),5), 15.2; ...
   'PSI',X.psi_NL.signals.values(:,1),50,100, 0, 5; ...
   'H',  X.H_NL.signals.values(:,2), 80,160, interp1(t,X.H_NL.signals.values(:,2),80), 605};
  for s = 1:3
    y = sets{s,2}; t0 = sets{s,3}; tf = sets{s,4}; y0 = sets{s,5}; yf = sets{s,6};
    j = t>=t0 & t<=tf; tj = t(j); yj = y(j); A = yf-y0;
    OS = max(0,(max(yj*sign(A)) - yf*sign(A))/abs(A)*100);
    fora = abs(yj-yf) > 0.02*abs(A);
    if any(fora), ts = tj(find(fora,1,'last')) - t0; else, ts = 0; end
    ess = abs(yj(end)-yf)/abs(A)*100;
    fprintf(fid,'%-9s %-4s | OS=%-6.1f ts=%-6.1f ess=%-5.2f\n', combs{r}, sets{s,1}, OS, ts, ess);
  end
end
fclose(fid);
type('C:\Users\kaue\Documents\PID_DH\Artigo\dados_recampanha_v2.txt');
