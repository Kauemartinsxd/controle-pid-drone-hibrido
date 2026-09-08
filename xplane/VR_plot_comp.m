% VR_plot_comp.m
% =============================================================
% VALIDACAO COM VOO REAL (VR) — etapa 3/3: comparacao e metricas.
%
% Carrega VR_segmentos.mat + a campanha VR_replay_*.mat mais recente
% (ou a indicada em VR_arq_replay) e, por segmento:
%
%   - sobrepoe voo real x X-Plane: taxa primaria do eixo (q/p/r),
%     atitude associada (theta/phi) e V;
%   - metricas: RMS do erro e FIT% = 100*(1 - ||e||/||y - ymed||)
%     na taxa primaria; razao de picos (amplitude XP/real — testa a
%     hipotese de curso �15 deg); p/ leme, frequencia dominante do
%     dutch roll (FFT de r no pos-doublet) real x XP.
%
% Saidas: voos/VR_comp_seg<NN>_<voo>_<eixo>.png (por segmento),
%         voos/VR_comp_resumo.png (sheet das taxas primarias),
%         tabela no console.
% =============================================================

xpDir   = fileparts(mfilename('fullpath'));
voosDir = fullfile(xpDir, 'voos');

L = load(fullfile(voosDir, 'VR_segmentos.mat'));
SEG = L.SEG; T_PRE = L.T_PRE;

if ~exist('VR_arq_replay','var') || isempty(VR_arq_replay)
    d = dir(fullfile(voosDir, 'VR_replay_*.mat'));
    if isempty(d), error('nenhuma campanha VR_replay_*.mat em voos/'); end
    [~, imax] = max([d.datenum]);
    VR_arq_replay = fullfile(voosDir, d(imax).name);
end
C = load(VR_arq_replay);
R = C.R;
if ~exist('VR_sufixo','var'), VR_sufixo = ''; end   % sufixo dos PNGs (ex.: '_ORIG')
fprintf('Campanha: %s (%d segmentos)\n\n', VR_arq_replay, numel(R));

prim = struct('ail', 'p', 'elev', 'q', 'rudd', 'r');
att  = struct('ail', 'phi', 'elev', 'theta', 'rudd', 'phi');

nR = numel(R);
TAB = nan(nR, 6);   % [iseg rmsRate fit% razaoPico fDR_real fDR_xp]
figS = figure('Visible','off','Position',[0 0 420*4 260*ceil(nR/4)]);
try, figS.Theme = 'light'; catch, end   % MATLAB do Kaue em tema escuro

for k = 1:nR
  j = R(k).iseg; s = SEG(j);
  pv = prim.(s.eixo); av = att.(s.eixo);

  tmax = min(s.t(end), R(k).t(end));
  tg = s.t(s.t <= tmax);
  yr = rad2deg(s.(pv)(s.t <= tmax));                    % taxa real [deg/s]
  yx = interp1(R(k).t, rad2deg(R(k).(pv)), tg);         % taxa XP  [deg/s]
  % atitude e V como DELTA do nivel pre-doublet de CADA traco (o
  % replay em modo delta preserva a dinamica, nao o nivel absoluto)
  ipre = tg < T_PRE - 0.5;
  ar = s.(av)(s.t <= tmax);  ar = ar - mean(ar(ipre));
  ax = interp1(R(k).t, R(k).(av), tg); ax = ax - mean(ax(ipre), 'omitnan');
  Vr = s.V(s.t <= tmax);     Vr = Vr - mean(Vr(ipre));
  Vx = interp1(R(k).t, R(k).V, tg); Vx = Vx - mean(Vx(ipre), 'omitnan');

  % metricas na JANELA DINAMICA (1 s antes ate 4 s depois do doublet)
  idy  = tg > T_PRE - 1 & tg < T_PRE + 4 & ~isnan(yx);
  e    = yx(idy) - yr(idy);
  rmse = sqrt(mean(e.^2));
  fit  = 100 * (1 - norm(e) / norm(yr(idy) - mean(yr(idy))));
  rpk  = max(abs(yx(idy))) / max(abs(yr(idy)));

  fdr_r = nan; fdr_x = nan;
  if strcmp(s.eixo, 'rudd')
    fdr_r = freq_dom(tg, yr, T_PRE + 1.5);
    fdr_x = freq_dom(tg, yx, T_PRE + 1.5);
  end
  TAB(k,:) = [j, rmse, fit, rpk, fdr_r, fdr_x];

  % figura detalhada do segmento
  fig = figure('Visible','off','Position',[0 0 900 800]);
  try, fig.Theme = 'light'; catch, end
  subplot(4,1,1);
  cmap = struct('ail',1,'elev',2,'rudd',4);
  plot(s.t, s.u(:,cmap.(s.eixo)) - s.u_trim(cmap.(s.eixo)), 'k', 'LineWidth', 1.2);
  ylabel('\Deltau [-]'); grid on;
  title(sprintf('seg %d: %s %s t0=%.0f s | fit(%s)=%.0f%% rmse=%.1f deg/s', ...
      j, s.voo, s.eixo, s.t0_abs + T_PRE, pv, fit, rmse), 'Interpreter', 'none');
  subplot(4,1,2);
  plot(tg, yr, 'b', tg, yx, 'r', 'LineWidth', 1.1);
  legend('real', 'X-Plane'); ylabel([pv ' [deg/s]']); grid on;
  subplot(4,1,3);
  plot(tg, ar, 'b', tg, ax, 'r', 'LineWidth', 1.1);
  ylabel(['\Delta' av ' [deg]']); grid on;
  subplot(4,1,4);
  plot(tg, Vr, 'b', tg, Vx, 'r', 'LineWidth', 1.1);
  ylabel('\DeltaV [m/s]'); xlabel('t [s]'); grid on;
  png = fullfile(voosDir, sprintf('VR_comp_seg%02d_%s_%s%s.png', j, s.voo, s.eixo, VR_sufixo));
  exportgraphics(fig, png, 'Resolution', 100); close(fig);

  % subplot no resumo
  figure(figS);
  subplot(ceil(nR/4), 4, k);
  plot(tg, yr, 'b', tg, yx, 'r', 'LineWidth', 1);
  grid on; title(sprintf('%d) %s %s fit %.0f%%', j, s.voo, s.eixo, fit), 'Interpreter', 'none');
  if k == 1, legend('real', 'XP'); end
end

png = fullfile(voosDir, ['VR_comp_resumo' VR_sufixo '.png']);
exportgraphics(figS, png, 'Resolution', 100); close(figS);
fprintf('Resumo: %s\n\n', png);

fprintf(' seg | voo    eixo | RMS taxa | fit%% | pico XP/real | fDR real | fDR XP\n');
fprintf('-----+-------------+----------+------+--------------+----------+-------\n');
for k = 1:nR
  j = TAB(k,1); s = SEG(j);
  fprintf(' %3d | %-6s %-4s | %7.1f  | %4.0f | %11.2f  | %7.2f  | %5.2f\n', ...
      j, s.voo, s.eixo, TAB(k,2), TAB(k,3), TAB(k,4), TAB(k,5), TAB(k,6));
end
med = @(c) mean(TAB(~isnan(TAB(:,c)), c));
fprintf('\nMedias: fit %.0f%% | razao de pico %.2f\n', med(3), med(4));
clear VR_arq_replay

%% -----------------
function f = freq_dom(t, y, t_ini)
    % frequencia dominante [Hz] de y no trecho t > t_ini (FFT, detrend)
    m = t > t_ini & ~isnan(y);
    if nnz(m) < 32, f = nan; return; end
    yy = detrend(y(m)); dt = median(diff(t(m)));
    n = 2^nextpow2(numel(yy)*4);
    Y = abs(fft(yy, n));
    fr = (0:n-1) / (n*dt);
    band = fr > 0.2 & fr < 5;               % dutch roll esperado ~0.5-2 Hz
    [~, ip] = max(Y(band)); fb = fr(band);
    f = fb(ip);
end
