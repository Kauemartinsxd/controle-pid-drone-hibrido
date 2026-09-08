% VR_extrai_segmentos.m
% =============================================================
% VALIDACAO COM VOO REAL (VR) — etapa 1/3: extracao de janelas.
%
% Le os dois logs ArduPilot (.mat exportado do Mission Planner) dos
% voos reais do DH de dez/2024 (ArduPlane V4.5.6, Pixhawk 1, modo
% MANUAL, Q_ENABLE=0 -> asa fixa pura, RCOU = passthrough do radio)
% e corta JANELAS de ~10 s em torno de cada doublet (2 s pre + 8 s
% pos), com:
%
%   - entradas RCOU convertidas p/ normalizado [-1,+1] (superficies)
%     e [0,1] (manete): (PWM-1500)/400 e (PWM-1100)/800;
%   - saidas medidas re-amostradas a 25 Hz: phi/theta/psi (ATT),
%     p/q/r (IMU I=0, rad/s), V (ARSP pitot), h (BARO alt relativa);
%   - SINAIS DE CONVENCAO por eixo estimados DO PROPRIO DADO REAL
%     (correlacao entrada x taxa com lag curto): o X-Plane usa
%     +elev = nariz sobe (+q), +ail = rola p/ direita (+p),
%     +rudd = nariz p/ direita (+r); o sinal sgn.<eixo> converte o
%     PWM do servo real para essa convencao;
%   - filtro de qualidade: V0 > 12 m/s, |phi0| < 25 deg,
%     |theta0| < 15 deg no inicio da janela (voo trimado).
%
% Saida: xplane/voos/VR_segmentos.mat (struct array SEG + meta)
%        + contact sheet VR_segmentos_overview.png
% =============================================================

clear SEG;
xpDir   = fileparts(mfilename('fullpath'));
voosDir = fullfile(xpDir, 'voos');
raiz    = fileparts(fileparts(xpDir));

logs = { ...
  fullfile(raiz, 'Log-DH-longitudinal-dez-24.bin-440203 (1).mat'),      'long'; ...
  fullfile(raiz, 'Log-DH-laterodirecional-dez-24.bin-449987 (1).mat'),  'latdir'};

T_PRE  = 2;    % s antes do inicio do doublet
T_POS  = 8;    % s depois do inicio do doublet
FS     = 25;   % Hz da grade comum (= taxa do RCOU/ATT)
chname = {'ail','elev','thr','rudd'};   % RCOU C1..C4 (SERVOx_FUNCTION 4/19/70/21)

SEG = struct('voo',{},'eixo',{},'t0_abs',{},'t',{},'u',{},'phi',{},'theta',{}, ...
             'psi',{},'p',{},'q',{},'r',{},'V',{},'h',{},'V0',{},'h0',{}, ...
             'phi0',{},'theta0',{},'u_trim',{});
sgn = struct();

for iv = 1:2
  S = load(logs{iv,1}, 'RCOU','ATT','IMU','ARSP','BARO');
  tU  = S.RCOU(:,2)*1e-6;
  U   = S.RCOU(:,3:6);                       % C1 ail, C2 elev, C3 thr, C4 rudd [PWM]
  tA  = S.ATT(:,2)*1e-6;
  phi = S.ATT(:,4); theta = S.ATT(:,6); yaw = S.ATT(:,8);   % [deg]
  im0 = S.IMU(:,3) == 0;                     % instancia 0 do IMU
  tG  = S.IMU(im0,2)*1e-6;
  gyr = S.IMU(im0,4:6);                      % p q r [rad/s]
  tV  = S.ARSP(:,2)*1e-6; V = S.ARSP(:,4);   % pitot [m/s]
  tH  = S.BARO(:,2)*1e-6; h = S.BARO(:,4);   % alt rel. armagem [m]
  as_u = interp1(tV, V, tU);

  fprintf('=== voo %s: max|p| %.1f, max|q| %.1f, max|r| %.1f rad/s ===\n', ...
      logs{iv,2}, max(abs(gyr(:,1))), max(abs(gyr(:,2))), max(abs(gyr(:,3))));

  %% Deteccao de doublets: excursao > 250 PWM do neutro, em voo
  for ch = [1 2 4]
    ex = abs(U(:,ch) - 1500) > 250 & as_u > 10;
    d = diff([0; ex; 0]); ini = find(d==1); fim = find(d==-1) - 1;
    ev = [];
    for k = 1:numel(ini)
      if ~isempty(ev) && tU(ini(k)) - ev(end,2) < 3
        ev(end,2) = tU(fim(k));
      else
        ev(end+1,:) = [tU(ini(k)), tU(fim(k))]; %#ok<SAGROW>
      end
    end
    for k = 1:size(ev,1)
      t0 = ev(k,1) - T_PRE;
      tg = (t0 : 1/FS : t0 + T_PRE + T_POS)';
      if tg(1) < tU(1) || tg(end) > tU(end), continue; end
      % entradas: ZOH (bordas do doublet preservadas); saidas: linear
      un = zeros(numel(tg), 4);
      for c = 1:4
        un(:,c) = interp1(tU, U(:,c), tg, 'previous');
      end
      un(:,[1 2 4]) = (un(:,[1 2 4]) - 1500) / 400;   % superficies [-1,+1]
      un(:,3)       = (un(:,3) - 1100) / 800;         % manete [0,1]
      s = struct();
      s.voo    = logs{iv,2};
      s.eixo   = chname{ch};
      s.t0_abs = t0;
      s.t      = tg - t0;
      s.u      = un;                                   % [ail elev thr rudd]
      s.phi    = interp1(tA, phi,   tg);
      s.theta  = interp1(tA, theta, tg);
      s.psi    = interp1(tA, unwrap(deg2rad(yaw))*180/pi, tg);
      s.p      = interp1(tG, gyr(:,1), tg);
      s.q      = interp1(tG, gyr(:,2), tg);
      s.r      = interp1(tG, gyr(:,3), tg);
      s.V      = interp1(tV, V, tg);
      s.h      = interp1(tH, h, tg);
      ipre     = s.t <= T_PRE;                         % trecho pre-doublet
      s.V0     = mean(s.V(ipre));
      s.h0     = mean(s.h(ipre));
      s.phi0   = mean(s.phi(ipre));
      s.theta0 = mean(s.theta(ipre));
      s.u_trim = mean(un(ipre,:), 1);                  % trim real da janela
      % filtro de qualidade (voo trimado no engate da janela)
      if s.V0 < 12 || abs(s.phi0) > 25 || abs(s.theta0) > 15
        fprintf('  [%s/%-4s t=%3.0f s] DESCARTADA (V0=%.1f phi0=%.0f theta0=%.0f)\n', ...
            s.voo, s.eixo, t0+T_PRE, s.V0, s.phi0, s.theta0);
        continue;
      end
      SEG(end+1) = s; %#ok<SAGROW>
    end
  end

  %% Sinais de convencao por eixo (correlacao du x taxa, lags 0..0.6 s)
  % X-Plane: +elev -> +q, +ail -> +p, +rudd -> +r
  pares = {'ail','p',1; 'elev','q',2; 'rudd','r',4};
  for kk = 1:size(pares,1)
    idx = find(strcmp({SEG.voo}, logs{iv,2}) & strcmp({SEG.eixo}, pares{kk,1}));
    if isempty(idx), continue; end
    cbest = 0;
    for L = 0:round(0.6*FS)
      c = 0;
      for j = idx
        du = SEG(j).u(:,pares{kk,3}) - SEG(j).u_trim(pares{kk,3});
        ra = SEG(j).(pares{kk,2});
        c  = c + sum(du(1:end-L) .* ra(1+L:end));
      end
      if abs(c) > abs(cbest), cbest = c; end
    end
    sgn.(logs{iv,2}).(pares{kk,1}) = sign(cbest);
    fprintf('  sinal %s/%s: %+d (corr pico %.1f)\n', logs{iv,2}, pares{kk,1}, sign(cbest), cbest);
  end
end

%% Consolida sinais (voto entre voos quando o eixo aparece nos dois)
sinais = struct('ail',0,'elev',0,'rudd',0);
for f = {'ail','elev','rudd'}
  v = 0;
  for iv = 1:2
    if isfield(sgn, logs{iv,2}) && isfield(sgn.(logs{iv,2}), f{1})
      v = v + sgn.(logs{iv,2}).(f{1});
    end
  end
  sinais.(f{1}) = sign(v);
end
fprintf('\nSinais consolidados (servo real -> convencao X-Plane): ail %+d, elev %+d, rudd %+d\n', ...
    sinais.ail, sinais.elev, sinais.rudd);

%% Salva
fn = fullfile(voosDir, 'VR_segmentos.mat');
save(fn, 'SEG', 'sinais', 'T_PRE', 'T_POS', 'FS');
fprintf('%d segmentos salvos em %s\n', numel(SEG), fn);
for j = 1:numel(SEG)
  fprintf('  %2d) %-6s %-4s t0=%4.0f s  V0=%4.1f m/s  h0=%3.0f m  phi0=%+5.1f  theta0=%+5.1f\n', ...
      j, SEG(j).voo, SEG(j).eixo, SEG(j).t0_abs + T_PRE, SEG(j).V0, SEG(j).h0, SEG(j).phi0, SEG(j).theta0);
end

%% Contact sheet
n = numel(SEG); nc = 4; nr = ceil(n/nc);
fig = figure('Visible','off','Position',[0 0 420*nc 260*nr]);
try, fig.Theme = 'light'; catch, end   % MATLAB do Kaue em tema escuro
for j = 1:n
  subplot(nr, nc, j);
  cmap = struct('ail',1,'elev',2,'rudd',4);
  cin  = cmap.(SEG(j).eixo);
  yyaxis left;  plot(SEG(j).t, SEG(j).u(:,cin) - SEG(j).u_trim(cin), 'LineWidth', 1); ylabel('\Deltau [-]');
  yyaxis right;
  switch SEG(j).eixo
    case 'ail',  plot(SEG(j).t, rad2deg(SEG(j).p)); ylabel('p [deg/s]');
    case 'elev', plot(SEG(j).t, rad2deg(SEG(j).q)); ylabel('q [deg/s]');
    case 'rudd', plot(SEG(j).t, rad2deg(SEG(j).r)); ylabel('r [deg/s]');
  end
  grid on; title(sprintf('%d) %s %s t=%.0fs V=%.1f', j, SEG(j).voo, SEG(j).eixo, ...
      SEG(j).t0_abs + T_PRE, SEG(j).V0), 'Interpreter', 'none');
end
png = fullfile(voosDir, 'VR_segmentos_overview.png');
exportgraphics(fig, png, 'Resolution', 100); close(fig);
fprintf('Contact sheet: %s\n', png);
