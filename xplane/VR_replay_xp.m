% VR_replay_xp.m
% =============================================================
% VALIDACAO COM VOO REAL (VR) — etapa 2/3: campanha de replay.
%
% Para cada janela de VR_segmentos.mat:
%   1) TELEPORTA o DH no X-Plane para a condicao inicial da janela
%      (V0 real, MSL 600 = campo dos voos, theta0/phi0 reais);
%   2) TRIM AUTOMATICO (~TRIM_T s): PI de atitude segura theta=theta0
%      e V=V0 e captura o trim do X-Plane (de0_xp, thr0_xp) — o
%      replay e' em MODO DELTA: u_xp(t) = trim_xp + [u_real(t) -
%      trim_real_da_janela], isolando a dinamica do desajuste de trim;
%   3) RE-TELEPORTA para zerar taxas/atitude e REPLAY em malha
%      aberta das entradas reais (ZOH 25 Hz) a ~20 Hz por T_PRE+T_POS s,
%      gravando a resposta do X-Plane;
%   4) salva tudo em xplane/voos/VR_replay_<timestamp>.mat.
%
% Convencao de sinais (derivada dos proprios logs, VR_extrai_segmentos):
%   X-Plane: +elev = nariz sobe, +ail = rola direita, +rudd = nariz
%   direita. sinais.(eixo) converte o PWM do servo real p/ isso.
%
% REQUER: X-Plane 9 aberto, GEMEO v1.1 carregado e VOANDO (qualquer
% estado — o teleporte assume), plugin XPC ativo.
%
% Config opcional (defina antes de rodar):
%   VR_lista        indices dos segmentos a voar (default: todos validos)
%   VR_reload_cada  true (default) = xp_reload_acf ANTES de cada segmento:
%                   o estoque de energia do motor eletrico do XP9 dura
%                   ~130 s de voo motorizado — sem reload por segmento os
%                   segmentos tardios de cada bloco voam com motor fraco/
%                   morto (thr satura em 1,0 planando, hdot -2) e o trim
%                   nao fecha (visto 2026-09-01: sonda pos-reload nivelou
%                   a 17 m/s com thr 0,59; minutos depois thr 1,0)
%   VR_ganho        struct ail/elev/rudd, ganho de amplitude por eixo
%   VR_J            struct opcional com campos Jyy (guinada) e/ou Jzz
%                   (rolagem): raios de giracao ao quadrado [m^2] escritos
%                   via dref APOS cada reload (reload reseta p/ o .acf)
%                   (default 1,1,1 — hipotese curso completo = �15 deg)
% =============================================================

xpDir   = fileparts(mfilename('fullpath'));
voosDir = fullfile(xpDir, 'voos');
addpath(xpDir);
addpath(fullfile(fileparts(fileparts(xpDir)), 'trabalho_julio', ...
    'PIPER-1-6-roll_back', 'PIPER-1-6-roll_back', 'xplane', ...
    'XPlaneConnect-master', 'MATLAB'));
import XPlaneConnect.*
global GlobalSocket
try, closeUDP(GlobalSocket); catch, end
GlobalSocket = openUDP('127.0.0.1', 49009, 0, 500);

L = load(fullfile(voosDir, 'VR_segmentos.mat'));   % SEG, sinais, T_PRE, T_POS, FS
SEG = L.SEG; sinais = L.sinais; T_PRE = L.T_PRE; T_POS = L.T_POS;

if ~exist('VR_lista','var') || isempty(VR_lista)
    VR_lista = find([SEG.h0] > 10);            % exclui pouso/solo
end
if ~exist('VR_ganho','var'), VR_ganho = struct('ail',1,'elev',1,'rudd',1); end
if ~exist('VR_reload_cada','var'), VR_reload_cada = true; end

MSL0    = 600;      % campo ~600 m MSL (mesmo das campanhas anteriores)
TRIM_T  = 45;       % s MAXIMOS de trim automatico (para antes se convergir)
TRIM_MIN = 8;       % s minimos de trim antes de testar convergencia
REPLAY_T = T_PRE + T_POS;
DT      = 0.05;     % alvo 20 Hz

% ganhos do autotrim (normalizado/rad) — estrutura CLASSICA de trim:
% malha interna de atitude (de <- theta_ref) + externas lentas
% (theta_ref <- erro de V; manete <- erro de razao de subida)
Kq = 0.50; Kth = 0.80; Kith = 0.25;    % profundor (interna)
Kp = 0.40; Kphi = 0.60;                % aileron (asas niveladas)
% pareamento: manete->V (apertada), theta_ref->hdot (lenta). O
% pareamento anterior (theta->V) excitava a fugoide no .acf original.
Kv  = 0.08;                            % manete por (m/s * s) de erro de V
Khd = 0.010;                           % theta_ref [rad] por (m/s * s) de erro de hdot

R = struct('iseg',{},'t',{},'u_sent',{},'V',{},'p',{},'q',{},'r',{}, ...
           'phi',{},'theta',{},'psi',{},'h',{},'de0',{},'thr0',{},'trim_ok',{});

drefs = { ...
    'sim/flightmodel/position/true_airspeed', ...  % 1 VT
    'sim/flightmodel/position/Prad', ...           % 2 p
    'sim/flightmodel/position/Qrad', ...           % 3 q
    'sim/flightmodel/position/Rrad', ...           % 4 r
    'sim/flightmodel/position/phi', ...            % 5 phi deg
    'sim/flightmodel/position/theta', ...          % 6 theta deg
    'sim/flightmodel/position/psi', ...            % 7 psi deg
    'sim/flightmodel/position/elevation', ...      % 8 h MSL
    'sim/time/total_flight_time_sec'};             % 9 t_xp

ii = 1; n_reload = 0;
while ii <= numel(VR_lista)
  j = VR_lista(ii);
  s = SEG(j);
  fprintf('\n===== [%d/%d] seg %d: %s %s t0=%.0f s, V0=%.1f m/s =====\n', ...
      ii, numel(VR_lista), j, s.voo, s.eixo, s.t0_abs + T_PRE, s.V0);

  % motor fresco p/ este segmento (estoque ~130 s do eletrico do XP9)
  if VR_reload_cada
    xp_reload_acf;
    try, closeUDP(GlobalSocket); catch, end
    GlobalSocket = openUDP('127.0.0.1', 49009, 0, 500);
    if exist('VR_J','var') && isstruct(VR_J)
      if isfield(VR_J,'Jyy'), sendDREF('sim/aircraft/weight/acf_Jyy_unitmass', VR_J.Jyy, GlobalSocket); end
      if isfield(VR_J,'Jzz'), sendDREF('sim/aircraft/weight/acf_Jzz_unitmass', VR_J.Jzz, GlobalSocket); end
      pause(0.1);
    end
  end

  th0 = deg2rad(s.theta0);
  ph0 = deg2rad(s.phi0);

  %% 1) teleporte inicial (nivelado na proa atual, V0)
  r0 = [];
  for k_try = 1:6
    try
      r0 = double(getDREFs({'sim/flightmodel/position/latitude', ...
          'sim/flightmodel/position/longitude', ...
          'sim/flightmodel/position/psi'}, GlobalSocket));
      break;
    catch
      fprintf('sem resposta do X-Plane (tent. %d/6)...\n', k_try);
      pause(1.5);
      try, closeUDP(GlobalSocket); catch, end
      GlobalSocket = openUDP('127.0.0.1', 49009, 0, 500);
    end
  end
  if isempty(r0), error('X-Plane nao responde — esta aberto com o XPC ativo?'); end
  lat = r0(1); lon = r0(2); psi0 = r0(3); hdg = deg2rad(psi0);

  teleporta(lat, lon, MSL0, rad2deg(th0), 0, psi0, s.V0, GlobalSocket);

  %% 2) autotrim: equilibrio em (V0, hdot0) — captura de0/thr0.
  % NAO segura theta = theta0 real: os theta0 das janelas incluem
  % subida/descida transitoria e nao sao equilibrios (seg 4 a -7.4 deg
  % dispara a 26 m/s com manete zero — visto 2026-09-01). Estrutura
  % classica: theta_ref lento nula o erro de V (profundor), manete
  % nula o erro de razao de subida hdot0 (do BARO real da janela).
  % Converge com |V-V0|<1.5, |hdot-hdot0|<1, |q|<3 deg/s por 2 s; o
  % replay comeca SEM re-teleporte (equilibrio do proprio X-Plane).
  ipre  = s.t <= T_PRE;
  cfit  = polyfit(s.t(ipre), s.h(ipre), 1);
  hdot0 = max(-3, min(3, cfit(1)));            % razao de subida real [m/s]
  de = 0; ith = 0; thr = 0.45; da = 0;
  th_ref = th0; hdot_f = 0; h_ant = NaN;
  nT = round(TRIM_T/DT); TR = nan(nT, 5); t0 = tic; tprev = 0;
  n_conv = 0; k = 0; conv = false;
  while toc(t0) < TRIM_T
    k = k + 1;
    raw = ler(drefs, GlobalSocket);
    if isempty(raw), pause(DT); continue; end
    t  = toc(t0); dt = max(t - tprev, 1e-3); tprev = t;
    q  = raw(3); p = raw(2);
    if ~isnan(h_ant), hdot_f = 0.9*hdot_f + 0.1*(raw(8) - h_ant)/dt; end
    h_ant = raw(8);
    % theta_ref lento persegue a razao de subida real; manete segura V
    th_ref = max(deg2rad(-15), min(deg2rad(15), th_ref + Khd*(hdot0 - hdot_f)*dt));
    eth = deg2rad(raw(6)) - th_ref;
    eph = deg2rad(raw(5));
    ith = max(-0.5, min(0.5, ith + Kith*(-eth)*dt));
    de  = max(-1, min(1, ith - Kq*q - Kth*eth));
    da  = max(-1, min(1, -Kp*p - Kphi*eph));
    thr = max(0, min(1, thr + Kv*(s.V0 - raw(1))*dt));
    sendCTRL([de, da, 0, thr, -998, -998], 0, GlobalSocket);
    TR(min(k,nT),:) = [t, de, thr, raw(1), hdot_f];
    if t > TRIM_MIN && abs(raw(1) - s.V0) < 1.5 && abs(hdot_f - hdot0) < 1 ...
            && abs(q) < deg2rad(3)
      n_conv = n_conv + 1;
      if n_conv >= round(2/DT), conv = true; break; end
    else
      n_conv = 0;
    end
    resto = k*DT - toc(t0); if resto > 0, pause(resto); end
  end
  ult = TR(TR(:,1) > toc(t0) - 2, :);          % ultimos ~2 s
  de0 = mean(ult(:,2), 'omitnan');
  thr0 = mean(ult(:,3), 'omitnan');
  Vfim = mean(ult(:,4), 'omitnan');
  trim_ok = conv;
  fprintf('trim (%.0f s): de0=%+.3f thr0=%.2f V=%.1f (alvo %.1f) hdot=%+.1f (alvo %+.1f) %s\n', ...
      toc(t0), de0, thr0, Vfim, s.V0, mean(ult(:,5),'omitnan'), hdot0, ...
      ternario(trim_ok, 'CONVERGIU', 'NAO CONVERGIU'));

  % motor morto? (thr alto e V caindo — pendencia conhecida do XP9)
  % SEM input() aqui: prompt interativo e' incompativel com execucao
  % via MCP (a chamada seguinte e' consumida como resposta e o script
  % entra em loop de retry infinito — visto 2026-09-01). Reload
  % automatico via xp_reload_acf, no maximo 2 tentativas por segmento.
  if thr0 > 0.95 && Vfim < s.V0 - 4
    n_reload = n_reload + 1;
    fprintf(2, 'MOTOR PROVAVELMENTE MORTO (thr %.2f, V %.1f) — tentativa %d/2.\n', ...
        thr0, Vfim, n_reload);
    if n_reload > 2
      error('VR_replay_xp: motor continua morto apos 2 reloads — abortando campanha.');
    end
    if ~exist('xp_reload_acf', 'file')
      error('VR_replay_xp: motor morto e xp_reload_acf indisponivel — recarregue manualmente e rode de novo.');
    end
    xp_reload_acf;  pause(3);
    try, closeUDP(GlobalSocket); catch, end
    GlobalSocket = openUDP('127.0.0.1', 49009, 0, 500);
    continue;   % refaz este segmento (ii nao foi incrementado)
  end
  n_reload = 0;

  %% 3) replay em malha aberta — transicao SUAVE a partir do trim
  % (sem teleporte: o ultimo sendCTRL do trim vira o u(0) do replay)
  nR = round(REPLAY_T/DT) + 20;
  D = nan(nR, 10); US = nan(nR, 5);
  t0 = tic; k = 0;
  while true
    t = toc(t0);
    if t > REPLAY_T, break; end
    k = k + 1;
    % entradas reais na convencao X-Plane (ZOH, modo delta)
    du_ail  = interp1(s.t, s.u(:,1), t, 'previous', s.u(end,1)) - s.u_trim(1);
    du_elev = interp1(s.t, s.u(:,2), t, 'previous', s.u(end,2)) - s.u_trim(2);
    du_thr  = interp1(s.t, s.u(:,3), t, 'previous', s.u(end,3)) - s.u_trim(3);
    du_rudd = interp1(s.t, s.u(:,4), t, 'previous', s.u(end,4)) - s.u_trim(4);
    de_c  = max(-1, min(1, de0 + sinais.elev * VR_ganho.elev * du_elev));
    da_c  = max(-1, min(1,       sinais.ail  * VR_ganho.ail  * du_ail));
    dr_c  = max(-1, min(1,       sinais.rudd * VR_ganho.rudd * du_rudd));
    thr_c = max(0,  min(1, thr0 + du_thr));
    sendCTRL([de_c, da_c, dr_c, thr_c, -998, -998], 0, GlobalSocket);
    raw = ler(drefs, GlobalSocket);
    if ~isempty(raw)
      D(k,:)  = [t, raw(:)'];
      US(k,:) = [t, de_c, da_c, dr_c, thr_c];
    end
    resto = (k*DT) - toc(t0); if resto > 0, pause(resto); end
  end
  D = D(~isnan(D(:,1)), :); US = US(~isnan(US(:,1)), :);

  ir = numel(R) + 1;
  R(ir).iseg = j;      R(ir).t = D(:,1);      R(ir).u_sent = US;
  R(ir).V = D(:,2);    R(ir).p = D(:,3);      R(ir).q = D(:,4);
  R(ir).r = D(:,5);    R(ir).phi = D(:,6);    R(ir).theta = D(:,7);
  R(ir).psi = rad2deg(unwrap(deg2rad(D(:,8))));
  R(ir).h = D(:,9);
  R(ir).de0 = de0;     R(ir).thr0 = thr0;     R(ir).trim_ok = trim_ok;
  fprintf('replay: %d amostras (%.1f Hz efetivo), V fim %.1f m/s\n', ...
      size(D,1), size(D,1)/REPLAY_T, D(end,2));
  if D(end,2) < 5
    fprintf(2, 'AVISO: VT final < 5 m/s — possivel crash durante o replay.\n');
  end
  ii = ii + 1;
end

fn = fullfile(voosDir, ['VR_replay_' datestr(now, 'yyyymmdd_HHMMSS') '.mat']);
save(fn, 'R', 'VR_lista', 'sinais', 'VR_ganho', 'MSL0', 'TRIM_T');
fprintf('\nCampanha salva: %s (%d segmentos)\n', fn, numel(R));
clear VR_lista

%% ----------------- funcoes locais -----------------
function teleporta(lat, lon, msl, pitch_deg, roll_deg, psi_deg, VT, sock)
    import XPlaneConnect.*
    hdg = deg2rad(psi_deg);
    for rep = 1:2   % repete p/ garantir (licao do xp_read_dh)
        sendPOSI([lat, lon, msl, pitch_deg, roll_deg, psi_deg, -998], 0, sock);
        pause(0.05);
        sendDREF('sim/flightmodel/position/local_vx',  VT*sin(hdg), sock);
        sendDREF('sim/flightmodel/position/local_vy',  0,           sock);
        sendDREF('sim/flightmodel/position/local_vz', -VT*cos(hdg), sock);
        sendDREF('sim/flightmodel/position/Prad', 0, sock);
        sendDREF('sim/flightmodel/position/Qrad', 0, sock);
        sendDREF('sim/flightmodel/position/Rrad', 0, sock);
        pause(0.05);
    end
end

function raw = ler(drefs, sock)
    global GlobalSocket
    import XPlaneConnect.*
    raw = [];
    try
        raw = double(getDREFs(drefs, sock));
        % descarta resposta orfa (validacao como no xp_send_dh)
        if numel(raw) < 9 || raw(1) < -1 || raw(1) > 200, raw = []; end
    catch
        try, closeUDP(GlobalSocket); catch, end
        try, GlobalSocket = openUDP('127.0.0.1', 49009, 0, 500); catch, end
    end
end

function s = ternario(c, a, b)
    if c, s = a; else, s = b; end
end
