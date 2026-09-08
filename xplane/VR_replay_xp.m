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

% (2026-09-08) ESCALA DE CURSO: o PWM real normalizado (+-1 = +-VR_curso_real
% graus FISICOS; +-15 confirmado pelo responsavel do DH) vai direto no
% sendCTRL, cujo +-1 e' o CURSO DO .acf (25 deg no gemeo v1.2, 15 no
% original). Sem esta escala o gemeo recebia 25/15 = 1,67x a deflexao real
% em TODOS os eixos (campanhas de 2026-09-01: "aileron 2x quente", "leme
% 1,07 apos Jyy", "profundor 0,75" — todos com 1,67x de entrada).
% VR_curso_real = [] desliga a escala (mapeamento 1:1 antigo).
if ~exist('VR_curso_real','var'), VR_curso_real = 15; end
lims = double(getDREFs({'sim/aircraft/controls/acf_elev_up', ...
    'sim/aircraft/controls/acf_ail1_up','sim/aircraft/controls/acf_rudd_lr'}, GlobalSocket));
if numel(lims) < 3 || any(lims < 5) || any(lims > 60)
    error('VR_replay_xp: cursos do .acf invalidos (%s) — X-Plane com aeronave carregada?', mat2str(lims, 3));
end
if isempty(VR_curso_real)
    esc = struct('elev', 1, 'ail', 1, 'rudd', 1);
else
    esc = struct('elev', VR_curso_real/lims(1), 'ail', VR_curso_real/lims(2), 'rudd', VR_curso_real/lims(3));
end
fprintf('cursos do .acf: elev %.0f / ail %.0f / rudd %.0f deg; curso real %s deg -> escala %.3f / %.3f / %.3f\n', ...
    lims(1), lims(2), lims(3), mat2str(VR_curso_real), esc.elev, esc.ail, esc.rudd);
fprintf('sinais servo->XP: ail %+d elev %+d rudd %+d | ganhos extra: ail %.2f elev %.2f rudd %.2f\n', ...
    sinais.ail, sinais.elev, sinais.rudd, VR_ganho.ail, VR_ganho.elev, VR_ganho.rudd);

% arquivo da campanha definido JA' (salvamento incremental por segmento:
% timeout do cliente MCP nao perde dados)
fn = fullfile(voosDir, ['VR_replay_' datestr(now, 'yyyymmdd_HHMMSS') '.mat']);

MSL0    = 600;      % campo ~600 m MSL (mesmo das campanhas anteriores)
if ~exist('VR_trim_max','var') || isempty(VR_trim_max), VR_trim_max = 45; end
TRIM_T  = VR_trim_max;   % s MAXIMOS de trim automatico (para antes se convergir); <= ~100 pelo estoque do motor
% (2026-09-08) VR_trim_min: o teleporte derruba o RPM do motor (spool ~20 s) e
% o SOPRO da helice sobre a empenagem muda a autoridade de profundor/leme; com
% trim de 13-19 s o doublet caia com o motor em estados diferentes (pico de q
% 0,92 vs 0,43 na mesma configuracao). Default 8 (historico); use >= 25 para
% calibracao. O empuxo (POINT_thrust) e TRQ no fim do trim vao para R.
if ~exist('VR_trim_min','var') || isempty(VR_trim_min), VR_trim_min = 8; end
TRIM_MIN = VR_trim_min;   % s minimos de trim antes de testar convergencia
% (2026-09-08 tarde) VR_trim_modo: 'hdot_thr' (default, historico: manete<-V
% rapida, theta_ref<-hdot lenta) deixava o doublet cair com a manete ainda
% subindo/caindo (empuxo 1,2..5,3 N na mesma condicao). 'classico' = manete
% <- erro de hdot (integrador lento) e theta_ref <- erro de V (pitch for
% speed), e a convergencia exige tambem MANETE ESTAVEL (excursao < 0,03 nos
% ultimos 4 s) — empuxo reprodutivel = sopro reprodutivel na empenagem.
if ~exist('VR_trim_modo','var') || isempty(VR_trim_modo), VR_trim_modo = 'hdot_thr'; end
Ktv = 0.015;   % classico/thr_fixo: theta_ref [rad] por (m/s * s) de erro de V (V alta -> nariz sobe)
Kpv = 0.010;   % thr_fixo: termo proporcional de theta_ref [rad por m/s]
Kth = 0.03;    % classico: manete por (m/s * s) de erro de hdot
% 'thr_fixo': manete CONSTANTE = VR_thr_fixo (mesmo RPM/sopro em toda corrida,
% por construcao) e so theta_ref persegue V0 (pitch for speed); a razao de
% subida resultante e' livre (gamma pequeno nao altera o curto periodo). O
% 'classico' (manete<-hdot) divergiu no teste de 15:47 (manete foi a zero no
% transiente pos-teleporte e nao voltou).
if ~exist('VR_thr_fixo','var') || isempty(VR_thr_fixo), VR_thr_fixo = 0.42; end
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
    reload_robusto(3);
    try, closeUDP(GlobalSocket); catch, end
    GlobalSocket = openUDP('127.0.0.1', 49009, 0, 500);
    if exist('VR_J','var') && isstruct(VR_J)
      if isfield(VR_J,'Jyy'), sendDREF('sim/aircraft/weight/acf_Jyy_unitmass', VR_J.Jyy, GlobalSocket); end
      if isfield(VR_J,'Jzz'), sendDREF('sim/aircraft/weight/acf_Jzz_unitmass', VR_J.Jzz, GlobalSocket); end
      pause(0.1);
    end
    % (2026-09-08) VR_drefs: cell {nome, valor; ...} escrito APOS cada reload
    % (reload reseta p/ o .acf) — calibracao em tempo de execucao de
    % parametros do XP9 que sao writable e honrados pela fisica (ex.:
    % acf_elev_crat/acf_rudd_crat/acf_ail1_crat = razao de corda das
    % superficies, acf_dihed1 = diedro por parte, acf_Croot/Ctip). Valores
    % vetoriais: escreve o vetor inteiro (leia antes e altere os indices).
    if exist('VR_drefs','var') && iscell(VR_drefs) && ~isempty(VR_drefs)
      for kd = 1:size(VR_drefs, 1)
        sendDREF(VR_drefs{kd,1}, VR_drefs{kd,2}, GlobalSocket); pause(0.05);
      end
      pause(0.2);
      for kd = 1:size(VR_drefs, 1)
        try
          qv = getDREFs(VR_drefs(kd,1), GlobalSocket); if iscell(qv), qv = qv{1}; end; qv = double(qv(:))';
          if numel(qv) <= 4, fprintf('  dref %s = %s\n', VR_drefs{kd,1}, mat2str(qv, 4));
          else, fprintf('  dref %s: %d valores, soma %.4g\n', VR_drefs{kd,1}, numel(qv), sum(qv)); end
        catch
        end
      end
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
    if strcmp(VR_trim_modo, 'classico')
      % pitch for speed + manete pela razao de subida (ambos integradores lentos)
      th_ref = max(deg2rad(-15), min(deg2rad(15), th_ref + Ktv*(raw(1) - s.V0)*dt));
      thr = max(0, min(1, thr + Kth*(hdot0 - hdot_f)*dt));
    elseif strcmp(VR_trim_modo, 'thr_fixo')
      % manete constante; theta_ref = integrador + proporcional no erro de V
      ith_v = th_ref + Ktv*(raw(1) - s.V0)*dt;           % parte integral (guardada em th_ref)
      th_ref = max(deg2rad(-15), min(deg2rad(15), ith_v));
      thr = VR_thr_fixo;
    else
      % theta_ref lento persegue a razao de subida real; manete segura V
      th_ref = max(deg2rad(-15), min(deg2rad(15), th_ref + Khd*(hdot0 - hdot_f)*dt));
      thr = max(0, min(1, thr + Kv*(s.V0 - raw(1))*dt));
    end
    if strcmp(VR_trim_modo, 'thr_fixo')
      eth = deg2rad(raw(6)) - (th_ref + Kpv*(raw(1) - s.V0));   % + proporcional em V
    else
      eth = deg2rad(raw(6)) - th_ref;
    end
    eph = deg2rad(raw(5));
    ith = max(-0.5, min(0.5, ith + Kith*(-eth)*dt));
    de  = max(-1, min(1, ith - Kq*q - Kth*eth));
    da  = max(-1, min(1, -Kp*p - Kphi*eph));
    sendCTRL([de, da, 0, thr, -998, -998], 0, GlobalSocket);
    TR(min(k,nT),:) = [t, de, thr, raw(1), hdot_f];
    thr_estavel = true;
    if strcmp(VR_trim_modo, 'classico')
      ult4 = TR(TR(:,1) > t - 4 & ~isnan(TR(:,1)), 3);
      thr_estavel = numel(ult4) > 10 && (max(ult4) - min(ult4)) < 0.03;
    end
    hdot_ok = abs(hdot_f - hdot0) < 1;
    if strcmp(VR_trim_modo, 'thr_fixo'), hdot_ok = true; end      % gamma livre
    if t > TRIM_MIN && abs(raw(1) - s.V0) < 1.5 && hdot_ok ...
            && abs(q) < deg2rad(3) && thr_estavel
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
  % estado do motor no fim do trim (proxy do sopro): empuxo [N] e torque
  thrust0 = NaN; trq0 = NaN;
  try
    qa = getDREFs({'sim/flightmodel/engine/POINT_thrust'}, GlobalSocket); if iscell(qa), qa = qa{1}; end; thrust0 = double(qa(1));
    qa = getDREFs({'sim/flightmodel/engine/ENGN_TRQ'}, GlobalSocket);     if iscell(qa), qa = qa{1}; end; trq0 = double(qa(1));
  catch
  end
  fprintf('motor no fim do trim: empuxo %.2f N, TRQ %.3f\n', thrust0, trq0);
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
    reload_robusto(3);  pause(3);
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
    de_c  = max(-1, min(1, de0 + sinais.elev * VR_ganho.elev * esc.elev * du_elev));
    da_c  = max(-1, min(1,       sinais.ail  * VR_ganho.ail  * esc.ail  * du_ail));
    dr_c  = max(-1, min(1,       sinais.rudd * VR_ganho.rudd * esc.rudd * du_rudd));
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
  R(ir).thrust0 = thrust0; R(ir).trq0 = trq0;
  fprintf('replay: %d amostras (%.1f Hz efetivo), V fim %.1f m/s\n', ...
      size(D,1), size(D,1)/REPLAY_T, D(end,2));
  if D(end,2) < 5
    fprintf(2, 'AVISO: VT final < 5 m/s — possivel crash durante o replay.\n');
  end
  save(fn, 'R', 'VR_lista', 'sinais', 'VR_ganho', 'esc', 'VR_curso_real', 'lims', 'MSL0', 'TRIM_T');
  fprintf('(salvo incremental: %d/%d segmentos em %s)\n', numel(R), numel(VR_lista), fn);
  ii = ii + 1;
end

save(fn, 'R', 'VR_lista', 'sinais', 'VR_ganho', 'esc', 'VR_curso_real', 'lims', 'MSL0', 'TRIM_T');
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

function reload_robusto(n_max)
    % xp_reload_acf com ate' n_max tentativas. Modo de falha visto em
    % 2026-09-08 (3o segmento de um bloco): o dialogo Open Aircraft abre, os
    % cliques nao pegam, o dialogo fica aberto e o XPC para de responder
    % ("No response received"). ESC via Robot fecha o dialogo; tenta de novo.
    global GlobalSocket
    import XPlaneConnect.*
    for tent = 1:n_max
        try
            xp_reload_acf;
            return;
        catch ME
            fprintf(2, 'reload falhou (tentativa %d/%d): %s\n', tent, n_max, strtok(ME.message, newline));
            if tent == n_max, rethrow(ME); end
            try
                % ativa PELO PID: a janela do Plane Maker tambem se chama
                % 'X-System' (2026-09-08) — ver xp_activate.m
                if xp_activate()
                    rb = java.awt.Robot();
                    rb.keyPress(java.awt.event.KeyEvent.VK_ESCAPE); pause(0.1);
                    rb.keyRelease(java.awt.event.KeyEvent.VK_ESCAPE);
                end
            catch
            end
            pause(5);
            try, closeUDP(GlobalSocket); catch, end
            GlobalSocket = openUDP('127.0.0.1', 49009, 0, 500);
        end
    end
end
