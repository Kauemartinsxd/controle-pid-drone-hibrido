% XP_missao.m
% =============================================================
% Lancador de MISSAO POR WAYPOINTS do DH no X-Plane (Fase 3 do
% PLANO_GUIAGEM) — clone do XP_voo.m com a guiagem LOS do
% modelo_XP_DH_GUIA.slx (bloco Guidance_Star, estilo PIPER-1-6).
%
% GANHOS: 100%% os da dissertacao (DH_inicializacao). O retune da
% Fase 0 (XP_retune_Ctheta.m) e' um EXPERIMENTO SEPARADO, opt-in
% via XP_use_Ctheta_XP=true — por decisao do Kaue (2026-08-20) o
% trabalho oficial voa com os ganhos ja definidos.
% O que o lancador ajusta sao ANCORAS DE OPERACAO do .acf v3
% (mesma categoria do que o XP_voo ja sobrescrevia): TrimInput,
% Xe(8), clamp, pitch0/thr0/de0, he/h_ref. Diagnostico 2026-08-20:
% o equilibrio real do .acf v3 a VT 12 e' theta ~5.5 deg / thr
% ~0.85 / de ~+2 deg (voo asa5 + identificacao da Fase 0) — as
% ancoras antigas (Xe(8)=2, TrimInput de=+7.6) deixavam o trim
% fora do centro e o voo caia no 2o regime (VT~10, thr 1.0,
% sink 1.7 m/s ate o chao).
%
% Uso basico:
%   XP_missao                  % circuito oval 280x160 m na proa de engate
%
% Config opcional (defina antes; tudo com default):
%   XP_WPs_frame [Nx4]  WPs no referencial da PROA DE ENGATE:
%                       [a_frente(m) a_direita(m) hMSL(m) vel(m/s)]
%                       (rotacionado p/ NE com a proa lida no engate)
%   XP_WPs_NE    [Nx4]  WPs em NE do ponto de engate (precede o frame)
%   XP_R_accept  [m]    raio de aceitacao (default 80)
%   XP_msl0      [m]    altitude de engate (default 600)
%   XP_VT0       [m/s]  velocidade de engate (default 12)
%   XP_TimeXP    [s]    duracao (default: perimetro/12 x 1.5 + 15)
%   XP_use_Ctheta_XP    true = usa o C_theta re-sintonizado da Fase 0
%   XP_theta_test_t/_final  degrau de theta_ref p/ validacao (inerte)
%   XP_tag       [char] sufixo do nome do .mat salvo
%
% Fim de missao (2026-08-31): a simulacao ENCERRA sozinha 5 s apos o
% aviao entrar no circulo do ULTIMO WP (blocos Cond_fim_missao -> latch
% -> integrador -> Stop nos modelos GUIA; dist calculada de xN/xE e
% WPfim_N/E — independente do dist_mon, que atrasa 1 amostra na troca
% de WP). TimeXP segue como teto de seguranca se o WP nao for atingido.
% Pos-voo: salva xplane/voos/XP_missao_*.mat e gera plot_XP_missao.
% =============================================================

%% Reload automatico do .acf — OPT-IN (decisao do Kaue 2026-08-31: o
%% padrao e' reload MANUAL via File->Open Aircraft; XP_auto_reload=true
%% liga a automacao xp_reload_acf p/ campanhas sem operador).
if exist('XP_auto_reload','var') && ~isempty(XP_auto_reload) && XP_auto_reload
    addpath(fileparts(mfilename('fullpath')));
    xp_reload_acf;
end

%% Config do usuario (preservada atraves do clear do DH_inicializacao)
if ~exist('XP_msl0','var')      || isempty(XP_msl0),      XP_msl0 = 600;   end
if ~exist('XP_VT0','var')       || isempty(XP_VT0),       XP_VT0  = 12;    end
if ~exist('XP_R_accept','var')  || isempty(XP_R_accept),  XP_R_accept = 60; end  % 60 p/ o oval de 6 WPs (80 ate 2026-08-31)
if ~exist('XP_WPs_frame','var'), XP_WPs_frame = []; end
if ~exist('XP_WPs_NE','var'),    XP_WPs_NE    = []; end
if ~exist('XP_TimeXP','var'),    XP_TimeXP    = []; end
if ~exist('XP_use_Ctheta_XP','var') || isempty(XP_use_Ctheta_XP), XP_use_Ctheta_XP = false; end
if ~exist('XP_theta_test_t','var')     || isempty(XP_theta_test_t),     XP_theta_test_t = 1e9; end
if ~exist('XP_theta_test_final','var') || isempty(XP_theta_test_final), XP_theta_test_final = 0; end
if ~exist('XP_tag','var'),       XP_tag = ''; end
if isempty(XP_WPs_frame) && isempty(XP_WPs_NE)
    % default = circuito OVAL "stadium" de 6 WPs: retas de 160 m + pontas
    % semicirculares de raio 100 (folgado vs raio natural ~83 m a 12 m/s),
    % WP no apice. Perimetro 884 m -> ~126 s, cabe nos ~130 s de motor por
    % reload (2026-08-31; o quadrado historico de 500 m dava 265 s e virava
    % planeio na metade — voos de ago/2026 em voos/). R_accept default 60
    % (espacamento minimo 141 m > 2R; validado no SIL: R50/pontas de 80
    % perdia WP3 por 2 m).
    XP_WPs_frame = [ 160    0  XP_msl0  12;
                     260  100  XP_msl0  12;
                     160  200  XP_msl0  12;
                       0  200  XP_msl0  12;
                    -100  100  XP_msl0  12;
                       0    0  XP_msl0  12];
end
setpref('XP_DH','missao', struct('msl0',XP_msl0,'VT0',XP_VT0,'R',XP_R_accept, ...
    'WPf',XP_WPs_frame,'WPne',XP_WPs_NE,'T',XP_TimeXP,'useCXP',XP_use_Ctheta_XP, ...
    'tht',XP_theta_test_t,'thf',XP_theta_test_final,'tag',XP_tag));

%% 1) Workspace completo (ganhos da dissertacao, socket, refs)
run(fullfile(fileparts(mfilename('fullpath')), 'XP_inicializacao.m'));

cfgm = getpref('XP_DH','missao');
rmpref('XP_DH','missao');
XP_msl0 = cfgm.msl0; XP_VT0 = cfgm.VT0; XP_R_accept = cfgm.R;

%% 2) PRE-FLIGHT: X-Plane rodando e aeronave recuperavel (solo OU voo sao)
import XPlaneConnect.*
global GlobalSocket
r0 = double(getDREFs({'sim/flightmodel/position/elevation', ...
                      'sim/flightmodel/position/y_agl', ...
                      'sim/flightmodel/position/theta', ...
                      'sim/flightmodel/position/phi', ...
                      'sim/flightmodel/position/true_airspeed', ...
                      'sim/time/total_flight_time_sec', ...
                      'sim/flightmodel/position/psi'}, GlobalSocket));
pause(0.6);
t2 = double(getDREFs({'sim/time/total_flight_time_sec'}, GlobalSocket));
if t2 <= r0(6)
    error(['XP_missao: tempo do X-Plane CONGELADO (tela de crash/pausa). ' ...
        'Faca Reset Flight no X-Plane e rode de novo.']);
end
% O teleporte-no-engate (xp_read_dh cmd==1) corrige posicao, atitude,
% velocidade e taxas a partir de QUALQUER estado com a fisica rodando —
% inclusive espiral pos-corrida (o X-Plane mantem o ultimo comando).
% Por isso o pre-flight so bloqueia simulador travado; o resto e' aviso.
no_solo = (r0(2) < 3) && (r0(5) < 3);
if ~no_solo
    fprintf(['XP_missao: engatando A PARTIR DE VOO (AGL=%.1f m, VT=%.1f m/s, ' ...
        'phi=%.1f deg) — o teleporte normaliza o estado.\n'], r0(2), r0(5), r0(4));
end
psi_engate = r0(7);                       % [deg] proa mantida no teleporte
ground_msl = r0(1) - r0(2);

% NOTA DE POTENCIA (2026-08-29): o .acf com pmax=1226 W entrega T ate
% ~30 N; o modelo da dissertacao tem T_max = 14 N (F = 14*dt). Com o
% dobro do ganho no canal de throttle o C_vel (intocado) oscila em
% ciclo-limite (thr 0.4<->1.0, periodo ~3 s) — voa e segura h, mas o
% ideal e' casar a potencia NO PLANE MAKER (power 1.6 -> ~0.8 hp;
% validar T_max ~+14 N a 12 m/s via POINT_thrust). NAO escrever
% acf_pmax via dref com o sim rodando: derruba o latch do motor
% eletrico do XP9 (helice morta/travada — visto em 2026-08-29).
if XP_msl0 - ground_msl < 120
    warning('Solo local em %.0f m MSL: subindo engate p/ %.0f m MSL (150 m AGL).', ...
        ground_msl, ground_msl+150);
    XP_msl0 = ground_msl + 150;
end
estados_pf = {'em voo', 'no solo'};
fprintf('XP_missao: pre-flight OK (%s, proa de engate %.0f deg).\n', ...
    estados_pf{no_solo+1}, psi_engate);

%% 3) Ancoras de operacao do .acf v3 (NAO sao ganhos do controlador)
% Ponto de regime medido (asa5 + ident Fase 0): theta~5.5, thr~0.85, de~+2
% ===== ANCORAS DO GEMEO v1 (2026-08-30, ver EQUIVALENCIA_ACF.md) =====
% O .acf agora e' equivalente ao drone real: o trim medido em voo
% (alpha 14.50, de +6.99, thr 0.43) coincide com o Ue/Xe da propria
% dissertacao — as ancoras voltam a ser "os numeros do projeto".
XP_thr0   = 0.43;                 % throttle de trim do gemeo
XP_de0    = deg2rad(7.0)/deg2rad(25);  % profundor de trim normalizado
XP_pitch0 = 14;                   % atitude de trim [deg]
TrimInput = [0.43; deg2rad(7.0); 0; 0];   % feedforward = trim do gemeo
Xe(8) = deg2rad(14.3);            % centro de theta_ref = theta de trim
theta_ref_clamp = [-0.1745  deg2rad(3)];  % theta_ref em [4.3, 17.3] deg:
% teto 17.3 fica ~1 deg abaixo do estol do gemeo (~18.5); piso 4.3 da
% gamma de descida. (Ancoras antigas do .acf pre-Fase B: thr0 0.55,
% de0 +2, pitch0 2, Xe8 2 — validas so p/ o backup _preFaseB.)
h_ref  = XP_msl0;                 % refs/bias no ponto de engate (b=0)
he     = XP_msl0;
VT_ref = Ve;                      % 12 m/s — ponto de projeto

if cfgm.useCXP
    % EXPERIMENTO Fase 0 (fora do trabalho oficial): C_theta re-sintonizado
    % p/ a planta identificada do X-Plane — ver XP_retune_Ctheta.m
    C_theta.Kp = 1.6; C_theta.Ki = 0.9; C_theta.Kd = 0; C_theta.N = 100;
    fprintf('XP_missao: *** C_theta_XP (retune Fase 0) ATIVO — experimento, nao oficial ***\n');
end

%% 4) Missao: WPs em NE do engate + interpolacao de Dh > 20 m
if ~isempty(cfgm.WPne)
    WPs_user = cfgm.WPne;
else
    cpsi = cosd(psi_engate); spsi = sind(psi_engate);
    WPs_user = cfgm.WPf;
    ne = [cpsi -spsi; spsi cpsi] * cfgm.WPf(:,1:2)';   % frame -> NE
    WPs_user(:,1:2) = ne';
end
% WP1 vira o 1o ALVO (wp_idx nasce em 1); interpolacao de altitude (Julio)
max_dalt = 20;
WPs = WPs_user(1,:);
for i = 2:size(WPs_user,1)
    dalt = abs(WPs_user(i,3) - WPs_user(i-1,3));
    if dalt > max_dalt
        n_sub = ceil(dalt / max_dalt);
        for k = 1:n_sub-1
            WPs(end+1,:) = WPs_user(i-1,:) + (k/n_sub)*(WPs_user(i,:) - WPs_user(i-1,:)); %#ok<SAGROW>
        end
    end
    WPs(end+1,:) = WPs_user(i,:); %#ok<SAGROW>
end
R_accept = XP_R_accept;
N_WPs = size(WPs,1);   % p/ o corte de fim de missao no modelo (Cond_fim_missao)
WPfim_N = WPs(end,1);  WPfim_E = WPs(end,2);   % ultimo WP (dist independente do dist_mon)
h_ref0 = h_ref; VT_ref0 = VT_ref;          % base dos deltas do Guidance_Star
theta_test_t = cfgm.tht; theta_test_init = 0; theta_test_final = cfgm.thf;

% duracao: perimetro (engate->WP1->...->WPn) a 12 m/s x 1.5 + 15 s
per = norm(WPs(1,1:2));
for i = 2:size(WPs,1), per = per + norm(WPs(i,1:2)-WPs(i-1,1:2)); end
TimeXP = cfgm.T;
if isempty(TimeXP), TimeXP = ceil(per/12*1.5 + 15); end

fprintf('XP_missao: %d WPs (%d apos interpolacao), perimetro %.0f m, TimeXP %.0f s, R_accept %.0f m.\n', ...
    size(WPs_user,1), size(WPs,1), per, TimeXP, R_accept);

%% 5) Compila o modelo GUIA e ARMA o teleporte-no-engate
mdl = 'modelo_XP_DH_GUIA';
load_system(mdl);
set_param(mdl, 'SimulationCommand', 'update');

global XP_IC
XP_IC = struct('target_msl', XP_msl0, 'h0_agl', XP_msl0-ground_msl, ...
               'VT0', XP_VT0, 'psi0', NaN, 'thr0', XP_thr0, ...
               'de0', XP_de0, 'pitch0', XP_pitch0);
fprintf('XP_missao: teleporte ARMADO (MSL %.0f m, %.1f m/s, proa atual). Engatando...\n', ...
    XP_msl0, XP_VT0);

%% 6) Voa a missao
out = sim(mdl);

%% 7) Salva o voo
voosDir = fullfile(rootDir, 'xplane', 'voos');
if ~isfolder(voosDir), mkdir(voosDir); end
voo = struct();
voo.quando     = datestr(now, 'yyyy-mm-dd HH:MM:SS');
voo.Y          = squeeze(out.Y_xp);      % 13 x T (ver xp_read_dh)
voo.U          = out.U_xp;               % T x 4:  [thr de da dr]
voo.t          = out.tout(:);
voo.t_xplane   = out.t_xplane_log(:);
voo.wp_idx     = out.wp_idx_log(:);
voo.dist_wp    = out.dist_log(:);
voo.WPs        = WPs;
voo.WPs_user   = WPs_user;
voo.R_accept   = R_accept;
voo.psi_engate = psi_engate;
try, voo.theta_ref = out.theta_ref; catch, end
voo.cfg = struct('XP_msl0',XP_msl0, 'XP_VT0',XP_VT0, 'TimeXP',TimeXP, ...
    'h_ref',h_ref, 'he',he, 'VT_ref',VT_ref, 'TrimInput',TrimInput, ...
    'C_theta',C_theta, 'C_vel',C_vel, 'C_alt',C_alt, 'C_phi',C_phi, ...
    'Kq',Kq, 'Kp',Kp, 'Kr',Kr, 'K_heading',K_heading, ...
    'theta_ref_clamp',theta_ref_clamp, 'tau_ref',tau_ref, ...
    'Xe8',Xe(8), 'use_Ctheta_XP',cfgm.useCXP, ...
    'theta_test',[cfgm.tht cfgm.thf], 'act',act, 'eng',eng);
tag = cfgm.tag; if ~isempty(tag), tag = ['_' tag]; end
vooFile = fullfile(voosDir, ['XP_missao_' datestr(now,'yyyymmdd_HHMMSS') tag '.mat']);
save(vooFile, 'voo');
fprintf('Missao salva em: %s\n', vooFile);

%% 8) Resumo + capturas
Y = voo.Y; U = voo.U; t = voo.t;
dtx = voo.t_xplane(end) - voo.t_xplane(1);
fprintf('\n===== XP_missao: resumo =====\n');
fprintf('Sync : sim %.1f s | X-Plane avancou %.1f s | razao %.3f\n', t(end), dtx, dtx/max(t(end),eps));
fprintf('VT   : min %.1f | max %.1f m/s (ref %.1f) | h: min %.1f | max %.1f m (WP1 %.1f)\n', ...
    min(Y(1,:)), max(Y(1,:)), VT_ref, min(Y(8,:)), max(Y(8,:)), WPs(1,3));
fprintf('phi  : max |%.1f| deg | theta: %.1f a %.1f deg\n', ...
    rad2deg(max(abs(Y(5,:)))), rad2deg(min(Y(6,:))), rad2deg(max(Y(6,:))));
fprintf('thr  : %.2f a %.2f | de: %+.1f a %+.1f deg\n', ...
    min(U(:,1)), max(U(:,1)), rad2deg(min(U(:,2))), rad2deg(max(U(:,2))));
nWP = size(WPs,1);
rotulos = {'-- fora --', 'CAPTURADO'};
for i = 1:nWP
    dmin = min(sqrt((Y(11,:) - WPs(i,1)).^2 + (Y(12,:) - WPs(i,2)).^2));
    hit = dmin <= R_accept;
    fprintf('WP%d (N %+7.1f, E %+7.1f): dist min %6.1f m  %s\n', ...
        i, WPs(i,1), WPs(i,2), dmin, rotulos{hit+1});
end
fprintf('wp_idx final: %d de %d | Figuras: plot_XP_missao\n', voo.wp_idx(end), nWP);
plot_XP_missao(voo, vooFile);

%% 9) MESMA missao no modelo NL da Ana (comparacao automatica SILxXP)
% Os WPs (em NE do engate) voltam ao referencial da PROA DE ENGATE —
% que e' o proprio NE do SIL (engate na origem com psi=0).
try
    cpsi2 = cosd(voo.psi_engate); spsi2 = sind(voo.psi_engate);
    wpF = voo.WPs_user;
    wpF(:,1:2) = [ voo.WPs_user(:,1)*cpsi2 + voo.WPs_user(:,2)*spsi2, ...
                   voo.WPs_user(:,2)*cpsi2 - voo.WPs_user(:,1)*spsi2 ];
    NL_WPs = wpF; NL_R_accept = voo.R_accept; NL_TimeXP = voo.cfg.TimeXP;
    NL_tag = 'autoNL';
    vooFileXP = vooFile;                       % sobrevive? NL_missao faz clear —
    setpref('XP_DH','xpcomp', vooFileXP);      % preserva via pref
    fprintf('\n===== Repetindo a missao no modelo NL (sem X-Plane)... =====\n');
    run(fullfile(fileparts(mfilename('fullpath')), 'NL_missao.m'));
    vooNL = voo;                               % voo do NL (workspace pos-NL_missao)
    vooFileXP = getpref('XP_DH','xpcomp'); rmpref('XP_DH','xpcomp');
    dXP = load(vooFileXP); voo = dXP.voo;      % restaura o voo XP como 'voo'
    plot_missao_comparada(voo, vooNL, vooFileXP);
catch e_cmp
    fprintf('comparacao NL automatica falhou (%s) — voo X-Plane salvo normalmente.\n', e_cmp.message);
end
