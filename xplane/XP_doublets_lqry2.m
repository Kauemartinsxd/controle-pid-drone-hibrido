% XP_doublets_lqry2.m
% =============================================================
% LQRy v2 do Mirko (lqry_mirko_atualizado, CL_NL_DH_18_jun_2026) no GEMEO
% do X-Plane 9: VOO RETO com os MESMOS doublets do modelo dele —
% V_T +3 m/s (10-20 s), H e psi com amplitude configuravel — em agenda
% COMPRIMIDA p/ caber na energia do motor eletrico do XP9 (~130 s/reload):
%   original -> nova: 40->28, 60->41, 80->54, 90->62, 110->75, 130->88
% (a MESMA agenda e' aplicada ao SIL por lqry_sim_sched p/ a sobreposicao).
%
% Derivado de XP_missao_lqry2.m (pre-flight, ancoras, ICs bumpless e
% Planta_XP identicos; ganhos/estrutura do Mirko INTOCADOS). Diferencas:
%   - NAO faz clearvars/restoredefaultpath (sessao preservada; path
%     restaurado no fim);
%   - guiagem neutralizada: 2 WPs na proa do engate a 10 e 20 km
%     (LOS = proa constante; nenhum WP e' capturado em 100 s);
%   - Steps retemporizados EM MEMORIA (o .slx NAO e' salvo);
%   - salva tambem os logs *_NL (mesmos To Workspace do SIL);
%   - reload automatico do .acf (xp_reload_acf) — autorizado pelo Kaue
%     em 2026-09-01 para esta campanha.
%
% Config (defina antes de rodar; defaults entre parenteses):
%   XP_dbl_VT       12 -> i=2 (V12_H600) | 15 -> i=5 (V15_H600)     (15)
%   XP_dbl_refAlt   amplitude do doublet de H [m]                    (10)
%   XP_dbl_refPsi   amplitude do doublet de psi [deg]                (10)
%   XP_dbl_prot     alpha-protection na Planta_XP: 0 = LQRy PURO     (0)
%   XP_dbl_clamp    clamp de theta_ref [0, 17.3 deg]: 0 = LQRy PURO  (0)
%   XP_dbl_T        duracao da corrida [s]                           (100)
%   XP_dbl_tag      sufixo do arquivo em voos/                       ('DBL10')
%   XP_dbl_reload   reload automatico antes do voo                   (1)
%   XP_dbl_thr0 / XP_dbl_de0_deg   ancoras de engate (default = as do
%                   XP_missao_lqry2: i=2 -> 0.45 / +5.50; senao trim da planta)
% =============================================================

%% 0) Config
if ~exist('XP_dbl_VT','var')     || isempty(XP_dbl_VT),     XP_dbl_VT = 15;      end
if ~exist('XP_dbl_refAlt','var') || isempty(XP_dbl_refAlt), XP_dbl_refAlt = 10;  end
if ~exist('XP_dbl_refPsi','var') || isempty(XP_dbl_refPsi), XP_dbl_refPsi = 10;  end
if ~exist('XP_dbl_prot','var')   || isempty(XP_dbl_prot),   XP_dbl_prot = 0;     end
if ~exist('XP_dbl_clamp','var')  || isempty(XP_dbl_clamp),  XP_dbl_clamp = 0;    end
if ~exist('XP_dbl_T','var')      || isempty(XP_dbl_T),      XP_dbl_T = 100;      end
if ~exist('XP_dbl_tag','var')    || isempty(XP_dbl_tag),    XP_dbl_tag = 'DBL10'; end
if ~exist('XP_dbl_reload','var') || isempty(XP_dbl_reload), XP_dbl_reload = 1;   end
% modo lateral: 0 = psi Hold (Caso 4) | 1 = phi Hold (Caso 2; guiagem de bank
% DESLIGADA via K_bank_guia = 0 -> so' o doublet de phi atua, como no SIL)
if ~exist('XP_dbl_phi_psi','var') || isempty(XP_dbl_phi_psi), XP_dbl_phi_psi = 0; end
if ~exist('XP_dbl_refPhi','var')  || isempty(XP_dbl_refPhi),  XP_dbl_refPhi = 0;  end
% taxa do laco X-Plane <-> Simulink (blocos read_xp/send_xp): o .slx traz
% 0.05 s (20 Hz). Exigencia do Mirko: >= 100 Hz. Medido 2026-09-02: RTT do
% getDREFs 2,5 ms e fisica do XP9 a ~400 fps => 100 Hz e' viavel. O ajuste
% e' feito EM MEMORIA (slx intacto).
if ~exist('XP_dbl_Ts_io','var')   || isempty(XP_dbl_Ts_io),   XP_dbl_Ts_io = 0.01; end
% aquecimento pos-teleporte [s] (0 = engate imediato, como antes): trim
% classico segura a aeronave ate o motor entrar em regime; o trim real vira
% offset sobre as ancoras (ADENDO 10). Convergencia encerra antes do maximo.
if ~exist('XP_dbl_warmup','var')  || isempty(XP_dbl_warmup),  XP_dbl_warmup = 0;   end   % 0 = engate imediato (padrao historico)
% 1 = aquecimento SIMPLES (manete fixa na ancora, theta = pitch0; so' spool do
% motor) | 0 = busca de trim por janelas (gasta energia do motor; nao convergiu)
if ~exist('XP_dbl_warm_simple','var') || isempty(XP_dbl_warm_simple), XP_dbl_warm_simple = 1; end
XP_dbl_sched = [40 28; 60 41; 80 54; 90 62; 110 75; 130 88];
XP_dbl_msl0  = 600;

%% 1) Paths (sem restoredefaultpath) + socket
XP_dbl_path0 = path;
xpDir  = 'C:\Users\kaue\Documents\Dissertacao_Mestrado\controle-pid-drone-hibrido\xplane';
raizN  = 'C:\Users\kaue\Documents\Dissertacao_Mestrado\lqry_mirko_atualizado\Nova pasta';
julioX = fullfile('C:\Users\kaue\Documents\Dissertacao_Mestrado', 'trabalho_julio', ...
    'PIPER-1-6-roll_back', 'PIPER-1-6-roll_back', 'xplane', 'XPlaneConnect-master', 'MATLAB');
addpath(xpDir); addpath(julioX);
import XPlaneConnect.*
clear global XP_IC
clear xp_read_dh xp_send_dh
global GlobalSocket
try, closeUDP(GlobalSocket); catch, end
GlobalSocket = openUDP('127.0.0.1', 49009, 0, 500);

%% 2) Reload do .acf (rearma a energia do motor)
if XP_dbl_reload
    xp_reload_acf;          % reabre GlobalSocket e verifica motor VIVO
    pause(1.0);
end

%% 3) Ganhos e trims do LQRy atualizado (INTOCADOS; cells 1x9)
load(fullfile(raizN, 'Ganho_hold_theta.mat'));
load(fullfile(raizN, 'Ganho_hold_H.mat'));
load(fullfile(raizN, 'Ganho_hold_VT.mat'));
load(fullfile(raizN, 'Ganho_hold_phi.mat'));
load(fullfile(raizN, 'Ganho_hold_psi.mat'));
load(fullfile(raizN, 'Dados_Trim.mat'));
i = find([Plantas.Ve] == XP_dbl_VT & [Plantas.He] == 600, 1);
assert(~isempty(i), 'XP_doublets_lqry2: nao ha planta para Ve=%g @ 600 m', XP_dbl_VT);
i_lat = i;
fprintf('XP_doublets_lqry2: planta i=%d (%s) | doublets H %g m / psi %g deg | prot=%d clamp=%d\n', ...
    i, Plantas(i).nome, XP_dbl_refAlt, XP_dbl_refPsi, XP_dbl_prot, XP_dbl_clamp);

%% 4) Vars do modelo (como no XP_missao_lqry2)
Ts = 1/100; surfaces = 24; Variacao_Iner = 0;
% VT_Throttle: 1 = Velocity Hold (Casos 1-4 da Tab. 11) | 0 = throttle FIXO no
% trim (Casos 5-8) — teste 2026-09-02: isola a malha de velocidade (bang-bang
% do throttle no motor do XP9) do resto.
if ~exist('XP_dbl_VT_Throttle','var') || isempty(XP_dbl_VT_Throttle), XP_dbl_VT_Throttle = 1; end
VT_Throttle = XP_dbl_VT_Throttle; att_alt = 0; phi_psi = XP_dbl_phi_psi;
refPhi = XP_dbl_refPhi; reftheta = 0;
refAlt = XP_dbl_refAlt; refPsi = XP_dbl_refPsi;
refVel = XP_dbl_VT;
K_bank_guia  = 0.1975; phi_max_guia = deg2rad(20);
if phi_psi == 1, K_bank_guia = 0; end     % bank hold puro: sem bank-to-turn da guiagem
if XP_dbl_clamp, XP_clamp_lqry = deg2rad([0 17.3]); else, XP_clamp_lqry = [-pi pi]; end
prot_on = XP_dbl_prot; alpha_prot = deg2rad(16);

%% 5) PRE-FLIGHT
r0 = double(getDREFs({'sim/flightmodel/position/elevation', ...
    'sim/flightmodel/position/y_agl', 'sim/flightmodel/position/theta', ...
    'sim/flightmodel/position/phi', 'sim/flightmodel/position/true_airspeed', ...
    'sim/time/total_flight_time_sec', 'sim/flightmodel/position/psi'}, GlobalSocket));
pause(0.6);
t2 = double(getDREFs({'sim/time/total_flight_time_sec'}, GlobalSocket));
if t2 <= r0(6)
    error('XP_doublets_lqry2: tempo do X-Plane CONGELADO. Reset/despause e rode de novo.');
end
q = getDREFs({'sim/flightmodel/engine/ENGN_TRQ'}, GlobalSocket); trq_pf = double(q(1));
if r0(2) < 3 && abs(trq_pf(1)) < 0.02
    sendCTRL([0,0,0,0.4,-998,-998], 0, GlobalSocket); pause(1.0);
    q = getDREFs({'sim/flightmodel/engine/ENGN_TRQ'}, GlobalSocket); trq_pf = double(q(1));
    sendCTRL([0,0,0,0,-998,-998], 0, GlobalSocket);
    if abs(trq_pf(1)) < 0.02
        warning('MOTOR MORTO (TRQ=0). Reload nao pegou — rode de novo.');
    end
end
psi_engate = r0(7);
ground_msl = r0(1) - r0(2);
XP_msl0 = XP_dbl_msl0;
if XP_msl0 - ground_msl < 120
    XP_msl0 = ground_msl + 150;
    warning('Solo alto: engate movido p/ %.0f m MSL.', XP_msl0);
end
fprintf('XP_doublets_lqry2: pre-flight OK (proa %.0f deg, solo %.0f m MSL, TRQ %.2f).\n', psi_engate, ground_msl, trq_pf(1));

%% 6) Ancoras de engate + ICs bumpless (identico ao XP_missao_lqry2)
alpha0 = atan2(double(Plantas(i).Xe(3)), double(Plantas(i).Xe(1)));
XP_thr0 = 0.45; XP_de0_dg = 5.50;                 % trim REAL do gemeo a 12 m/s
if i ~= 2
    XP_thr0   = double(Plantas(i).Ue(1));
    XP_de0_dg = rad2deg(double(Plantas(i).Ue(2)));
end
if exist('XP_dbl_thr0','var')    && ~isempty(XP_dbl_thr0),    XP_thr0   = XP_dbl_thr0;    end
if exist('XP_dbl_de0_deg','var') && ~isempty(XP_dbl_de0_deg), XP_de0_dg = XP_dbl_de0_deg; end
XP_pitch0 = rad2deg(alpha0);
% atitude de engate do gemeo (override): o trim do gemeo a 15 m/s, medido pelos
% aquecimentos de 2026-09-02, e' de ~+8,5 deg / thr ~0,45 / theta ~9 deg — nao
% os +2,2 / 0,337 / 8 da Ana; engatar nas ancoras da Ana da' pitch-down de
% -25 deg/s no 1o segundo e o LQRY entra na armadilha (ADENDO 11).
if exist('XP_dbl_pitch0','var') && ~isempty(XP_dbl_pitch0), XP_pitch0 = XP_dbl_pitch0; end
XP_h_ref0 = XP_msl0;
theta0    = alpha0;
x4        = [XP_dbl_VT; alpha0; 0; theta0];
XP_U_trim4 = [XP_thr0; deg2rad(XP_de0_dg); 0; 0];
de_trim    = deg2rad(XP_de0_dg);
GsS = double(GstateLong_speed{i}); GiS = double(GintLong_speed{i});
GsT = double(GstateLong{i});       GiT = double(GintLong{i});
GsA = double(GstateLong_Alt{i});   GiA = double(GintLong_Alt{i});
% ICs dos integradores: o controlador v2 trabalha em DESVIOS (TrimConst* subtrai
% Xe(1)/alpha_e/theta_e) e a Planta_XP soma XP_U_trim4 = ancoras. No SIL do
% Mirko os integradores partem de ZERO e o engate e' bumpless. A formula do
% XP_missao_lqry2 (estados ABSOLUTOS em x4) pre-carregava -23.6/-0.106/-4.02,
% i.e. +1182% de throttle, +66 deg de profundor e +46 deg de theta_ref no
% engate (medido nos voos DBL10 de 2026-09-01 17:38/17:40) -> corrigido:
% desvios nulos no engate => ICs = 0, identico ao SIL.
thr_delta0 = XP_thr0 - XP_U_trim4(1);            % = 0 (trim somado = ancoras)
de_delta0  = XP_de0_dg - rad2deg(XP_U_trim4(2)); % = 0
x4_dev     = zeros(4,1);                         % desvios no engate (VT/alpha/q/theta)
XP_IC_int_speed = double((thr_delta0*100 - GsS*x4_dev) / GiS);
XP_IC_int_theta = double((de_delta0*(1 - GsT(5)) - GsT(1:4)*x4_dev) / GiT);
XP_IC_int_alt   = double((0 - GsA*[x4_dev; 0]) / GiA);
fprintf('ancoras: alpha/theta %.1f deg | de %+.2f deg | thr %.3f | ICs %.3f %.4f %.4f\n', ...
    rad2deg(alpha0), XP_de0_dg, XP_thr0, XP_IC_int_speed, XP_IC_int_theta, XP_IC_int_alt);

%% 7) "Missao": 2 WPs na proa do engate (guiagem neutra)
cpsi = cosd(psi_engate); spsi = sind(psi_engate);
WPs_user = [10000 0 XP_msl0 XP_dbl_VT; 20000 0 XP_msl0 XP_dbl_VT];
ne = [cpsi -spsi; spsi cpsi] * WPs_user(:,1:2)';
WPs = WPs_user; WPs(:,1:2) = ne';
R_accept = 80; h_ref0 = XP_h_ref0; VT_ref0 = XP_dbl_VT;
TimeXP = XP_dbl_T;

%% 8) Compila, retemporiza os Steps EM MEMORIA e ARMA o teleporte
mdl = 'modelo_XP_LQRY2_GUIA';
if bdIsLoaded(mdl), bdclose(mdl); end
load_system(fullfile(xpDir, [mdl '.slx']));
stp = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Step');
n_re = 0;
for k = 1:numel(stp)
    tm = str2double(get_param(stp{k}, 'Time'));
    j = find(XP_dbl_sched(:,1) == tm, 1);
    if ~isempty(j), set_param(stp{k}, 'Time', num2str(XP_dbl_sched(j,2))); n_re = n_re + 1; end
end
fprintf('XP_doublets_lqry2: %d Steps retemporizados (agenda comprimida), TimeXP %g s.\n', n_re, TimeXP);
% Taxa do laco: read_xp/send_xp sao MATLAB Function blocks (charts
% Stateflow.EMChart) com sampleTime fixo no slx; ajusta em memoria.
rt = sfroot;
for nm = {'read_xp', 'send_xp'}
    ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/Planta/' nm{1}]);
    assert(~isempty(ch), 'XP_doublets_lqry2: chart %s nao encontrado', nm{1});
    ch.SampleTime = num2str(XP_dbl_Ts_io);
    % O script do chart tem "Ts = 0.05;" HARDCODED como divisor (so' chama
    % o X-Plane quando floor(t/Ts) muda) — sem trocar isto o sample time do
    % bloco nao muda nada (voo _103342 de 2026-09-02: 10001 amostras, 2001
    % distintas).
    scr = ch.Script;
    assert(contains(scr, 'Ts = 0.05;'), 'XP_doublets_lqry2: "Ts = 0.05;" nao achado no script de %s', nm{1});
    ch.Script = strrep(scr, 'Ts = 0.05;', sprintf('Ts = %g;', XP_dbl_Ts_io));
end
fprintf('XP_doublets_lqry2: laco X-Plane a %.0f Hz (Ts_io = %g s; controlador ode4 a %.0f Hz).\n', ...
    1/XP_dbl_Ts_io, XP_dbl_Ts_io, 1/Ts);
% ANTI-WINDUP (ADENDO 11, 2026-09-02): o motor do XP9 tem tau ~3,5 s (medido) e
% o atuador de manete do modelo do Mirko tem 0,1 s -> o hold de VT satura o
% throttle, os 3 integradores enrolam e o profundor crava (ADENDO 10). No SIL,
% LIMITAR a saida dos integradores ao alcance util do atuador (clamping) resgata
% o controlador ate tau = 5 s sem tocar em nenhum ganho. Feito EM MEMORIA:
%   Integrator3 (VT): |GiS*x| <= 100 %   Integrator (theta): |GiT*x| <= 25 deg
%   Integrator2 (H):  |GiA*x| <= 20 deg de theta_ref
if ~exist('XP_dbl_antiwindup','var') || isempty(XP_dbl_antiwindup), XP_dbl_antiwindup = 0; end   % 0 = controlador do Mirko intacto (padrao)
% GUIAGEM DE PROA DESLIGADA no modo doublets (ADENDO 11, bug 2): o chart
% Guidance_Star soma PsiRefGuia = psi_rel + wrap(psi_los - psi_abs) a referencia
% do psi Hold; os WPs ficam na proa PRE-voo e a proa real apos teleporte/
% aquecimento difere ~4 deg => o psi Hold nasce com erro e pede phi_ref -58 deg
% em 1 s. Em voo reto + doublets a referencia deve ser SO o Step (como no SIL).
% Feito em memoria: Goto_PsiRefGuia passa a receber Constante 0.
if ~exist('XP_dbl_guia_psi_off','var') || isempty(XP_dbl_guia_psi_off), XP_dbl_guia_psi_off = 1; end
if XP_dbl_guia_psi_off
    ph_g = get_param([mdl '/Goto_PsiRefGuia'], 'PortHandles'); l_g = get_param(ph_g.Inport(1), 'Line');
    src_g = get_param(l_g, 'SrcPortHandle'); delete_line(l_g);
    pos_g = get_param([mdl '/Goto_PsiRefGuia'], 'Position');
    add_block('simulink/Sources/Constant', [mdl '/C_psi_guia_off'], 'Value', '0', 'Position', pos_g + [-120 0 -120 0]);
    add_line(mdl, 'C_psi_guia_off/1', 'Goto_PsiRefGuia/1');
    add_block('simulink/Sinks/Terminator', [mdl '/T_psi_guia_off'], 'Position', pos_g + [0 60 0 60]);
    add_line(mdl, src_g, get_param([mdl '/T_psi_guia_off'], 'PortHandles').Inport(1));
    fprintf('XP_doublets_lqry2: PsiRefGuia = 0 (guiagem de proa desligada; psi_ref = so o Step).\n');
end
if XP_dbl_antiwindup
    limS = 100/max(abs(GiS(:))); limT = deg2rad(25)/max(abs(GiT(:))); limA = deg2rad(20)/max(abs(GiA(:)));
    set_param([mdl '/Integrator3'], 'LimitOutput','on', 'UpperSaturationLimit', num2str(limS), 'LowerSaturationLimit', num2str(-limS));
    set_param([mdl '/Integrator'],  'LimitOutput','on', 'UpperSaturationLimit', num2str(limT), 'LowerSaturationLimit', num2str(-limT));
    set_param([mdl '/Integrator2'], 'LimitOutput','on', 'UpperSaturationLimit', num2str(limA), 'LowerSaturationLimit', num2str(-limA));
    fprintf('XP_doublets_lqry2: ANTI-WINDUP (clamp) nos integradores: VT +-%.3g | theta +-%.3g | H +-%.3g\n', limS, limT, limA);
end
set_param(mdl, 'SimulationCommand', 'update');
global XP_IC XP_TRIM_DELTA XP_TRIM_FOUND
XP_TRIM_DELTA = []; XP_TRIM_FOUND = [];
XP_IC = struct('target_msl', XP_msl0, 'h0_agl', XP_msl0 - ground_msl, ...
               'VT0', XP_dbl_VT, 'psi0', NaN, 'thr0', XP_thr0, ...
               'de0', deg2rad(XP_de0_dg)/deg2rad(25), 'pitch0', XP_pitch0, ...
               'warmup', XP_dbl_warmup, 'warm_simple', XP_dbl_warm_simple);
fprintf('XP_doublets_lqry2: teleporte ARMADO. Engatando...\n');

%% 9) Voa
out = sim(mdl);

%% 10) Salva (voo + logs *_NL) — o modelo retemporizado e' DESCARTADO
voosDir = fullfile(xpDir, 'voos');
voo = struct();
voo.quando   = datestr(now, 'yyyy-mm-dd HH:MM:SS');
voo.Y        = squeeze(out.Y_xp);
voo.U        = squeeze(out.U_xp);
if size(voo.U,1) == 4 && size(voo.U,2) > 4, voo.U = voo.U.'; end
fatU = max(1, round((size(voo.U,1)-1)/(size(voo.Y,2)-1)));
voo.U        = voo.U(1:fatU:end, :);
voo.t        = (0:size(voo.Y,2)-1)'*XP_dbl_Ts_io;
voo.Ts_io    = XP_dbl_Ts_io;
voo.t_xplane = out.t_xplane_log(:);
voo.WPs = WPs; voo.psi_engate = psi_engate; voo.h_ref0 = XP_h_ref0;
voo.sched = XP_dbl_sched;
voo.NL = struct();
for nm = {'theta_NL','q_NL','elev_NL','VT_NL','Throttle_NL','H_NL','phi_NL','p_NL','ail_NL','psi_NL','r_NL','rud_NL','beta_NL'}
    try
        v = out.get(nm{1});
        voo.NL.(nm{1}) = struct('time', v.time, 'values', v.signals.values);
    catch
        warning('log %s ausente no modelo XP', nm{1});
    end
end
voo.cfg = struct('controlador', 'LQRY2 (lqry_mirko_atualizado, 18-jun-2026) — doublets voo reto', ...
    'i_planta', i, 'planta_nome', Plantas(i).nome, 'VT', XP_dbl_VT, 'Ts_io', XP_dbl_Ts_io, ...
    'warmup', XP_dbl_warmup, 'trim_found', XP_TRIM_FOUND, 'trim_delta', XP_TRIM_DELTA, 'antiwindup', XP_dbl_antiwindup, 'guia_psi_off', XP_dbl_guia_psi_off, 'pitch0', XP_pitch0, ...
    'refAlt', refAlt, 'refPsi', refPsi, 'refPhi', refPhi, 'prot_on', prot_on, 'clamp', XP_clamp_lqry, ...
    'U_trim4', XP_U_trim4, 'XP_msl0', XP_msl0, 'TimeXP', TimeXP, ...
    'ancoras', [XP_thr0 XP_de0_dg rad2deg(alpha0)], ...
    'ICs', [XP_IC_int_speed XP_IC_int_theta XP_IC_int_alt], ...
    'VT_Throttle', VT_Throttle, 'phi_psi', phi_psi, 'att_alt', att_alt, ...
    'Ue', Plantas(i).Ue, 'Xe', Plantas(i).Xe);
vooFile = fullfile(voosDir, ['XP_dbl_' datestr(now,'yyyymmdd_HHMMSS') '_' XP_dbl_tag '.mat']);
save(vooFile, 'voo');
XP_dbl_lastfile = vooFile;
bdclose(mdl);                       % descarta a retemporizacao (slx intacto)
path(XP_dbl_path0);

%% 11) Resumo
Y = voo.Y; t = voo.t; R2D = 180/pi;
alpha_d = Y(14,:)*R2D; phi_d = Y(5,:)*R2D; theta_d = Y(6,:)*R2D;
k_stall = find(alpha_d > 18.5, 1); k_roll = find(abs(phi_d) > 60, 1);
fprintf('\n===== XP_doublets_lqry2 (%s, i=%d, prot=%d, clamp=%d) =====\n', XP_dbl_tag, i, prot_on, XP_dbl_clamp);
fprintf('sim %.1f s | X-Plane %.1f s | VT %.1f..%.1f (ref %g) | h %.0f..%.0f | alpha %.1f..%.1f | theta %.1f..%.1f | phi max |%.1f|\n', ...
    t(end), voo.t_xplane(end)-voo.t_xplane(1), min(Y(1,:)), max(Y(1,:)), XP_dbl_VT, min(Y(8,:)), max(Y(8,:)), ...
    min(alpha_d), max(alpha_d), min(theta_d), max(theta_d), max(abs(phi_d)));
if isempty(k_stall), fprintf('alpha nunca passou de 18.5 deg (estol do gemeo).\n');
else, fprintf('ESTOL: alpha > 18.5 deg em t = %.1f s.\n', t(k_stall)); end
if isempty(k_roll), fprintf('|phi| nunca passou de 60 deg.\n');
else, fprintf('DEPARTURE lateral: |phi| > 60 deg em t = %.1f s.\n', t(k_roll)); end
fprintf('salvo em: %s\n', vooFile);
