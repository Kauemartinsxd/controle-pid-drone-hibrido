% XP_missao_lqry2.m
% =============================================================
% Missao por waypoints do DH no X-Plane com o LQRY ATUALIZADO do
% Mirko (lqry_mirko_atualizado, modelo CL_NL_DH_18_jun_2026):
% gain scheduling 3x3 (Ve x He, escolhido pela var i), PsiHold
% re-projetado (VALIDADO no SIL: phi0=3 deg converge em ~12 s — a
% replica antiga explodia), atuadores modelados (surfaces=24 rad/s
% + throttle lag 0.1 s). Ganhos/estrutura do Mirko INTOCADOS.
%
% Mesma metodologia do PID e do LQRY v1 (XP_missao_lqry.m): so a
% Planta virou o X-Plane; guiagem LOS identica; ancoras de engate
% identicas; ICs dos integradores pre-carregados p/ engate bumpless.
%
% Config extra:
%   XP_i_planta  indice do gain scheduling (default 2 = V12_H600).
%                Grade: 1..3 = V12/H{590,600,610}, 4..6 = V15/...,
%                7..9 = V18/... — NOTA: o trim do .acf a 12 m/s
%                (alpha ~5 deg) parece-se com a planta V18 (alpha_e
%                4.5 deg); se i=2 divergir como o v1, testar i=8.
%   XP_phi_psi   0 = PsiHold (default; guiagem manda psi_ref)
%                1 = bank hold (guiagem manda phi_ref bank-to-turn)
%
% LEMBRETE: File -> Open Aircraft no X-Plane antes de cada corrida.
% =============================================================

%% Config do usuario
if ~exist('XP_msl0','var')      || isempty(XP_msl0),      XP_msl0 = 600;   end
if ~exist('XP_VT0','var')       || isempty(XP_VT0),       XP_VT0  = 12;    end
if ~exist('XP_R_accept','var')  || isempty(XP_R_accept),  XP_R_accept = 80; end
if ~exist('XP_WPs_frame','var'), XP_WPs_frame = []; end
if ~exist('XP_WPs_NE','var'),    XP_WPs_NE    = []; end
if ~exist('XP_TimeXP','var'),    XP_TimeXP    = []; end
% GEMEO v1 (2026-08-30): o .acf agora equivale a planta nominal do
% projeto — i=2 (V12_H600) vale para os DOIS eixos (o scheduling por
% eixo i=8/i_lat=2 era o paliativo para o .acf divergente antigo).
if ~exist('XP_i_planta','var')  || isempty(XP_i_planta),  XP_i_planta = 2; end
if ~exist('XP_i_lat','var')     || isempty(XP_i_lat),     XP_i_lat = 2;    end
if ~exist('XP_phi_psi','var')   || isempty(XP_phi_psi),   XP_phi_psi = 0;  end
if ~exist('XP_tag','var') || isempty(XP_tag), XP_tag = 'LQRY2'; end
% velocidade da missao (refVel, WPs e engate). A 15 m/s (i=5) o trim e'
% alpha ~8 -> margem de estol ~10 deg (vs 4 deg a 12 m/s) — envelope
% nativo do projeto do Mirko (script dele usava refVel 15.2).
if ~exist('XP_VT_missao','var') || isempty(XP_VT_missao), XP_VT_missao = 12; end
XP_VT0 = XP_VT_missao;
if isempty(XP_WPs_frame) && isempty(XP_WPs_NE)
    XP_WPs_frame = [ 500    0  XP_msl0  XP_VT_missao;
                     500  500  XP_msl0  XP_VT_missao;
                       0  500  XP_msl0  XP_VT_missao;
                       0    0  XP_msl0  XP_VT_missao];
end
cfg_user = struct('msl0',XP_msl0,'VT0',XP_VT0,'R',XP_R_accept, ...
    'WPf',XP_WPs_frame,'WPne',XP_WPs_NE,'T',XP_TimeXP,'tag',XP_tag, ...
    'i',XP_i_planta,'ilat',XP_i_lat,'phipsi',XP_phi_psi,'VTm',XP_VT_missao);

%% 1) Paths limpos (ganhos novos + xplane + XPC; nada do PID/planta velha)
restoredefaultpath; rehash toolboxcache;
raizN  = 'C:\Users\kaue\Documents\Dissertacao_Mestrado\lqry_mirko_atualizado\Nova pasta';
xpDir  = fileparts(mfilename('fullpath'));
julioX = fullfile(fileparts(fileparts(xpDir)), 'trabalho_julio', ...
    'PIPER-1-6-roll_back', 'PIPER-1-6-roll_back', 'xplane', 'XPlaneConnect-master', 'MATLAB');
addpath(raizN); addpath(xpDir); addpath(julioX);
import XPlaneConnect.*

clearvars -except cfg_user raizN xpDir julioX XP_att_alt XP_reftheta XP_clamp_lqry XP_de0_override XP_thr0_override
clear global XP_IC
clear xp_read_dh xp_send_dh
global GlobalSocket
try, closeUDP(GlobalSocket); catch, end
GlobalSocket = openUDP('127.0.0.1', 49009, 0, 500);

%% 2) Ganhos e trims do LQRY atualizado (INTOCADOS; cells 1x9)
load(fullfile(raizN,'Ganho_hold_theta.mat'));
load(fullfile(raizN,'Ganho_hold_H.mat'));
load(fullfile(raizN,'Ganho_hold_VT.mat'));
load(fullfile(raizN,'Ganho_hold_phi.mat'));
load(fullfile(raizN,'Ganho_hold_psi.mat'));
load(fullfile(raizN,'Dados_Trim.mat'));      % Plantas 1x9 (nome Ve He Xe Ue A B C D)
i = cfg_user.i;          % LONGITUDINAL: agendado por alpha (planta 8 ~ alpha do .acf)
i_lat = cfg_user.ilat;   % LATERAL: agendado por pressao dinamica (planta 2 = 12 m/s)
fprintf('XP_missao_lqry2: scheduling long i=%d (%s) | lat i_lat=%d (%s)\n', ...
    i, Plantas(i).nome, i_lat, Plantas(i_lat).nome);
% O controlador do Mirko novo trabalha em DESVIOS: a planta original soma
% Ue da planta agendada. A Planta_XP reproduz isso (XP_U_trim4).
XP_U_trim4 = [Plantas(i).Ue(1); Plantas(i).Ue(2); Plantas(i_lat).Ue(3); Plantas(i_lat).Ue(4)];
fprintf('trim somado aos comandos: thr %.3f | de %+.2f deg | da/dr %+.2f/%+.2f deg\n', ...
    XP_U_trim4(1), rad2deg(XP_U_trim4(2)), rad2deg(XP_U_trim4(3)), rad2deg(XP_U_trim4(4)));

%% 3) Vars do modelo
Ts = 1/100; surfaces = 24; Variacao_Iner = 0;
VT_Throttle = 1;
att_alt     = 0;                 % hold H (cascata p/ theta)
phi_psi     = cfg_user.phipsi;   % 0 = PsiHold (novo, validado) | 1 = bank hold
refPhi = 0; reftheta = 0; refAlt = 0; refPsi = 0;
refVel = cfg_user.VTm;
K_bank_guia  = 0.1975;           % so usado se phi_psi=1 (bank-to-turn)
phi_max_guia = deg2rad(20);
% PROTECAO DE ENVELOPE (experimento 2026-08-30): clamp no theta_ref do
% LQRY — MESMOS limites efetivos do PID (teto 1 deg abaixo do estol do
% gemeo). Ganhos intocados; testa a hipotese "a diferenca decisiva e' a
% protecao de envelope". Desligar: XP_clamp_lqry = [-pi pi].
if ~evalin('base',"exist('XP_clamp_lqry','var')")
    XP_clamp_lqry = deg2rad([0 17.3]);   % teto 1 deg abaixo do estol; piso 0 p/ descidas
end
% ALPHA-PROTECTION da plataforma (fly-by-wire na Planta_XP): acima de
% alpha_prot o teto nose-up do profundor desce ao trim (rampa de 2 deg).
% Unica protecao efetiva p/ regulador de estados (clamp de ref vaza).
prot_on    = 1;
alpha_prot = deg2rad(16);

%% 4) PRE-FLIGHT
r0 = double(getDREFs({'sim/flightmodel/position/elevation', ...
    'sim/flightmodel/position/y_agl','sim/flightmodel/position/theta', ...
    'sim/flightmodel/position/phi','sim/flightmodel/position/true_airspeed', ...
    'sim/time/total_flight_time_sec','sim/flightmodel/position/psi'}, GlobalSocket));
pause(0.6);
t2 = double(getDREFs({'sim/time/total_flight_time_sec'}, GlobalSocket));
if t2 <= r0(6)
    error('XP_missao_lqry2: tempo do X-Plane CONGELADO. Reset/despause e rode de novo.');
end
q = getDREFs({'sim/flightmodel/engine/ENGN_TRQ'}, GlobalSocket); trq_pf = double(q(1));
if r0(2) < 3 && abs(trq_pf(1)) < 0.02
    sendCTRL([0,0,0,0.4,-998,-998], 0, GlobalSocket); pause(1.0);
    q = getDREFs({'sim/flightmodel/engine/ENGN_TRQ'}, GlobalSocket); trq_pf = double(q(1));
    sendCTRL([0,0,0,0,-998,-998], 0, GlobalSocket);
    if abs(trq_pf(1)) < 0.02
        warning('MOTOR MORTO (TRQ=0). File -> Open Aircraft e rode de novo.');
    end
end
psi_engate = r0(7);
ground_msl = r0(1) - r0(2);
XP_msl0 = cfg_user.msl0;
if XP_msl0 - ground_msl < 120
    XP_msl0 = ground_msl + 150;
    warning('Solo alto: engate movido p/ %.0f m MSL.', XP_msl0);
end
fprintf('XP_missao_lqry2: pre-flight OK (proa %.0f deg, phi_psi=%d).\n', psi_engate, phi_psi);

%% 5) Ancoras de engate + ICs (com os ganhos{i} escolhidos)
% ancoras do GEMEO v1 derivadas do TRIM DA PLANTA AGENDADA (alpha) com
% CORRECAO do trim de comando p/ o trim REAL do gemeo (o casamento de
% de/thr com a planta vale no ponto 12 m/s onde o CG foi calibrado; em
% outras VTs o gradiente de de difere — medido na polar):
alpha0    = atan2(double(Plantas(i).Xe(3)), double(Plantas(i).Xe(1)));
XP_thr0   = double(Plantas(i).Ue(1));
XP_de0_dg = rad2deg(double(Plantas(i).Ue(2)));
if ~exist('XP_de0_override','var') || isempty(XP_de0_override)
else, XP_de0_dg = XP_de0_override; end
if ~exist('XP_thr0_override','var') || isempty(XP_thr0_override)
else, XP_thr0 = XP_thr0_override; end
XP_pitch0 = rad2deg(alpha0);          % nivelado: theta = alpha
XP_h_ref0 = XP_msl0;
theta0    = alpha0;
x4        = [cfg_user.VTm; alpha0; 0; theta0];
% o trim SOMADO na Planta_XP acompanha as ancoras reais:
XP_U_trim4 = [XP_thr0; deg2rad(XP_de0_dg); 0; 0];
de_trim    = deg2rad(XP_de0_dg);     % teto nose-up da alpha_protection [rad]
fprintf('ancoras (planta %d + trim real): alpha/theta %.1f deg | de %+.2f deg | thr %.3f\n', ...
    i, rad2deg(alpha0), XP_de0_dg, XP_thr0);
GsS = double(GstateLong_speed{i}); GiS = double(GintLong_speed{i});
GsT = double(GstateLong{i});       GiT = double(GintLong{i});
GsA = double(GstateLong_Alt{i});   GiA = double(GintLong_Alt{i});
% comandos do controlador sao DESVIOS (a Planta_XP soma XP_U_trim4):
thr_delta0 = XP_thr0 - XP_U_trim4(1);                       % [0..1]
de_delta0  = XP_de0_dg - rad2deg(XP_U_trim4(2));            % [deg]
XP_IC_int_speed = double((thr_delta0*100 - GsS*x4) / GiS);
XP_IC_int_theta = double((de_delta0*(1 - GsT(5)) - GsT(1:4)*x4) / GiT);
XP_IC_int_alt   = double((theta0 - GsA*[x4; 0]) / GiA);
fprintf('ICs: int_speed %.3f | int_theta %.4f | int_alt %.4f\n', ...
    XP_IC_int_speed, XP_IC_int_theta, XP_IC_int_alt);

%% 6) Missao
if ~isempty(cfg_user.WPne)
    WPs_user = cfg_user.WPne;
else
    cpsi = cosd(psi_engate); spsi = sind(psi_engate);
    WPs_user = cfg_user.WPf;
    ne = [cpsi -spsi; spsi cpsi] * cfg_user.WPf(:,1:2)';
    WPs_user(:,1:2) = ne';
end
max_dalt = 20;
WPs = WPs_user(1,:);
for k = 2:size(WPs_user,1)
    dalt = abs(WPs_user(k,3) - WPs_user(k-1,3));
    if dalt > max_dalt
        n_sub = ceil(dalt / max_dalt);
        for kk = 1:n_sub-1
            WPs(end+1,:) = WPs_user(k-1,:) + (kk/n_sub)*(WPs_user(k,:) - WPs_user(k-1,:)); %#ok<SAGROW>
        end
    end
    WPs(end+1,:) = WPs_user(k,:); %#ok<SAGROW>
end
R_accept = cfg_user.R;
h_ref0 = XP_h_ref0; VT_ref0 = cfg_user.VTm;
per = norm(WPs(1,1:2));
for k = 2:size(WPs,1), per = per + norm(WPs(k,1:2)-WPs(k-1,1:2)); end
TimeXP = cfg_user.T;
if isempty(TimeXP), TimeXP = ceil(per/cfg_user.VTm*1.5 + 15); end
fprintf('XP_missao_lqry2: %d WPs, perimetro %.0f m, TimeXP %.0f s.\n', size(WPs,1), per, TimeXP);

%% 7) Compila e ARMA
mdl = 'modelo_XP_LQRY2_GUIA';
load_system(fullfile(xpDir,[mdl '.slx']));
set_param(mdl, 'SimulationCommand', 'update');
global XP_IC
XP_IC = struct('target_msl', XP_msl0, 'h0_agl', XP_msl0-ground_msl, ...
               'VT0', cfg_user.VT0, 'psi0', NaN, 'thr0', XP_thr0, ...
               'de0', deg2rad(XP_de0_dg)/deg2rad(25), 'pitch0', XP_pitch0);
fprintf('XP_missao_lqry2: teleporte ARMADO. Engatando...\n');

%% 8) Voa
out = sim(mdl);

%% 9) Salva
voosDir = fullfile(xpDir, 'voos');
if ~isfolder(voosDir), mkdir(voosDir); end
voo = struct();
voo.quando     = datestr(now, 'yyyy-mm-dd HH:MM:SS');
voo.Y          = squeeze(out.Y_xp);
voo.U          = out.U_xp;
if size(voo.U,1) == 4 && size(voo.U,2) > 4
    voo.U = voo.U.';       % o chart alpha_protection loga 4xT -> T x 4
end
fatU = max(1, round((size(voo.U,1)-1)/(size(voo.Y,2)-1)));
voo.U = voo.U(1:fatU:end, :);
voo.t          = (0:size(voo.Y,2)-1)'*0.05;
voo.t_xplane   = out.t_xplane_log(:);
voo.wp_idx     = out.wp_idx_log(:);
voo.dist_wp    = out.dist_log(:);
voo.WPs        = WPs;
voo.WPs_user   = WPs_user;
voo.R_accept   = R_accept;
voo.psi_engate = psi_engate;
voo.cfg = struct('controlador','LQRY2 (lqry_mirko_atualizado, 18-jun-2026)', ...
    'i_planta',i, 'i_lat',i_lat, 'planta_nome',Plantas(i).nome, ...
    'U_trim4',XP_U_trim4, 'phi_psi',phi_psi, ...
    'XP_msl0',XP_msl0, 'XP_VT0',cfg_user.VT0, 'TimeXP',TimeXP, ...
    'VT_ref',refVel, 'h_ref',XP_h_ref0, ...
    'GstateLong',GsT,'GintLong',GiT,'GstateLong_Alt',GsA,'GintLong_Alt',GiA, ...
    'GstateLong_speed',GsS,'GintLong_speed',GiS, ...
    'GstateLat',double(GstateLat{i}),'Gintlat',double(Gintlat{i}), ...
    'GstateLat_psi',double(GstateLat_psi{i}),'Gintlat_psi',double(Gintlat_psi{i}), ...
    'ICs',[XP_IC_int_speed XP_IC_int_theta XP_IC_int_alt], ...
    'clamp_envelope',XP_clamp_lqry, ...
    'alpha_prot',[prot_on alpha_prot de_trim], ...
    'Ue',Plantas(i).Ue, 'Xe',Plantas(i).Xe);
vooFile = fullfile(voosDir, ['XP_missao_' datestr(now,'yyyymmdd_HHMMSS') '_' cfg_user.tag '.mat']);
save(vooFile, 'voo');
fprintf('Missao LQRY2 salva em: %s\n', vooFile);

%% 10) Resumo + capturas + figuras
Y = voo.Y; U = voo.U; t = voo.t;
dtx = voo.t_xplane(end) - voo.t_xplane(1);
fprintf('\n===== XP_missao_lqry2 (i=%d, phi_psi=%d): resumo =====\n', i, phi_psi);
fprintf('Sync : sim %.1f s | X-Plane %.1f s | razao %.3f\n', t(end), dtx, dtx/max(t(end),eps));
fprintf('VT   : min %.1f | max %.1f (ref %.1f) | h: min %.1f | max %.1f\n', ...
    min(Y(1,:)), max(Y(1,:)), refVel, min(Y(8,:)), max(Y(8,:)));
fprintf('phi  : max |%.1f| deg | theta %.1f..%.1f | alpha %.1f..%.1f deg\n', ...
    rad2deg(max(abs(Y(5,:)))), rad2deg(min(Y(6,:))), rad2deg(max(Y(6,:))), ...
    rad2deg(min(Y(14,:))), rad2deg(max(Y(14,:))));
fprintf('thr  : %.2f a %.2f | de: %+.1f a %+.1f deg\n', ...
    min(U(:,1)), max(U(:,1)), rad2deg(min(U(:,2))), rad2deg(max(U(:,2))));
rotulos = {'-- fora --', 'CAPTURADO'};
for k = 1:size(WPs,1)
    dmin = min(sqrt((Y(11,:) - WPs(k,1)).^2 + (Y(12,:) - WPs(k,2)).^2));
    hit = dmin <= R_accept;
    fprintf('WP%d (N %+7.1f, E %+7.1f): dist min %6.1f m  %s\n', k, WPs(k,1), WPs(k,2), dmin, rotulos{hit+1});
end
fprintf('wp_idx final: %d de %d\n', voo.wp_idx(end), size(WPs,1));
try
    plot_XP_missao(voo, vooFile);
catch ME_plot
    close all force;
    warning('plot_XP_missao falhou: %s', ME_plot.message);
end
