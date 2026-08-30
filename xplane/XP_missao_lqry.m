% XP_missao_lqry.m
% =============================================================
% Missao por waypoints do DH no X-Plane com o controlador LQRY
% (replica do Mirko) — MESMA metodologia do XP_missao.m do PID:
% mesmo harness (xp_read_dh/xp_send_dh 14 canais, teleporte no 1o
% sample, pacing), mesma guiagem LOS (Guidance_Star), mesmas
% ancoras de engate e mesma missao default (G2: quadrado 500x500 m
% na proa de engate, h constante, VT 12).
%
% CONTROLADOR 100%% DO MIRKO (mirko_replica): ganhos Ganho_hold_*.mat
% intocados; modelo modelo_XP_LQRY_GUIA.slx = copia do
% CL_NL_DH_SIL_manobras com APENAS a Planta substituida pelo X-Plane
% (interface 2 in / 11 out preservada) + guiagem somada nos pontos
% das referencias (steps do harness dele zerados).
%
% Diferencas estruturais vs PID (documentadas):
%   - H de controle e' RELATIVO ao engate (o estimador de h do Mirko
%     integrava de 0; aqui alimentamos h_XP - XP_h_ref0 real);
%   - modos: att_alt=0 (hold H), VT_Throttle=1 (hold VT),
%     phi_psi=0 (heading hold psi) — switches "u2>0";
%   - ICs dos integradores PRE-CARREGADOS p/ engate bumpless (os
%     equivalentes das ancoras TrimInput/he do PID): sem isso o
%     throttle partiria de ~0 (Gint_speed lento) e o elevator longe
%     do trim do .acf.
%
% ATENCAO paths: NAO misturar com a arvore do PID (funcoes homonimas
% — aviso do sims_D_lqry.m). Este lancador faz restoredefaultpath e
% adiciona SO: mirko_replica/Dados_mat_SIL (ganhos), xplane/ e
% XPlaneConnect. NAO adicionar Modelo_DH_HIL (planta substituida)
% nem o raiz do repo do PID.
%
% Uso:  XP_missao_lqry               % G2 default
% Config opcional: XP_WPs_frame/XP_WPs_NE/XP_R_accept/XP_msl0/
%   XP_VT0/XP_TimeXP/XP_tag (identicos ao XP_missao do PID)
%
% LEMBRETE DE MOTOR (PENDENCIA_MOTOR.md): File -> Open Aircraft no
% X-Plane antes de CADA corrida (religa o motor; endurance ~150 s).
% =============================================================

%% Config do usuario
if ~exist('XP_msl0','var')      || isempty(XP_msl0),      XP_msl0 = 600;   end
if ~exist('XP_VT0','var')       || isempty(XP_VT0),       XP_VT0  = 12;    end
if ~exist('XP_R_accept','var')  || isempty(XP_R_accept),  XP_R_accept = 80; end
if ~exist('XP_WPs_frame','var'), XP_WPs_frame = []; end
if ~exist('XP_WPs_NE','var'),    XP_WPs_NE    = []; end
if ~exist('XP_TimeXP','var'),    XP_TimeXP    = []; end
if ~exist('XP_tag','var') || isempty(XP_tag), XP_tag = 'LQRY'; end
if isempty(XP_WPs_frame) && isempty(XP_WPs_NE)
    XP_WPs_frame = [ 500    0  XP_msl0  12;
                     500  500  XP_msl0  12;
                       0  500  XP_msl0  12;
                       0    0  XP_msl0  12];
end
cfg_user = struct('msl0',XP_msl0,'VT0',XP_VT0,'R',XP_R_accept, ...
    'WPf',XP_WPs_frame,'WPne',XP_WPs_NE,'T',XP_TimeXP,'tag',XP_tag);

%% 1) Paths LIMPOS (arvore do LQRY independente da do PID)
restoredefaultpath; rehash toolboxcache;
raizM  = 'C:\Users\kaue\Documents\PID_DH\HIL_PID\Matlab\mirko_replica';
xpDir  = fileparts(mfilename('fullpath'));
julioX = fullfile(fileparts(fileparts(xpDir)), 'trabalho_julio', ...
    'PIPER-1-6-roll_back', 'PIPER-1-6-roll_back', 'xplane', 'XPlaneConnect-master', 'MATLAB');
addpath(fullfile(raizM,'Dados_mat_SIL'));
addpath(xpDir); addpath(julioX);
import XPlaneConnect.*

clearvars -except cfg_user raizM xpDir julioX XP_att_alt XP_reftheta
clear global XP_IC
clear xp_read_dh xp_send_dh
global GlobalSocket
try, closeUDP(GlobalSocket); catch, end
GlobalSocket = openUDP('127.0.0.1', 49009, 0, 500);

%% 2) Ganhos e trim do LQRY (INTOCADOS)
load('Ganho_hold_theta.mat');  % GstateLong, GintLong
load('Ganho_hold_H.mat');      % GstateLong_Alt, GintLong_Alt
load('Ganho_hold_VT.mat');     % GstateLong_speed, GintLong_speed
load('Ganho_hold_phi.mat');    % GstateLat, Gintlat
load('Ganho_hold_psi.mat');    % GstateLat_psi, Gintlat_psi
load('Dados_Trim.mat');        % Ue (7x1), Xe (14x1) do Mirko

%% 3) Vars do modelo (modos, refs do harness do Mirko zeradas)
Ts = 1/100;
VT_Throttle = 1;    % throttle <- hold VT
att_alt     = 0;    % elevator <- cascata hold H -> hold theta
phi_psi     = 1;    % aileron  <- BANK HOLD (unico modo lateral estavel do
                    % LQRY: o heading hold phi_psi=0 diverge ate no SIL
                    % original com phi(0)=3 deg — testado 2026-08-30).
                    % A guiagem comanda phi_ref (bank-to-turn no chart).
refPhi = 0; reftheta = 0; refAlt = 0; refPsi = 0;
refVel = 12;        % = Ve do projeto (steps do Mirko: Xe(1) -> refVel em t=5)
% overrides p/ testes de diagnostico (definir ANTES de rodar; opcionais):
if evalin('base',"exist('XP_att_alt','var')"),   att_alt  = evalin('base','XP_att_alt');   end
if evalin('base',"exist('XP_reftheta','var')"),  reftheta = evalin('base','XP_reftheta');  end
K_bank_guia  = 0.1975;        % = V/(g*tau_psi), tau_psi=6 s — igual K_heading do PID
phi_max_guia = deg2rad(20);   % saturacao de phi_ref da guiagem

%% 4) PRE-FLIGHT (mesmo do PID)
r0 = double(getDREFs({'sim/flightmodel/position/elevation', ...
    'sim/flightmodel/position/y_agl','sim/flightmodel/position/theta', ...
    'sim/flightmodel/position/phi','sim/flightmodel/position/true_airspeed', ...
    'sim/time/total_flight_time_sec','sim/flightmodel/position/psi'}, GlobalSocket));
pause(0.6);
t2 = double(getDREFs({'sim/time/total_flight_time_sec'}, GlobalSocket));
if t2 <= r0(6)
    error('XP_missao_lqry: tempo do X-Plane CONGELADO. Reset/despause e rode de novo.');
end
q = getDREFs({'sim/flightmodel/engine/ENGN_TRQ'}, GlobalSocket); trq_pf = double(q(1));
if r0(2) < 3 && abs(trq_pf(1)) < 0.02
    sendCTRL([0,0,0,0.4,-998,-998], 0, GlobalSocket); pause(1.0);
    q = getDREFs({'sim/flightmodel/engine/ENGN_TRQ'}, GlobalSocket); trq_pf = double(q(1));
    sendCTRL([0,0,0,0,-998,-998], 0, GlobalSocket);
    if abs(trq_pf(1)) < 0.02
        warning(['MOTOR MORTO (TRQ=0). Faca File -> Open Aircraft no X-Plane ' ...
                 'e rode de novo (PENDENCIA_MOTOR.md).']);
    end
end
psi_engate = r0(7);
ground_msl = r0(1) - r0(2);
XP_msl0 = cfg_user.msl0;
if XP_msl0 - ground_msl < 120
    XP_msl0 = ground_msl + 150;
    warning('Solo alto: engate movido p/ %.0f m MSL.', XP_msl0);
end
fprintf('XP_missao_lqry: pre-flight OK (proa de engate %.0f deg).\n', psi_engate);

%% 5) Ancoras de engate (MESMAS do PID) + ICs dos integradores
XP_thr0   = 0.55;                       % throttle de engate ~ regime
XP_de0_dg = 2.0;                        % profundor fisico de trim [deg]
XP_pitch0 = 2.0;                        % atitude de teleporte [deg]
XP_h_ref0 = XP_msl0;                    % h de referencia (H_control relativo)
alpha0    = deg2rad(5);                 % AoA tipico do .acf a 12 m/s
theta0    = deg2rad(XP_pitch0);
x4        = [12; alpha0; 0; theta0];    % [VT alpha q theta] no engate

% (ganhos dos .mat do Mirko podem vir como single — Simulink exige double)
% IC do integrador do hold VT: thr%(0) = Gs*x4 + Gint*IC = thr0*100
XP_IC_int_speed = double((XP_thr0*100 - GstateLong_speed*x4) / GintLong_speed);
% IC do integrador do hold theta (5o estado = o proprio comando em deg):
%   de = Gs(1:4)*x4 + Gs(5)*de + Gint*IC  =>  IC = (de*(1-Gs5) - Gs(1:4)*x4)/Gint
XP_IC_int_theta = double((XP_de0_dg*(1 - GstateLong(5)) - GstateLong(1:4)*x4) / GintLong);
% IC do integrador do hold H: theta_ref(0) [rad] = Gs_alt*[x4; dH=0] + Gint*IC = theta0
XP_IC_int_alt   = double((theta0 - GstateLong_Alt*[x4; 0]) / GintLong_Alt);
fprintf('ICs pre-carregados: int_speed %.1f | int_theta %.2f | int_alt %.2f\n', ...
    XP_IC_int_speed, XP_IC_int_theta, XP_IC_int_alt);

%% 6) Missao (mesma montagem do PID: frame da proa -> NE + interp dh>20)
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
R_accept = cfg_user.R;
h_ref0 = XP_h_ref0;      % base dos deltas do Guidance_Star (dh = WP(3)-h_ref0)
VT_ref0 = 12;
per = norm(WPs(1,1:2));
for i = 2:size(WPs,1), per = per + norm(WPs(i,1:2)-WPs(i-1,1:2)); end
TimeXP = cfg_user.T;
if isempty(TimeXP), TimeXP = ceil(per/12*1.5 + 15); end
fprintf('XP_missao_lqry: %d WPs, perimetro %.0f m, TimeXP %.0f s, R_accept %.0f m.\n', ...
    size(WPs,1), per, TimeXP, R_accept);

%% 7) Compila e ARMA o teleporte
mdl = 'modelo_XP_LQRY_GUIA';
load_system(fullfile(xpDir,[mdl '.slx']));
set_param(mdl, 'SimulationCommand', 'update');
global XP_IC
XP_IC = struct('target_msl', XP_msl0, 'h0_agl', XP_msl0-ground_msl, ...
               'VT0', cfg_user.VT0, 'psi0', NaN, 'thr0', XP_thr0, ...
               'de0', deg2rad(XP_de0_dg)/deg2rad(25), 'pitch0', XP_pitch0);
fprintf('XP_missao_lqry: teleporte ARMADO (MSL %.0f m, %.1f m/s). Engatando...\n', XP_msl0, cfg_user.VT0);

%% 8) Voa
out = sim(mdl);

%% 9) Salva (formato compativel com plot_XP_missao)
voosDir = fullfile(xpDir, 'voos');
if ~isfolder(voosDir), mkdir(voosDir); end
voo = struct();
voo.quando     = datestr(now, 'yyyy-mm-dd HH:MM:SS');
voo.Y          = squeeze(out.Y_xp);      % 14 x T (ver xp_read_dh)
voo.U          = out.U_xp;               % T x 4: [thr de da dr]
% U e' logado no passo continuo (0.01): decima p/ o eixo de Y (0.05)
fatU = max(1, round((size(voo.U,1)-1)/(size(voo.Y,2)-1)));
voo.U = voo.U(1:fatU:end, :);
% Y/U sao logados a Ts=0.05 (charts discretos dentro da Planta), tout no
% passo do solver (0.01) — eixo de tempo coerente com Y:
voo.t          = (0:size(voo.Y,2)-1)'*0.05;
voo.t_xplane   = out.t_xplane_log(:);
voo.wp_idx     = out.wp_idx_log(:);
voo.dist_wp    = out.dist_log(:);
voo.WPs        = WPs;
voo.WPs_user   = WPs_user;
voo.R_accept   = R_accept;
voo.psi_engate = psi_engate;
voo.cfg = struct('controlador','LQRY (mirko_replica)', 'XP_msl0',XP_msl0, ...
    'XP_VT0',cfg_user.VT0, 'TimeXP',TimeXP, 'VT_ref',refVel, 'h_ref',XP_h_ref0, ...
    'GstateLong',GstateLong,'GintLong',GintLong, ...
    'GstateLong_Alt',GstateLong_Alt,'GintLong_Alt',GintLong_Alt, ...
    'GstateLong_speed',GstateLong_speed,'GintLong_speed',GintLong_speed, ...
    'GstateLat',GstateLat,'Gintlat',Gintlat, ...
    'GstateLat_psi',GstateLat_psi,'Gintlat_psi',Gintlat_psi, ...
    'ICs',[XP_IC_int_speed XP_IC_int_theta XP_IC_int_alt], ...
    'modos',[VT_Throttle att_alt phi_psi], 'Ue',Ue, 'Xe',Xe);
vooFile = fullfile(voosDir, ['XP_missao_' datestr(now,'yyyymmdd_HHMMSS') '_' cfg_user.tag '.mat']);
save(vooFile, 'voo');
fprintf('Missao LQRY salva em: %s\n', vooFile);

%% 10) Resumo + capturas + figuras
Y = voo.Y; U = voo.U; t = voo.t;
dtx = voo.t_xplane(end) - voo.t_xplane(1);
fprintf('\n===== XP_missao_lqry: resumo =====\n');
fprintf('Sync : sim %.1f s | X-Plane avancou %.1f s | razao %.3f\n', t(end), dtx, dtx/max(t(end),eps));
fprintf('VT   : min %.1f | max %.1f m/s (ref %.1f) | h: min %.1f | max %.1f m\n', ...
    min(Y(1,:)), max(Y(1,:)), refVel, min(Y(8,:)), max(Y(8,:)));
fprintf('phi  : max |%.1f| deg | theta: %.1f a %.1f deg | alpha: %.1f a %.1f deg\n', ...
    rad2deg(max(abs(Y(5,:)))), rad2deg(min(Y(6,:))), rad2deg(max(Y(6,:))), ...
    rad2deg(min(Y(14,:))), rad2deg(max(Y(14,:))));
fprintf('thr  : %.2f a %.2f | de: %+.1f a %+.1f deg\n', ...
    min(U(:,1)), max(U(:,1)), rad2deg(min(U(:,2))), rad2deg(max(U(:,2))));
rotulos = {'-- fora --', 'CAPTURADO'};
for i = 1:size(WPs,1)
    dmin = min(sqrt((Y(11,:) - WPs(i,1)).^2 + (Y(12,:) - WPs(i,2)).^2));
    hit = dmin <= R_accept;
    fprintf('WP%d (N %+7.1f, E %+7.1f): dist min %6.1f m  %s\n', i, WPs(i,1), WPs(i,2), dmin, rotulos{hit+1});
end
fprintf('wp_idx final: %d de %d\n', voo.wp_idx(end), size(WPs,1));
try
    plot_XP_missao(voo, vooFile);
catch ME_plot
    close all force;   % nao deixar janelas presas se o plot falhar
    warning('plot_XP_missao falhou: %s', ME_plot.message);
end
