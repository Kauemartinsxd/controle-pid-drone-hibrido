% XP_missao_lqry3.m
% =============================================================
% MISSAO POR WAYPOINTS do DH no X-Plane 9 (gemeo v1.2) com o LQRy v3 —
% re-sintese IMPLEMENTAVEL do LQRy do Mirko (lqry_v3_projeto: mesma
% arquitetura de 5 holds + gain scheduling, ganhos ancorados nos limites reais
% dos atuadores; lqry_v3_prepara_modelo: referencia no termo proporcional,
% theta de trim no Alt Hold, saturacao de phi_ref, anti-windup, fim automatico).
%
% Harness = XP_doublets_lqry2 (laco a 100 Hz, beta corrigido, ICs = 0 em
% desvios, reload com verificacao de assinatura) + missao/guiagem/salvamento
% do XP_missao (mesmo formato de voo: plot_XP_missao e GUI funcionam).
%
% Uso:
%   XP_missao_lqry3                     % oval de 6 WPs a 15 m/s (= GUI)
% Config opcional (defina antes; tudo com default):
%   XP3_WPs_frame [Nx4] [a_frente a_direita hMSL vel] na PROA DE ENGATE
%   XP3_WPs_NE    [Nx4] em NE do engate (precede o frame)
%   XP3_R_accept  [m] (100) | XP3_msl0 [m] (600) | XP3_VT [m/s] (15 -> i=5)
%   XP3_TimeXP    [s] teto (auto)      | XP3_tag ('LQRY3')
%   XP3_Ts_io     [s] laco X-Plane (0.01 = 100 Hz)
%   XP3_phi_psi   0 = psi Hold | 1 = phi Hold + bank-to-turn da guiagem   (0)
%   XP3_phimax_deg (25) | XP3_antiwindup (1) | XP3_clamp_deg ([-10 17]) | XP3_prot (0 = sem alpha-protection)
%   XP3_thr0 / XP3_de0_deg / XP3_pitch0  ancoras de engate do gemeo (default por VT)
%   XP3_ganhos    'v3' | 'orig' (so' p/ contraste)                        ('v3')
%   XP3_autoNL    repete a missao no NL ao fim (1)
%   XP_auto_reload (global opt-in do XP_missao): true = xp_reload_acf antes do voo
% =============================================================

%% 0) Config
if ~exist('XP3_msl0','var')      || isempty(XP3_msl0),      XP3_msl0 = 600;     end
if ~exist('XP3_VT','var')        || isempty(XP3_VT),        XP3_VT = 15;        end
if ~exist('XP3_R_accept','var')  || isempty(XP3_R_accept),  XP3_R_accept = 100; end
if ~exist('XP3_WPs_frame','var'), XP3_WPs_frame = []; end
if ~exist('XP3_WPs_NE','var'),    XP3_WPs_NE = []; end
if ~exist('XP3_TimeXP','var'),    XP3_TimeXP = []; end
if ~exist('XP3_tag','var')       || isempty(XP3_tag),       XP3_tag = 'LQRY3';  end
if ~exist('XP3_Ts_io','var')     || isempty(XP3_Ts_io),     XP3_Ts_io = 0.01;   end
if ~exist('XP3_phi_psi','var')   || isempty(XP3_phi_psi),   XP3_phi_psi = 0;    end
% ESTRUTURA DO MIRKO INTACTA por default (decisao do Kaue 2026-09-03: "so' ganhos"):
% sem saturacao de phi_ref ([]), sem anti-windup (0), sem clamp de theta_ref ([-180 180]),
% referencia so' pelo integrador (ref_prop 0). Tudo opt-in p/ experimentos.
if ~exist('XP3_phimax_deg','var'),                          XP3_phimax_deg = []; end
if ~exist('XP3_antiwindup','var')|| isempty(XP3_antiwindup),XP3_antiwindup = 0; end
if ~exist('XP3_clamp_deg','var') || isempty(XP3_clamp_deg), XP3_clamp_deg = [-180 180]; end
if ~exist('XP3_ref_prop','var')  || isempty(XP3_ref_prop),  XP3_ref_prop = 0; end
if ~exist('XP3_prot','var')      || isempty(XP3_prot),      XP3_prot = 0;       end
if ~exist('XP3_ganhos','var')    || isempty(XP3_ganhos),    XP3_ganhos = 'v3';  end
if ~exist('XP3_autoNL','var')    || isempty(XP3_autoNL),    XP3_autoNL = 1;     end
if isempty(XP3_WPs_frame) && isempty(XP3_WPs_NE)
    XP3_WPs_frame = [256    0  XP3_msl0  15;      % oval "stadium" x1,6 (GUI, 2026-09-01)
                     416  160  XP3_msl0  15;
                     256  320  XP3_msl0  15;
                       0  320  XP3_msl0  15;
                    -160  160  XP3_msl0  15;
                       0    0  XP3_msl0  15];
end

%% 1) Paths (sem restoredefaultpath; restaurado no fim) + socket
XP3_path0 = path;
here   = fileparts(mfilename('fullpath'));
xpDir  = fullfile(fileparts(here), 'xplane');
raizN  = 'C:\Users\kaue\Documents\Dissertacao_Mestrado\lqry_mirko_atualizado\Nova pasta';
julioX = fullfile('C:\Users\kaue\Documents\Dissertacao_Mestrado', 'trabalho_julio', ...
    'PIPER-1-6-roll_back', 'PIPER-1-6-roll_back', 'xplane', 'XPlaneConnect-master', 'MATLAB');
addpath(xpDir); addpath(here); addpath(julioX);
import XPlaneConnect.*
clear global XP_IC
clear xp_read_dh xp_send_dh
global GlobalSocket
try, closeUDP(GlobalSocket); catch, end
GlobalSocket = openUDP('127.0.0.1', 49009, 0, 500);

%% 2) Reload do .acf (opt-in, como no XP_missao)
if exist('XP_auto_reload','var') && ~isempty(XP_auto_reload) && XP_auto_reload
    xp_reload_acf; pause(1.0);
end

%% 3) Ganhos (v3 drop-in) e trims
if strcmp(XP3_ganhos, 'v3'), gd = fullfile(here, 'ganhos'); else, gd = raizN; end
load(fullfile(gd, 'Ganho_hold_theta.mat')); load(fullfile(gd, 'Ganho_hold_H.mat')); load(fullfile(gd, 'Ganho_hold_VT.mat'));
load(fullfile(gd, 'Ganho_hold_phi.mat'));   load(fullfile(gd, 'Ganho_hold_psi.mat'));
load(fullfile(raizN, 'Dados_Trim.mat'));
i = find([Plantas.Ve] == XP3_VT & [Plantas.He] == 600, 1);
assert(~isempty(i), 'XP_missao_lqry3: nao ha planta para Ve=%g @ 600 m', XP3_VT);
i_lat = i;
fprintf('XP_missao_lqry3: ganhos %s | planta i=%d (%s) | %s | laco %.0f Hz\n', XP3_ganhos, i, Plantas(i).nome, ...
    tern(XP3_phi_psi, 'phi Hold + bank-to-turn', 'psi Hold'), 1/XP3_Ts_io);

%% 4) Vars do modelo
Ts = 1/100; surfaces = 24; Variacao_Iner = 0;
VT_Throttle = 1; att_alt = 0; phi_psi = XP3_phi_psi;
refPhi = 0; reftheta = 0; refAlt = 0; refPsi = 0; refVel = XP3_VT;
K_bank_guia = 0.1975; phi_max_guia = deg2rad(20);
XP_clamp_lqry = deg2rad(XP3_clamp_deg);
prot_on = XP3_prot; alpha_prot = deg2rad(16);

%% 5) PRE-FLIGHT
r0 = double(getDREFs({'sim/flightmodel/position/elevation', ...
    'sim/flightmodel/position/y_agl', 'sim/flightmodel/position/theta', ...
    'sim/flightmodel/position/phi', 'sim/flightmodel/position/true_airspeed', ...
    'sim/time/total_flight_time_sec', 'sim/flightmodel/position/psi'}, GlobalSocket));
pause(0.6);
t2 = double(getDREFs({'sim/time/total_flight_time_sec'}, GlobalSocket));
if t2 <= r0(6)
    error('XP_missao_lqry3: tempo do X-Plane CONGELADO. Reset/despause e rode de novo.');
end
q = getDREFs({'sim/flightmodel/engine/ENGN_TRQ'}, GlobalSocket); trq_pf = double(q(1));
if r0(2) < 3 && abs(trq_pf(1)) < 0.02
    sendCTRL([0,0,0,0.4,-998,-998], 0, GlobalSocket); pause(1.0);
    q = getDREFs({'sim/flightmodel/engine/ENGN_TRQ'}, GlobalSocket); trq_pf = double(q(1));
    sendCTRL([0,0,0,0,-998,-998], 0, GlobalSocket);
    if abs(trq_pf(1)) < 0.02
        warning('MOTOR MORTO (TRQ=0). File -> Open Aircraft (ou XP_auto_reload=true) e rode de novo.');
    end
end
psi_engate = r0(7);
ground_msl = r0(1) - r0(2);
XP_msl0 = XP3_msl0;
if XP_msl0 - ground_msl < 120
    XP_msl0 = ground_msl + 150;
    warning('Solo alto: engate movido p/ %.0f m MSL.', XP_msl0);
end
fprintf('XP_missao_lqry3: pre-flight OK (proa %.0f deg, solo %.0f m MSL, TRQ %.2f).\n', psi_engate, ground_msl, trq_pf(1));

%% 6) Ancoras de engate do gemeo v1.2 (trim medido) — o controlador v3 trabalha em desvios
switch XP3_VT
    case 12, anc = [0.43, 7.0, 14.0];      % thr, de [deg], pitch [deg] (EQUIVALENCIA_ACF: trim a 12 m/s)
    case 15, anc = [0.40, 5.0,  9.0];      % ADENDO 12: thr 0,34-0,45, de +1..+8, theta ~9
    otherwise, anc = [double(Plantas(i).Ue(1)), rad2deg(double(Plantas(i).Ue(2))), rad2deg(double(Plantas(i).Xe(8)))];
end
if exist('XP3_thr0','var')    && ~isempty(XP3_thr0),    anc(1) = XP3_thr0;    end
if exist('XP3_de0_deg','var') && ~isempty(XP3_de0_deg), anc(2) = XP3_de0_deg; end
if exist('XP3_pitch0','var')  && ~isempty(XP3_pitch0),  anc(3) = XP3_pitch0;  end
XP_thr0 = anc(1); XP_de0_dg = anc(2); XP_pitch0 = anc(3);
XP_h_ref0 = XP_msl0;
XP_U_trim4 = [XP_thr0; deg2rad(XP_de0_dg); 0; 0];
de_trim    = deg2rad(XP_de0_dg);
theta_e_ff = deg2rad(XP_pitch0);            % feed-forward do theta de trim no Alt Hold
XP_IC_int_speed = 0; XP_IC_int_theta = 0; XP_IC_int_alt = 0;   % desvios nulos no engate
fprintf('ancoras do gemeo: thr %.2f | de %+.1f deg | pitch %.1f deg (theta_e_ff)\n', XP_thr0, XP_de0_dg, XP_pitch0);

%% 7) Missao: WPs em NE do engate + interpolacao de Dh > 20 m
if ~isempty(XP3_WPs_NE)
    WPs_user = XP3_WPs_NE;
else
    cpsi = cosd(psi_engate); spsi = sind(psi_engate);
    WPs_user = XP3_WPs_frame;
    ne = [cpsi -spsi; spsi cpsi] * XP3_WPs_frame(:,1:2)';
    WPs_user(:,1:2) = ne';
end
WPs = WPs_user(1,:);
for k = 2:size(WPs_user,1)
    dalt = abs(WPs_user(k,3) - WPs_user(k-1,3));
    if dalt > 20
        n_sub = ceil(dalt/20);
        for kk = 1:n_sub-1
            WPs(end+1,:) = WPs_user(k-1,:) + (kk/n_sub)*(WPs_user(k,:) - WPs_user(k-1,:)); %#ok<SAGROW>
        end
    end
    WPs(end+1,:) = WPs_user(k,:); %#ok<SAGROW>
end
R_accept = XP3_R_accept; h_ref0 = XP_h_ref0; VT_ref0 = XP3_VT;
N_WPs = size(WPs,1); WPfim_N = WPs(end,1); WPfim_E = WPs(end,2);
seg = [0 0; WPs(:,1:2)]; per = sum(sqrt(sum(diff(seg).^2, 2)));
TimeXP = XP3_TimeXP; if isempty(TimeXP), TimeXP = ceil(per/XP3_VT*1.5 + 20); end
fprintf('XP_missao_lqry3: %d WPs (%d apos interpolacao), perimetro %.0f m, teto %.0f s, R_accept %.0f m.\n', ...
    size(WPs_user,1), N_WPs, per, TimeXP, R_accept);

%% 8) Modelo + edicoes em memoria + ARMA o teleporte
mdl = 'modelo_XP_LQRY2_GUIA';
if bdIsLoaded(mdl), bdclose(mdl); end
load_system(fullfile(xpDir, [mdl '.slx']));
XP3_info = lqry_v3_prepara_modelo(mdl, struct('phi_max_deg', XP3_phimax_deg, 'antiwindup', XP3_antiwindup, ...
    'fim_auto', 1, 'Ts_io', XP3_Ts_io, 'ref_prop', XP3_ref_prop, 'theta_e_ff', XP3_ref_prop, ...
    'ic_bumpless', ~XP3_ref_prop, 'theta0', deg2rad(XP_pitch0), 'vt_pulse_off', 1, 'guia_psi_off', 0, 'i', i));
set_param(mdl, 'SimulationCommand', 'update');
global XP_IC XP_TRIM_DELTA XP_TRIM_FOUND
XP_TRIM_DELTA = []; XP_TRIM_FOUND = [];
XP_IC = struct('target_msl', XP_msl0, 'h0_agl', XP_msl0 - ground_msl, ...
               'VT0', XP3_VT, 'psi0', NaN, 'thr0', XP_thr0, ...
               'de0', deg2rad(XP_de0_dg)/deg2rad(25), 'pitch0', XP_pitch0, ...
               'warmup', 0, 'warm_simple', 1);
fprintf('XP_missao_lqry3: teleporte ARMADO (MSL %.0f, %.0f m/s). Engatando...\n', XP_msl0, XP3_VT);

%% 9) Voa
t0 = tic;
out = sim(mdl, 'StopTime', num2str(TimeXP));
t_cpu = toc(t0);

%% 10) Salva (formato XP_missao)
voosDir = fullfile(xpDir, 'voos'); if ~isfolder(voosDir), mkdir(voosDir); end
voo = struct();
voo.quando   = datestr(now, 'yyyy-mm-dd HH:MM:SS');
voo.Y        = squeeze(out.Y_xp); if size(voo.Y,1) ~= 14, voo.Y = voo.Y.'; end
voo.U        = squeeze(out.U_xp); if size(voo.U,1) == 4 && size(voo.U,2) > 4, voo.U = voo.U.'; end
voo.t        = (0:size(voo.Y,2)-1)'*XP3_Ts_io;
voo.U        = reamostra_log(voo.U, voo.t);        % o chart alpha_protection loga a 20 Hz
voo.Ts_io    = XP3_Ts_io;
voo.t_xplane = out.t_xplane_log(:);
% o chart de guiagem loga a 20 Hz (sample time proprio) e Y_xp a 100 Hz: reamostra p/ a base de voo.t
voo.wp_idx   = reamostra_log(out.wp_idx_log(:), voo.t);
voo.dist_wp  = reamostra_log(out.dist_log(:), voo.t);
voo.WPs = WPs; voo.WPs_user = WPs_user; voo.R_accept = R_accept; voo.psi_engate = psi_engate;
voo.NL = struct();
for nm = {'theta_NL','q_NL','elev_NL','VT_NL','Throttle_NL','H_NL','phi_NL','p_NL','ail_NL','psi_NL','r_NL','rud_NL','beta_NL'}
    try, v = out.get(nm{1}); voo.NL.(nm{1}) = struct('time', v.time, 'values', v.signals.values); catch, end
end
try, voo.probe = struct('phiref', out.probe_phiref, 'thetaref', out.probe_thetaref, 'refH', out.probe_refH, 'refVT', out.probe_refVT); catch, end
voo.cfg = struct('controlador', sprintf('LQRy %s (mesma estrutura do Mirko; lqry_v3) no gemeo X-Plane', XP3_ganhos), ...
    'i_planta', i, 'planta_nome', Plantas(i).nome, 'VT', XP3_VT, 'Ts_io', XP3_Ts_io, 'phi_psi', phi_psi, ...
    'phimax_deg', XP3_phimax_deg, 'antiwindup', XP3_antiwindup, 'clamp_deg', XP3_clamp_deg, 'ref_prop', XP3_ref_prop, 'prot_on', prot_on, ...
    'prepara', XP3_info, 'ancoras', [XP_thr0 XP_de0_dg XP_pitch0], 'U_trim4', XP_U_trim4, 'XP_msl0', XP_msl0, 'TimeXP', TimeXP, ...
    'GstateLong', double(GstateLong{i}), 'GintLong', double(GintLong{i}), 'GstateLong_Alt', double(GstateLong_Alt{i}), 'GintLong_Alt', double(GintLong_Alt{i}), ...
    'GstateLong_speed', double(GstateLong_speed{i}), 'GintLong_speed', double(GintLong_speed{i}), ...
    'GstateLat', double(GstateLat{i}), 'Gintlat', double(Gintlat{i}), 'GstateLat_psi', double(GstateLat_psi{i}), 'Gintlat_psi', double(Gintlat_psi{i}), ...
    'Ue', Plantas(i).Ue, 'Xe', Plantas(i).Xe, 't_cpu', t_cpu);
vooFile = fullfile(voosDir, ['XP_missao_' datestr(now,'yyyymmdd_HHMMSS') '_' XP3_tag '.mat']);
save(vooFile, 'voo');
XP3_lastfile = vooFile;
bdclose(mdl);                        % descarta as edicoes em memoria (slx intacto)

%% 11) Resumo + capturas + figuras
Y = voo.Y; U = voo.U; t = voo.t; R2D = 180/pi;
dtx = voo.t_xplane(end) - voo.t_xplane(1);
alpha_d = Y(14,:)*R2D; phi_d = Y(5,:)*R2D;
k_stall = find(alpha_d > 18.5, 1); k_roll = find(abs(phi_d) > 60, 1);
fprintf('\n===== XP_missao_lqry3 (%s, ganhos %s, i=%d): resumo =====\n', XP3_tag, XP3_ganhos, i);
fprintf('Sync : sim %.1f s | X-Plane %.1f s | razao %.3f\n', t(end), dtx, dtx/max(t(end),eps));
fprintf('VT   : %.1f..%.1f (ref %g) | h %.1f..%.1f | phi max |%.1f| | theta %.1f..%.1f | alpha %.1f..%.1f | de %+.1f..%+.1f | thr %.2f..%.2f\n', ...
    min(Y(1,:)), max(Y(1,:)), XP3_VT, min(Y(8,:)), max(Y(8,:)), max(abs(phi_d)), min(Y(6,:))*R2D, max(Y(6,:))*R2D, ...
    min(alpha_d), max(alpha_d), min(U(:,2))*R2D, max(U(:,2))*R2D, min(U(:,1)), max(U(:,1)));
if ~isempty(k_stall), fprintf('ESTOL: alpha > 18.5 deg em t = %.1f s.\n', t(k_stall)); end
if ~isempty(k_roll),  fprintf('DEPARTURE lateral: |phi| > 60 deg em t = %.1f s.\n', t(k_roll)); end
nWP = size(WPs,1); n_hit = 0;
for k = 1:nWP
    dmin = min(sqrt((Y(11,:) - WPs(k,1)).^2 + (Y(12,:) - WPs(k,2)).^2));
    hit = dmin <= R_accept; n_hit = n_hit + hit;
    fprintf('WP%d (N %+7.1f, E %+7.1f): dist min %6.1f m  %s\n', k, WPs(k,1), WPs(k,2), dmin, tern(hit, 'CAPTURADO', '-- fora --'));
end
fprintf('CAPTURAS: %d/%d | wp_idx final %d de %d | salvo em %s\n', n_hit, nWP, voo.wp_idx(end), nWP, vooFile);
XP3_resultado = struct('capturas', n_hit, 'nWP', nWP, 'arquivo', vooFile);
try, plot_XP_missao(voo, vooFile); catch e, close all force; warning('plot_XP_missao falhou: %s', e.message); end
path(XP3_path0);

%% 12) MESMA missao no NL (comparacao automatica), WPs de volta ao frame de engate
if XP3_autoNL
    try
        cpsi2 = cosd(voo.psi_engate); spsi2 = sind(voo.psi_engate);
        wpF = voo.WPs_user;
        wpF(:,1:2) = [voo.WPs_user(:,1)*cpsi2 + voo.WPs_user(:,2)*spsi2, voo.WPs_user(:,2)*cpsi2 - voo.WPs_user(:,1)*spsi2];
        NL3_WPs = wpF; NL3_R_accept = voo.R_accept; NL3_TimeXP = voo.cfg.TimeXP; NL3_VT = XP3_VT;
        NL3_phi_psi = XP3_phi_psi; NL3_ganhos = XP3_ganhos; NL3_tag = 'autoNL_LQRY3'; NL3_plot = false;
        vooXP = voo; vooFileXP = vooFile;
        fprintf('\n===== Repetindo a missao no modelo NL (LQRy %s, sem X-Plane)... =====\n', XP3_ganhos);
        run(fullfile(here, 'NL_missao_lqry3.m'));
        vooNL = voo; voo = vooXP; vooFile = vooFileXP;
        addpath(xpDir);
        plot_missao_comparada(voo, vooNL, vooFile);
    catch e_cmp
        fprintf('comparacao NL automatica falhou (%s) — voo X-Plane salvo normalmente.\n', e_cmp.message);
    end
end

function s = tern(c, a, b)
if c, s = a; else, s = b; end
end

function y = reamostra_log(x, t)
% log de taxa diferente (uniforme, 0..t(end)) -> amostras de t (segura o ultimo valor); x = vetor ou matriz (linhas = tempo)
n = size(x, 1);
if n == numel(t), y = x; return; end
tx = linspace(0, t(end), n)';
y = interp1(tx, x, t, 'previous', 'extrap');
end
