% NL_missao_lqry3.m
% =============================================================
% MISSAO POR WAYPOINTS com o LQRy v3 (re-sintese implementavel do LQRy do
% Mirko) na planta NL da Ana, SEM X-Plane: modelo_NL_LQRY_GUIA.slx =
% modelo_XP_LQRY2_GUIA (controlador + guiagem LOS) com a Planta_XP trocada
% pela sfunction_DH + cadeia de atuadores (gerado por lqry_v3_build_nl_model).
% Mesmo formato de saida do NL_missao/XP_missao (plot_XP_missao funciona).
%
% Uso:
%   NL_missao_lqry3                       % oval de 6 WPs a 15 m/s (= GUI), ganhos v3
% Config opcional (defina antes; tudo com default):
%   NL3_WPs        [Nx4] [a_frente a_direita hMSL vel] (frame do engate == NE)
%   NL3_R_accept   raio de aceitacao [m]              (100)
%   NL3_TimeXP     teto de duracao [s]                (auto)
%   NL3_VT         velocidade de engate/planta i [m/s] (15 -> i=5; 12 -> i=2; 18 -> i=8)
%   NL3_ganhos     'v3' | 'orig'                      ('v3')
%   NL3_coef       'ana' | 'sato'                     ('ana')
%   NL3_eng_tau    lag do motor [s]                   (0.3; X-Plane ~3.5)
%   NL3_sat_deg    curso das superficies [deg]        (15)
%   NL3_Cm_thr     efeito de potencia na arfagem      (0; X-Plane ~0.25)
%   NL3_phi_psi    0 = psi Hold (Caso 4) | 1 = phi Hold + bank-to-turn da guiagem (0)
%   NL3_phimax_deg saturacao do phi_ref               (25)
%   NL3_antiwindup 1/0                                (1)
%   NL3_clamp_deg  [min max] de theta_ref             ([-10 17])
%   NL3_tag        sufixo do .mat                     ('LQRY3')
% =============================================================

%% 0) Config
if ~exist('NL3_R_accept','var')  || isempty(NL3_R_accept),  NL3_R_accept = 100; end
if ~exist('NL3_WPs','var'),      NL3_WPs = []; end
if ~exist('NL3_TimeXP','var'),   NL3_TimeXP = []; end
if ~exist('NL3_VT','var')        || isempty(NL3_VT),        NL3_VT = 15; end
if ~exist('NL3_ganhos','var')    || isempty(NL3_ganhos),    NL3_ganhos = 'v3'; end
if ~exist('NL3_coef','var')      || isempty(NL3_coef),      NL3_coef = 'ana'; end
if ~exist('NL3_eng_tau','var')   || isempty(NL3_eng_tau),   NL3_eng_tau = 0.3; end
if ~exist('NL3_sat_deg','var')   || isempty(NL3_sat_deg),   NL3_sat_deg = 15; end
if ~exist('NL3_Cm_thr','var')    || isempty(NL3_Cm_thr),    NL3_Cm_thr = 0; end
if ~exist('NL3_phi_psi','var')   || isempty(NL3_phi_psi),   NL3_phi_psi = 0; end
% ESTRUTURA DO MIRKO INTACTA por default (decisao do Kaue 2026-09-03: "so' ganhos"):
% sem saturacao de phi_ref ([]), sem anti-windup (0), sem clamp de theta_ref ([-180 180]),
% referencia so' pelo integrador (ref_prop 0). Tudo opt-in p/ experimentos.
if ~exist('NL3_phimax_deg','var'),                          NL3_phimax_deg = []; end
if ~exist('NL3_antiwindup','var')|| isempty(NL3_antiwindup),NL3_antiwindup = 0; end
if ~exist('NL3_clamp_deg','var') || isempty(NL3_clamp_deg), NL3_clamp_deg = [-180 180]; end
if ~exist('NL3_ref_prop','var')  || isempty(NL3_ref_prop),  NL3_ref_prop = 0; end
if ~exist('NL3_tag','var')       || isempty(NL3_tag),       NL3_tag = 'LQRY3'; end
if ~exist('NL3_plot','var')      || isempty(NL3_plot),      NL3_plot = true; end
if isempty(NL3_WPs)
    % oval de 6 WPs a 15 m/s (mesmo da GUI, botao "Circuito OVAL")
    NL3_WPs = [256    0  600  15;
               416  160  600  15;
               256  320  600  15;
                 0  320  600  15;
              -160  160  600  15;
                 0    0  600  15];
end

%% 1) Paths (isolados: funcoes homonimas entre o repo PID e o mirko_run)
here   = fileparts(mfilename('fullpath'));
xpDir  = fullfile(fileparts(here), 'xplane');
mirko  = 'C:\Users\kaue\Documents\Dissertacao_Mestrado\lqry_mirko_atualizado\reproducao_SIL\mirko_run';
raizN  = 'C:\Users\kaue\Documents\Dissertacao_Mestrado\lqry_mirko_atualizado\Nova pasta';
NL3_path0 = path;
pp = strsplit(path, pathsep);
ruins = pp(contains(pp, 'controle-pid-drone-hibrido') & ~contains(pp, 'lqry_v3') & ~contains(pp, [filesep 'xplane']));
for k = 1:numel(ruins), rmpath(ruins{k}); end
addpath(xpDir); addpath(here); addpath(mirko);      % mirko_run na frente: sfunction_DH(t,x,u,flag,Xe,coef_Sato,coef_Ana,Variacao_Iner)
assert(startsWith(which('sfunction_DH'), mirko), 'NL_missao_lqry3: sfunction_DH errada no path: %s', which('sfunction_DH'));

%% 2) Ganhos e trims
if strcmp(NL3_ganhos, 'v3'), gd = fullfile(here, 'ganhos'); else, gd = raizN; end
load(fullfile(gd, 'Ganho_hold_theta.mat')); load(fullfile(gd, 'Ganho_hold_H.mat')); load(fullfile(gd, 'Ganho_hold_VT.mat'));
load(fullfile(gd, 'Ganho_hold_phi.mat'));   load(fullfile(gd, 'Ganho_hold_psi.mat'));
load(fullfile(raizN, 'Dados_Trim.mat'));
i = find([Plantas.Ve] == NL3_VT & [Plantas.He] == 600, 1);
assert(~isempty(i), 'NL_missao_lqry3: nao ha planta para Ve=%g @ 600 m', NL3_VT);
i_lat = i;
fprintf('NL_missao_lqry3: ganhos %s | planta i=%d (%s) | coef %s | motor tau %g s | curso +-%g deg | Cm_thr %g | %s\n', ...
    NL3_ganhos, i, Plantas(i).nome, NL3_coef, NL3_eng_tau, NL3_sat_deg, NL3_Cm_thr, tern(NL3_phi_psi, 'phi Hold + bank-to-turn', 'psi Hold'));

%% 3) Vars do modelo (controlador)
Ts = 1/100; surfaces = 24; Variacao_Iner = 0;
VT_Throttle = 1; att_alt = 0; phi_psi = NL3_phi_psi;
refPhi = 0; reftheta = 0; refAlt = 0; refPsi = 0; refVel = NL3_VT;
K_bank_guia = 0.1975; phi_max_guia = deg2rad(20);
XP_clamp_lqry = deg2rad(NL3_clamp_deg);
prot_on = 0; alpha_prot = deg2rad(16);
theta_e_ff = double(Plantas(i).Xe(8));        % feed-forward do theta de trim no Alt Hold (lqry_v3_prepara_modelo)
% planta
Xe_planta = Plantas(i).Xe; Xe_planta(12) = -600;
XP_U_trim4 = double(Plantas(i).Ue(1:4));
de_trim = XP_U_trim4(2);
XP_h_ref0 = 600;
coef_Ana = double(strcmp(NL3_coef, 'ana')); coef_Sato = double(strcmp(NL3_coef, 'sato'));
act.rate = deg2rad(150); act.tau = 0.05; act.bw = 1/act.tau;
eng.rate = 1.0; eng.tau = NL3_eng_tau;
sat_surf_rad = deg2rad(NL3_sat_deg);
global HYB
HYB = [];
if NL3_Cm_thr ~= 0
    HYB = struct('k_Iroll',1,'k_Iyaw',1,'k_Clda',1,'k_Cnb',1,'k_Clb',1,'k_Clp',1,'k_Cnr',1,'Cm_thr',NL3_Cm_thr);
end

%% 4) Missao (frame do engate == NE; engate na origem, proa 0)
WPs_user = NL3_WPs;
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
R_accept = NL3_R_accept; h_ref0 = 600; VT_ref0 = NL3_VT;
N_WPs = size(WPs,1); WPfim_N = WPs(end,1); WPfim_E = WPs(end,2);
seg = [0 0; WPs(:,1:2)]; per = sum(sqrt(sum(diff(seg).^2, 2)));
TimeXP = NL3_TimeXP; if isempty(TimeXP), TimeXP = ceil(per/NL3_VT*1.5 + 20); end
fprintf('NL_missao_lqry3: %d WPs (%d apos interpolacao), perimetro %.0f m, teto %.0f s, R_accept %.0f m.\n', ...
    size(WPs_user,1), N_WPs, per, TimeXP, R_accept);

%% 5) Modelo NL + edicoes em memoria (phi_ref sat, anti-windup, fim automatico)
mdl = 'modelo_NL_LQRY_GUIA';
if bdIsLoaded(mdl), bdclose(mdl); end
load_system(fullfile(here, [mdl '.slx']));
NL3_info = lqry_v3_prepara_modelo(mdl, struct('phi_max_deg', NL3_phimax_deg, 'antiwindup', NL3_antiwindup, ...
    'ref_prop', NL3_ref_prop, 'theta_e_ff', NL3_ref_prop, 'ic_bumpless', ~NL3_ref_prop, 'theta0', double(Plantas(i).Xe(8)), ...
    'fim_auto', 1, 'vt_pulse_off', 1, 'i', i));

%% 6) Simula
t0 = tic;
out = sim(mdl, 'StopTime', num2str(TimeXP));
t_cpu = toc(t0);

%% 7) Salva no formato XP_missao
voosDir = fullfile(xpDir, 'voos'); if ~isfolder(voosDir), mkdir(voosDir); end
voo = struct();
voo.quando   = datestr(now, 'yyyy-mm-dd HH:MM:SS');
voo.Y        = squeeze(out.Y_xp); if size(voo.Y,1) ~= 14, voo.Y = voo.Y.'; end
voo.U        = squeeze(out.U_xp); if size(voo.U,1) == 4 && size(voo.U,2) > 4, voo.U = voo.U.'; end
voo.t        = out.tout(:);
voo.U        = reamostra_log(voo.U, voo.t);        % o chart alpha_protection loga a 20 Hz
voo.t_xplane = voo.Y(10,:).';
% o chart de guiagem loga a 20 Hz (sample time proprio) e Y_xp a 100 Hz: reamostra p/ a base de voo.t
voo.wp_idx   = reamostra_log(out.wp_idx_log(:), voo.t);
voo.dist_wp  = reamostra_log(out.dist_log(:), voo.t);
voo.WPs      = WPs; voo.WPs_user = WPs_user; voo.R_accept = R_accept; voo.psi_engate = 0;
voo.NL = struct();
for nm = {'theta_NL','q_NL','elev_NL','VT_NL','Throttle_NL','H_NL','phi_NL','p_NL','ail_NL','psi_NL','r_NL','rud_NL','beta_NL'}
    try, v = out.get(nm{1}); voo.NL.(nm{1}) = struct('time', v.time, 'values', v.signals.values); catch, end
end
try, voo.probe = struct('phiref', out.probe_phiref, 'thetaref', out.probe_thetaref, 'refH', out.probe_refH, 'refVT', out.probe_refVT); catch, end
voo.cfg = struct('controlador', sprintf('LQRy %s (mesma estrutura do Mirko) na planta NL', NL3_ganhos), ...
    'planta', sprintf('NL %s (sfunction_DH mirko_run), motor tau %g, curso %g deg, Cm_thr %g', NL3_coef, NL3_eng_tau, NL3_sat_deg, NL3_Cm_thr), ...
    'i_planta', i, 'planta_nome', Plantas(i).nome, 'VT', NL3_VT, 'phi_psi', phi_psi, 'TimeXP', TimeXP, ...
    'phimax_deg', NL3_phimax_deg, 'antiwindup', NL3_antiwindup, 'clamp_deg', NL3_clamp_deg, 'ref_prop', NL3_ref_prop, 'prepara', NL3_info, ...
    'GstateLong', double(GstateLong{i}), 'GintLong', double(GintLong{i}), 'GstateLong_Alt', double(GstateLong_Alt{i}), 'GintLong_Alt', double(GintLong_Alt{i}), ...
    'GstateLong_speed', double(GstateLong_speed{i}), 'GintLong_speed', double(GintLong_speed{i}), ...
    'GstateLat', double(GstateLat{i}), 'Gintlat', double(Gintlat{i}), 'GstateLat_psi', double(GstateLat_psi{i}), 'Gintlat_psi', double(Gintlat_psi{i}), ...
    'Ue', Plantas(i).Ue, 'Xe', Plantas(i).Xe, 't_cpu', t_cpu);
vooFile = fullfile(voosDir, ['NL_missao_' datestr(now,'yyyymmdd_HHMMSS') '_' NL3_tag '.mat']);
save(vooFile, 'voo');
bdclose(mdl); HYB = [];
path(NL3_path0);

%% 8) Resumo
Y = voo.Y; U = voo.U; t = voo.t; R2D = 180/pi;
fprintf('\n===== NL_missao_lqry3 (%s, ganhos %s, i=%d): resumo (%.0f s de CPU) =====\n', NL3_tag, NL3_ganhos, i, t_cpu);
fprintf('sim %.1f s de %.0f | VT %.1f..%.1f (ref %g) | h %.1f..%.1f | phi max |%.1f| | theta %.1f..%.1f | alpha %.1f..%.1f | de %+.1f..%+.1f | thr %.2f..%.2f\n', ...
    t(end), TimeXP, min(Y(1,:)), max(Y(1,:)), NL3_VT, min(Y(8,:)), max(Y(8,:)), max(abs(Y(5,:)))*R2D, ...
    min(Y(6,:))*R2D, max(Y(6,:))*R2D, min(Y(14,:))*R2D, max(Y(14,:))*R2D, min(U(:,2))*R2D, max(U(:,2))*R2D, min(U(:,1)), max(U(:,1)));
nWP = size(WPs,1); n_hit = 0;
for k = 1:nWP
    dmin = min(sqrt((Y(11,:) - WPs(k,1)).^2 + (Y(12,:) - WPs(k,2)).^2));
    hit = dmin <= R_accept; n_hit = n_hit + hit;
    fprintf('WP%d (N %+7.1f, E %+7.1f): dist min %6.1f m  %s\n', k, WPs(k,1), WPs(k,2), dmin, tern(hit, 'CAPTURADO', '-- fora --'));
end
fprintf('CAPTURAS: %d/%d | wp_idx final %d | salvo em %s\n', n_hit, nWP, voo.wp_idx(end), vooFile);
NL3_resultado = struct('capturas', n_hit, 'nWP', nWP, 'arquivo', vooFile, 'voo', voo);
if NL3_plot
    try, plot_XP_missao(voo, vooFile); catch e, fprintf('plot falhou: %s\n', e.message); end
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
