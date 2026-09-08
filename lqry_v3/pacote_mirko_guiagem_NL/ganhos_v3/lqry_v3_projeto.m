function R = lqry_v3_projeto(varargin)
% LQRY_V3_PROJETO  Re-sintese IMPLEMENTAVEL do LQRy do Mirko ("v3").
%
%   MESMA arquitetura do CL_NL_DH_18_jun_2026 / modelo_XP_LQRY2_GUIA:
%     theta Hold : de[deg]      = GstateLong{i}*[dVT dalpha q dtheta de_deg] + GintLong{i}*int(theta - theta_ref)
%     Alt   Hold : theta_ref    = GstateLong_Alt{i}*[dVT dalpha q dtheta dH]  + GintLong_Alt{i}*int(H - H_ref)
%     Vel   Hold : thr[%]       = GstateLong_speed{i}*[dVT dalpha q dtheta]   + GintLong_speed{i}*int(VT - VT_ref)
%     phi   Hold : [da;dr][rad] = GstateLat{i}*[beta p r phi]                  + Gintlat{i}*[int(phi - phi_ref); int(beta)]
%     psi   Hold : phi_ref      = GstateLat_psi{i}*[beta p r phi psi]          + Gintlat_psi{i}*int(psi - psi_ref)
%   (unidades e sinais IDENTICOS aos dos Ganho_hold_*.mat do Mirko — os
%   .mat gerados aqui sao "drop-in": mesmos nomes de variaveis, cells 1x9.)
%
%   MESMO metodo (LQ com acao integral / realimentacao de saida, Stevens &
%   Lewis): cada hold e' sintetizado por LQR na planta aumentada com o
%   integrador do erro; onde a estrutura do Mirko NAO realimenta algum estado
%   (estado do atuador, integrador da malha interna, estado do motor), o ganho
%   e' REFINADO como realimentacao de saida estatica (minimiza tr(P X) com P da
%   equacao de Lyapunov — exatamente a formulacao LQRy do artigo, eq. 11-16).
%
%   O QUE MUDA em relacao ao projeto original: os pesos (regra de Bryson)
%   sao ancorados nos limites REAIS da plataforma — curso das superficies
%   +-15 deg, manete [0,1], motor eletrico com tau ate 3,5 s (medido no
%   X-Plane), laco discreto de 20-100 Hz. Os ganhos originais saturam o
%   profundor com ~1,8 deg de erro de theta (8,4 deg/deg + 28 deg de theta_ref
%   por metro de altitude) e a manete com 1,3 m/s de erro de VT: sao
%   "otimos" para um atuador ideal e nao para o aviao.
%
%   Uso:
%     R = lqry_v3_projeto;                       % 9 plantas, defaults, salva ganhos/ em lqry_v3/
%     R = lqry_v3_projeto('iset', 5, 'salvar', false, 'verbose', true);
%     R = lqry_v3_projeto('lim', struct('theta', 6*pi/180));   % override parcial dos limites
%
%   Saidas (R): ganhos (cells), matrizes de projeto, polos/margens/tabela de
%   "ganho por grau" de cada planta; arquivos ganhos/Ganho_hold_*.mat e
%   ganhos/LQRY_v3.mat + relatorio_projeto.txt.
%
%   Kaue / Claude, 2026-09-03.

%% ---------------- parametros ----------------
p = inputParser;
p.addParameter('raizN', 'C:\Users\kaue\Documents\Dissertacao_Mestrado\lqry_mirko_atualizado\Nova pasta');
p.addParameter('outdir', fullfile(fileparts(mfilename('fullpath')), 'ganhos'));
p.addParameter('iset', 1:9);
p.addParameter('salvar', true);
p.addParameter('verbose', true);
p.addParameter('lim', struct());
p.addParameter('Ti', struct());
p.addParameter('tau_motor_proj', 2.0);      % lag de motor NO PROJETO do Vel Hold [s] (nao medido -> SOF)
p.addParameter('tau_motor_verif', [0.1 0.3 3.5]);  % lags de motor na VERIFICACAO [s]
p.addParameter('Td_verif', 0.02);           % atraso de laco na verificacao [s]
p.addParameter('sof', true);                % refinar por realimentacao de saida (Lyapunov)
p.addParameter('merge', false);             % true: parte dos ganhos ja salvos em outdir e so' substitui iset
p.addParameter('ref_prop', false);          % false = ESTRUTURA DO MIRKO INTACTA (referencia so' pelo integrador). true = erro no termo proporcional (experimento; exige editar o modelo)
p.addParameter('lim_i', {});                % overrides de lim por planta (cell 1x9 de structs; [] = default do projeto final)
p.addParameter('Ti_i', {});                 % idem para Ti
p.parse(varargin{:}); o = p.Results;

D2R = pi/180; R2D = 180/pi;
surf = 24;          % atuador de superficie do Mirko [rad/s] (ElevActuator/Ail/rudder)
tau_thr = 0.1;      % lag de manete do controlador do Mirko [s] ("throttle 1")

% ---- limites de Bryson (defaults) ----
lim = struct( ...
    'VT', 3, 'alpha', 8*D2R, 'q', 30*D2R, 'theta', 8*D2R, 'de', 10, ...   % de em DEG (unidade do comando do theta Hold)
    'H', 5, 'thetaref', 6*D2R, 'thr', 25, ...                              % thr em %
    'beta', 6*D2R, 'p', 60*D2R, 'r', 30*D2R, 'phi', 15*D2R, 'da', 8*D2R, 'dr', 6*D2R, ...
    'psi', 25*D2R, 'phiref', 12*D2R);
% Na estrutura do Mirko a referencia entra SO' pelo integrador: a resposta a um degrau de
% referencia e' de 2a ordem com wn = sqrt(K*Gi) e zeta = Gs*sqrt(K)/(2*sqrt(Gi)) (K = ganho
% da planta) — integradores lentos (Ti = Gs/Gi grande) dao resposta sobreamortecida e
% arrastada, e a manobra de 90 deg da missao nunca "chega". Ti ~ 4-5 s p/ psi e H.
Ti = struct('theta', 1.2, 'H', 2.5, 'VT', 8, 'phi', 1.0, 'beta', 4, 'psi', 3);   % [s] "constante de tempo" do peso integral
fn = fieldnames(o.lim); for k = 1:numel(fn), lim.(fn{k}) = o.lim.(fn{k}); end
fn = fieldnames(o.Ti);  for k = 1:numel(fn), Ti.(fn{k})  = o.Ti.(fn{k});  end
lim_base = lim; Ti_base = Ti;
% Overrides POR PLANTA (defaults do projeto final, 2026-09-03): a 12 m/s (i = 1..3) a malha
% de V_T sai mais "P" (thr 25 %, Ti 16 s -> ~31 %/(m/s), integral fraca) — com os pesos das
% outras plantas a margem de fase com motor de 3,5 s caia a 15-26 deg (varredura no historico).
lim_i = cell(1,9); Ti_i = cell(1,9);
for k = 1:3, lim_i{k} = struct('thr', 25); Ti_i{k} = struct('VT', 16); end
if isfield(o, 'lim_i') && ~isempty(o.lim_i), lim_i = o.lim_i; end
if isfield(o, 'Ti_i')  && ~isempty(o.Ti_i),  Ti_i  = o.Ti_i;  end

S = load(fullfile(o.raizN, 'Dados_Trim.mat')); Plantas = S.Plantas; N = numel(Plantas);
GstateLong = cell(1,N); GintLong = cell(1,N); GstateLong_Alt = cell(1,N); GintLong_Alt = cell(1,N);
GstateLong_speed = cell(1,N); GintLong_speed = cell(1,N);
GstateLat = cell(1,N); Gintlat = cell(1,N); GstateLat_psi = cell(1,N); Gintlat_psi = cell(1,N);
R = struct('lim', lim, 'Ti', Ti, 'opts', o, 'planta', struct([]));
if o.merge && exist(fullfile(o.outdir, 'LQRY_v3.mat'), 'file')
    old = load(fullfile(o.outdir, 'LQRY_v3.mat')); old = old.LQRY3;
    GstateLong = old.GstateLong; GintLong = old.GintLong; GstateLong_Alt = old.GstateLong_Alt; GintLong_Alt = old.GintLong_Alt;
    GstateLong_speed = old.GstateLong_speed; GintLong_speed = old.GintLong_speed;
    GstateLat = old.GstateLat; Gintlat = old.Gintlat; GstateLat_psi = old.GstateLat_psi; Gintlat_psi = old.Gintlat_psi;
    R.planta = old.planta; if isfield(old, 'relatorio'), rep = old.relatorio; end
    R.lim_por_planta = struct(); if isfield(old, 'lim_por_planta'), R.lim_por_planta = old.lim_por_planta; end
end
rep = {};
say = @(varargin) fprintf(varargin{:});
if ~o.verbose, say = @(varargin) []; end

for i = o.iset
    lim = lim_base; Ti = Ti_base;
    if ~isempty(lim_i{i}), fn = fieldnames(lim_i{i}); for k = 1:numel(fn), lim.(fn{k}) = lim_i{i}.(fn{k}); end, end
    if ~isempty(Ti_i{i}),  fn = fieldnames(Ti_i{i});  for k = 1:numel(fn), Ti.(fn{k})  = Ti_i{i}.(fn{k});  end, end
    P = Plantas(i); A = P.A; B = P.B;
    ue = P.Xe(1); we = P.Xe(3); V = hypot(ue, we);
    say('\n================ Planta %d: %s (V %.1f m/s, alpha_e %.2f deg) ================\n', i, P.nome, V, atan2(we,ue)*R2D);

    %% ---------- modelo longitudinal [dVT dalpha q dtheta dH], entradas [thr(frac) de(rad)] ----------
    il = [1 3 5 8 12];
    T = [ue/V we/V 0 0 0; -we/V^2 ue/V^2 0 0 0; 0 0 1 0 0; 0 0 0 1 0; 0 0 0 0 -1];
    Al = T*A(il,il)/T; Bl = T*B(il,[1 2]);
    Bl_thr = Bl(:,1); Bl_de = Bl(:,2)*D2R;        % de passa a ser em DEG

    %% ---------- theta Hold: LQR na planta aumentada (todos os estados sao medidos) ----------
    % x5 = [dVT dalpha q dtheta de_deg], xi' = theta - theta_ref
    A5 = [Al(1:4,1:4), Bl_de(1:4); zeros(1,4), -surf]; B5 = [zeros(4,1); surf];
    Aa = [A5, zeros(5,1); [0 0 0 1 0], 0]; Ba = [B5; 0];
    Qth = diag(1./[lim.VT, lim.alpha, lim.q, lim.theta, lim.de, lim.theta*Ti.theta].^2);
    Rth = 1/lim.de^2;
    Kth = lqr(Aa, Ba, Qth, Rth);                  % u = -Kth*[x5; xi]
    Gs_th = -Kth(1:5); Gi_th = -Kth(6);
    GstateLong{i} = Gs_th; GintLong{i} = Gi_th;
    % malha fechada do theta Hold com theta_ref como entrada: x7 = [dVT dalpha q dtheta dH de xi_th]
    A7 = [Al, Bl_de, zeros(5,1);
          surf*[Gs_th(1:4) 0], surf*(Gs_th(5) - 1), surf*Gi_th;
          [0 0 0 1 0], 0, 0];
    B7_ref = [zeros(6,1); -1];                    % xi_th' = theta - theta_ref
    if o.ref_prop, B7_ref(6) = surf*Kth(4); end   % de_cmd += Gs_th(4)*(theta - theta_ref)  ->  -Gs_th(4)*theta_ref no atuador
    B7_thr = [Bl_thr; 0; 0];
    e_th = eig([A5 - B5*Kth(1:5), -B5*Kth(6); [0 0 0 1 0], 0]);

    %% ---------- Alt Hold: LQR (init) + SOF sobre a malha do theta Hold ----------
    % x8 = [x7; xi_H], xi_H' = H - H_ref ; medidos: [dVT dalpha q dtheta dH] e xi_H ; u = theta_ref [rad]
    A8 = [A7, zeros(7,1); [0 0 0 0 1 0 0], 0]; B8 = [B7_ref; 0];
    C8 = [eye(5), zeros(5,3); zeros(1,7), 1];     % 6 saidas medidas
    Qalt = diag(1./[lim.VT, lim.alpha, lim.q, lim.theta, lim.H, lim.de*3, lim.theta*Ti.theta*3, lim.H*Ti.H].^2);
    Ralt = 1/lim.thetaref^2;
    Kfull = lqr(A8, B8, Qalt, Ralt);
    K0 = Kfull([1:5 8]);
    Xalt = diag([lim.VT, lim.alpha, lim.q, lim.theta, lim.H, lim.de, lim.theta*Ti.theta, lim.H*Ti.H].^2);
    [Kalt, Jalt, okalt] = sof_refine(A8, B8, C8, Qalt, Ralt, K0, Xalt, o.sof);
    GstateLong_Alt{i} = -Kalt(1:5); GintLong_Alt{i} = -Kalt(6);
    % malha fechada theta+alt (entrada: H_ref e thr): x8
    A8c = A8 - B8*Kalt*C8;
    e_alt = eig(A8c);

    %% ---------- Vel Hold: SOF sobre theta Hold fechado + lag de manete + motor (nao medido) ----------
    % x10 = [x7 (theta_ref = 0); dthr_lag(%); T_motor(%); xi_V] ; u = thr_cmd [%] ; medidos [dVT dalpha q dtheta] e xi_V
    tm = o.tau_motor_proj;
    A10 = zeros(10); A10(1:7,1:7) = A7;
    A10(1:7,9) = B7_thr/100;                      % empuxo = T_motor/100 (fracao)
    A10(8,8) = -1/tau_thr;                        % dthr_lag' = (u - dthr_lag)/tau_thr
    A10(9,8) = 1/tm; A10(9,9) = -1/tm;            % T_motor' = (dthr_lag - T_motor)/tm
    A10(10,1) = 1;                                % xi_V' = VT - VT_ref
    B10 = zeros(10,1); B10(8) = 1/tau_thr;
    C10 = [eye(4), zeros(4,6); zeros(1,9), 1];
    Qv = diag(1./[lim.VT, lim.alpha*3, lim.q*3, lim.theta*3, lim.H*10, lim.de*3, lim.theta*Ti.theta*3, lim.thr*2, lim.thr*2, lim.VT*Ti.VT].^2);
    Rv = 1/lim.thr^2;
    Kfull = lqr(A10, B10, Qv, Rv);
    K0 = Kfull([1:4 10]);
    Xv = diag([lim.VT, lim.alpha, lim.q, lim.theta, lim.H, lim.de, lim.theta*Ti.theta, lim.thr, lim.thr, lim.VT*Ti.VT].^2);
    [Kv, Jv, okv] = sof_refine(A10, B10, C10, Qv, Rv, K0, Xv, o.sof);
    GstateLong_speed{i} = -Kv(1:4); GintLong_speed{i} = -Kv(5);
    e_vel = eig(A10 - B10*Kv*C10);

    %% ---------- modelo latero-direcional [beta p r phi psi], entradas [da dr] (rad) ----------
    ilat = [2 4 6 7 9];
    Tl = diag([1/V 1 1 1 1]);
    Ala = Tl*A(ilat,ilat)/Tl; Bla = Tl*B(ilat,[3 4]);

    %% ---------- phi Hold: LQR (init) + SOF (estados dos atuadores nao realimentados) ----------
    % x8 = [beta p r phi da dr xi_phi xi_beta] ; u = [da_cmd; dr_cmd] ; medidos [beta p r phi], [xi_phi; xi_beta]
    A8l = [Ala(1:4,1:4), Bla(1:4,:), zeros(4,2);
           zeros(2,4), -surf*eye(2), zeros(2,2);
           [0 0 0 1; 1 0 0 0], zeros(2,4)];
    B8l = [zeros(4,2); surf*eye(2); zeros(2,2)];
    C8l = [eye(4), zeros(4,4); zeros(2,6), eye(2)];
    Qphi = diag(1./[lim.beta, lim.p, lim.r, lim.phi, lim.da*2, lim.dr*2, lim.phi*Ti.phi, lim.beta*Ti.beta].^2);
    Rphi = diag(1./[lim.da, lim.dr].^2);
    Kfull = lqr(A8l, B8l, Qphi, Rphi);
    K0 = Kfull(:, [1:4 7 8]);
    Xphi = diag([lim.beta, lim.p, lim.r, lim.phi, lim.da, lim.dr, lim.phi*Ti.phi, lim.beta*Ti.beta].^2);
    [Kphi, Jphi, okphi] = sof_refine(A8l, B8l, C8l, Qphi, Rphi, K0, Xphi, o.sof);
    GstateLat{i} = -Kphi(:,1:4); Gintlat{i} = -Kphi(:,5:6);
    A8lc = A8l - B8l*Kphi*C8l;
    e_phi = eig(A8lc);

    %% ---------- psi Hold: SOF sobre phi Hold fechado ----------
    % x10 = [beta p r phi psi da dr xi_phi xi_beta xi_psi] ; u = phi_ref ; medidos [beta p r phi psi], xi_psi
    A10l = zeros(10);
    idx = [1 2 3 4 6 7 8 9];                      % posicoes de x8l dentro de x10l (psi entra na 5a)
    A10l(idx, idx) = A8lc;
    A10l(5, 1:4) = Ala(5,1:4); A10l(5,5) = Ala(5,5);  % psi' = f(r, ...)
    A10l(10, 5) = 1;                               % xi_psi' = psi - psi_ref
    B10l = zeros(10,1); B10l(8) = -1;              % xi_phi' = phi - phi_ref
    if o.ref_prop, B10l(6) = surf*Kphi(1,4); B10l(7) = surf*Kphi(2,4); end   % [da;dr] += Gs_phi(:,4)*(phi - phi_ref)
    C10l = [eye(5), zeros(5,5); zeros(1,9), 1];
    Qpsi = diag(1./[lim.beta, lim.p, lim.r, lim.phi, lim.psi, lim.da*3, lim.dr*3, lim.phi*Ti.phi*3, lim.beta*Ti.beta*3, lim.psi*Ti.psi].^2);
    Rpsi = 1/lim.phiref^2;
    Kfull = lqr(A10l, B10l, Qpsi, Rpsi);
    K0 = Kfull([1:5 10]);
    Xpsi = diag([lim.beta, lim.p, lim.r, lim.phi, lim.psi, lim.da, lim.dr, lim.phi*Ti.phi, lim.beta*Ti.beta, lim.psi*Ti.psi].^2);
    [Kpsi, Jpsi, okpsi] = sof_refine(A10l, B10l, C10l, Qpsi, Rpsi, K0, Xpsi, o.sof);
    GstateLat_psi{i} = -Kpsi(1:5); Gintlat_psi{i} = -Kpsi(6);
    e_psi = eig(A10l - B10l*Kpsi*C10l);

    %% ---------- verificacao: malha longitudinal completa (theta+alt+vel) com motor lento e atraso ----------
    ver = struct();
    for tmv = o.tau_motor_verif
        for Td = [0 o.Td_verif]
            [Acl, nomes] = long_completo(Al, Bl_thr, Bl_de, surf, tau_thr, tmv, Td, Gs_th, Gi_th, -Kalt(1:5), -Kalt(6), -Kv(1:4), -Kv(5));
            e = eig(Acl); [zmin, wn_zmin] = zeta_min(e);
            key = sprintf('long_tau%g_Td%g', tmv, Td); key = regexprep(key, '[.]', 'p');
            ver.(key) = struct('maxRe', max(real(e)), 'zeta_min', zmin, 'wn_zeta_min', wn_zmin, 'eig', e);
        end
    end
    [Acl_lat, ~] = lat_completo(Ala, Bla, surf, o.Td_verif, -Kphi(:,1:4), -Kphi(:,5:6), -Kpsi(1:5), -Kpsi(6));
    e = eig(Acl_lat); [zmin, wn_zmin] = zeta_min(e);
    ver.lat_Td = struct('maxRe', max(real(e)), 'zeta_min', zmin, 'wn_zeta_min', wn_zmin, 'eig', e);
    [Acl_lat0, ~] = lat_completo(Ala, Bla, surf, 0, -Kphi(:,1:4), -Kphi(:,5:6), -Kpsi(1:5), -Kpsi(6));
    e = eig(Acl_lat0); [zmin, wn_zmin] = zeta_min(e);
    ver.lat_Td0 = struct('maxRe', max(real(e)), 'zeta_min', zmin, 'wn_zeta_min', wn_zmin, 'eig', e);

    % resposta a REFERENCIA na estrutura do Mirko (linear): degrau de psi_ref de 90 deg e de H_ref de 20 m
    rf = struct();
    try
        [Acl_l, ~] = lat_completo(Ala, Bla, surf, 0, -Kphi(:,1:4), -Kphi(:,5:6), -Kpsi(1:5), -Kpsi(6));
        Bpsi = zeros(size(Acl_l,1),1); Bpsi(10) = -1;                          % xi_psi' = psi - psi_ref
        if o.ref_prop, Bpsi(8) = Bpsi(8) + Kpsi(5); Bpsi(6:7) = Bpsi(6:7) + 0; end %#ok<NASGU>
        Cpsi = zeros(3, size(Acl_l,1)); Cpsi(1,5) = 1;                          % psi
        Cpsi(2,:) = [-Kpsi(1:5), 0 0 0 0, -Kpsi(6), 0 0];                        % phi_ref = Gs*x5 + Gi*xi_psi (12 estados c/ Pade)
        Cpsi(3,4) = 1;                                                           % phi
        sysP = ss(Acl_l, Bpsi, Cpsi, 0);
        tt = 0:0.05:60; y = lsim(sysP, (pi/2)*ones(size(tt)), tt);
        rf.psi90 = struct('phiref_pico_deg', max(abs(y(:,2)))*R2D, 'phi_pico_deg', max(abs(y(:,3)))*R2D, ...
            't_90pct', tt(find(y(:,1) >= 0.9*pi/2, 1)), 'OS_pct', 100*(max(y(:,1)) - pi/2)/(pi/2), ...
            'erro_60s_deg', (y(end,1) - pi/2)*R2D);
        [Acl_g, ~] = long_completo(Al, Bl_thr, Bl_de, surf, tau_thr, 0.3, 0, Gs_th, Gi_th, -Kalt(1:5), -Kalt(6), -Kv(1:4), -Kv(5));
        BH = zeros(size(Acl_g,1),1); BH(8) = -1;                                 % xi_H' = H - H_ref
        CH = zeros(3, size(Acl_g,1)); CH(1,5) = 1;                               % H
        CH(2,:) = [-Kalt(1:5), 0, 0, -Kalt(6), 0 0 0 0 0];                       % theta_ref (desvio)
        CH(3,1) = 1;                                                             % dVT
        sysH = ss(Acl_g, BH, CH, 0);
        tt = 0:0.05:90; y = lsim(sysH, 20*ones(size(tt)), tt);
        rf.H20 = struct('thetaref_pico_deg', max(abs(y(:,2)))*R2D, 't_90pct', tt(find(y(:,1) >= 18, 1)), ...
            'OS_pct', 100*(max(y(:,1)) - 20)/20, 'dVT_pico', max(abs(y(:,3))), 'erro_90s_m', y(end,1) - 20);
    catch me
        rf.erro = me.message;
    end
    % margens (uma malha por vez, quebra no comando do atuador), motor 3,5 s, sem atraso
    mg = struct();
    try
        mg.de  = margem_malha(Al, Bl_thr, Bl_de, surf, tau_thr, 3.5, Gs_th, Gi_th, -Kalt(1:5), -Kalt(6), -Kv(1:4), -Kv(5), 'de');
        mg.thr = margem_malha(Al, Bl_thr, Bl_de, surf, tau_thr, 3.5, Gs_th, Gi_th, -Kalt(1:5), -Kalt(6), -Kv(1:4), -Kv(5), 'thr');
        mg.da  = margem_lat(Ala, Bla, surf, -Kphi(:,1:4), -Kphi(:,5:6), -Kpsi(1:5), -Kpsi(6), 1);
        mg.dr  = margem_lat(Ala, Bla, surf, -Kphi(:,1:4), -Kphi(:,5:6), -Kpsi(1:5), -Kpsi(6), 2);
    catch me
        mg.erro = me.message;
    end

    %% ---------- tabela "ganho por grau" (sensibilidade fisica) ----------
    tab = struct();
    tab.theta_de_deg_per_deg   = Gs_th(4)*D2R;            % deg de profundor por deg de erro de theta
    tab.q_de_deg_per_degps     = Gs_th(3)*D2R;
    tab.alpha_de_deg_per_deg   = Gs_th(2)*D2R;
    tab.int_theta_deg_per_degs = Gi_th*D2R;
    tab.theta_err_sat15        = 15/abs(Gs_th(4)*D2R);    % erro de theta [deg] que satura o profundor (15 deg)
    tab.H_thetaref_deg_per_m   = -Kalt(5)*R2D;            % deg de theta_ref por metro
    tab.theta_thetaref_per_rad = -Kalt(4);                 % termo de theta no Alt Hold (rad/rad)
    tab.alpha_thetaref_per_rad = -Kalt(2);
    tab.intH_deg_per_ms        = -Kalt(6)*R2D;
    tab.H_err_sat10deg         = 10/abs(-Kalt(5)*R2D);    % erro de H [m] p/ theta_ref de 10 deg
    tab.VT_thr_pct_per_mps     = -Kv(1);
    tab.intVT_pct_per_ms       = -Kv(5);
    tab.VT_err_sat100          = 100/abs(Kv(1));
    tab.phi_da_deg_per_deg     = -Kphi(1,4);
    tab.p_da_per_radps         = -Kphi(1,2);
    tab.r_dr_per_radps         = -Kphi(2,3);
    tab.beta_dr_per_rad        = -Kphi(2,1);
    tab.psi_phiref_deg_per_deg = -Kpsi(5);
    tab.beta_phiref_per_rad    = -Kpsi(1);
    tab.intpsi_per_rads        = -Kpsi(6);

    R.planta(i).i = i; R.planta(i).nome = P.nome; R.planta(i).V = V;
    R.planta(i).long = struct('Al', Al, 'Bl_thr', Bl_thr, 'Bl_de', Bl_de);
    R.planta(i).lat  = struct('Ala', Ala, 'Bla', Bla);
    R.planta(i).K = struct('theta', Kth, 'alt', Kalt, 'vel', Kv, 'phi', Kphi, 'psi', Kpsi);
    R.planta(i).J = struct('alt', Jalt, 'vel', Jv, 'phi', Jphi, 'psi', Jpsi);
    R.planta(i).sof_ok = struct('alt', okalt, 'vel', okv, 'phi', okphi, 'psi', okpsi);
    R.planta(i).eig = struct('theta', e_th, 'alt', e_alt, 'vel', e_vel, 'phi', e_phi, 'psi', e_psi);
    R.planta(i).verif = ver; R.planta(i).margens = mg; R.planta(i).tab = tab; R.planta(i).ref = rf;
    R.planta(i).lim = lim; R.planta(i).Ti = Ti; R.planta(i).tau_motor_proj = o.tau_motor_proj; R.planta(i).ref_prop = o.ref_prop;

    %% ---------- relatorio ----------
    L = {};
    L{end+1} = sprintf('---- Planta %d %s (V %.2f m/s) ----', i, P.nome, V);
    L{end+1} = sprintf('theta Hold : Gstate = [%s]  Gint = %.4g   (de[deg] por rad; efetivo %.3f deg/deg, %.3f deg/(deg/s) em q, int %.3f deg/(deg s); satura 15 deg com %.2f deg de erro)', ...
        num2str(Gs_th, '%.4g '), Gi_th, tab.theta_de_deg_per_deg, tab.q_de_deg_per_degps, tab.int_theta_deg_per_degs, tab.theta_err_sat15);
    L{end+1} = sprintf('             polos: %s', mat2str(e_th.', 3));
    L{end+1} = sprintf('Alt Hold   : Gstate = [%s]  Gint = %.4g   (theta_ref[rad]; %.3f deg/m, termo theta %.3f, alpha %.3f; 10 deg de theta_ref com %.1f m; SOF %s J=%.4g)', ...
        num2str(-Kalt(1:5), '%.4g '), -Kalt(6), tab.H_thetaref_deg_per_m, tab.theta_thetaref_per_rad, tab.alpha_thetaref_per_rad, tab.H_err_sat10deg, tf2str(okalt), Jalt);
    L{end+1} = sprintf('             polos dominantes: %s', mat2str(dominantes(e_alt, 6).', 3));
    L{end+1} = sprintf('Vel Hold   : Gstate = [%s]  Gint = %.4g   (thr[%%]; %.2f %%/(m/s), int %.2f %%/(m); satura 100%% com %.2f m/s; SOF %s J=%.4g; motor proj %.2g s)', ...
        num2str(-Kv(1:4), '%.4g '), -Kv(5), tab.VT_thr_pct_per_mps, tab.intVT_pct_per_ms, tab.VT_err_sat100, tf2str(okv), Jv, tm);
    L{end+1} = sprintf('             polos dominantes: %s', mat2str(dominantes(e_vel, 6).', 3));
    L{end+1} = sprintf('phi Hold   : Gstate = %s  Gint = %s   (phi->da %.3f rad/rad, p->da %.3f, r->dr %.3f, beta->dr %.3f; SOF %s)', ...
        mat2str(-Kphi(:,1:4), 4), mat2str(-Kphi(:,5:6), 4), tab.phi_da_deg_per_deg, tab.p_da_per_radps, tab.r_dr_per_radps, tab.beta_dr_per_rad, tf2str(okphi));
    L{end+1} = sprintf('             polos: %s', mat2str(e_phi.', 3));
    L{end+1} = sprintf('psi Hold   : Gstate = [%s]  Gint = %.4g   (phi_ref[rad]; psi->phi_ref %.3f rad/rad, beta %.3f; SOF %s)', ...
        num2str(-Kpsi(1:5), '%.4g '), -Kpsi(6), tab.psi_phiref_deg_per_deg, tab.beta_phiref_per_rad, tf2str(okpsi));
    L{end+1} = sprintf('             polos dominantes: %s', mat2str(dominantes(e_psi, 8).', 3));
    fnv = fieldnames(ver);
    for k = 1:numel(fnv)
        v = ver.(fnv{k});
        L{end+1} = sprintf('VERIF %-18s maxRe %+.4f  zeta_min %.3f @ %.2f rad/s  %s', fnv{k}, v.maxRe, v.zeta_min, v.wn_zeta_min, tf2str(v.maxRe < 0, 'ESTAVEL', 'INSTAVEL'));
    end
    if isfield(rf, 'psi90')
        L{end+1} = sprintf('REF psi 90 deg (linear): phi_ref pico %.1f deg, phi pico %.1f deg, 90%% em %.1f s, OS %.1f%%, erro em 60 s %.2f deg', ...
            rf.psi90.phiref_pico_deg, rf.psi90.phi_pico_deg, rf.psi90.t_90pct, rf.psi90.OS_pct, rf.psi90.erro_60s_deg);
        L{end+1} = sprintf('REF H +20 m (linear, motor 0,3 s): theta_ref pico %.1f deg, 90%% em %.1f s, OS %.1f%%, dVT pico %.2f m/s, erro em 90 s %.2f m', ...
            rf.H20.thetaref_pico_deg, rf.H20.t_90pct, rf.H20.OS_pct, rf.H20.dVT_pico, rf.H20.erro_90s_m);
    end
    if ~isfield(mg, 'erro')
        for c = {'de','thr','da','dr'}
            m = mg.(c{1});
            L{end+1} = sprintf('MARGEM %-4s (entrada da planta, motor 3,5 s): GM %.1f dB @ %.2f rad/s | PM %.1f deg @ %.2f rad/s | atraso %.0f ms', c{1}, m.GM_dB, m.wGM, m.PM, m.wPM, 1e3*m.DM);
        end
    else
        L{end+1} = ['MARGENS: erro ' mg.erro];
    end
    rep = [rep, L];
    if o.verbose, fprintf('%s\n', L{:}); end
end

R.GstateLong = GstateLong; R.GintLong = GintLong; R.GstateLong_Alt = GstateLong_Alt; R.GintLong_Alt = GintLong_Alt;
R.GstateLong_speed = GstateLong_speed; R.GintLong_speed = GintLong_speed;
R.GstateLat = GstateLat; R.Gintlat = Gintlat; R.GstateLat_psi = GstateLat_psi; R.Gintlat_psi = Gintlat_psi;
R.relatorio = rep;

%% ---------------- salva (drop-in dos Ganho_hold_*.mat) ----------------
if o.salvar
    if ~exist(o.outdir, 'dir'), mkdir(o.outdir); end
    save(fullfile(o.outdir, 'Ganho_hold_theta.mat'), 'GstateLong', 'GintLong');
    save(fullfile(o.outdir, 'Ganho_hold_H.mat'),     'GstateLong_Alt', 'GintLong_Alt');
    save(fullfile(o.outdir, 'Ganho_hold_VT.mat'),    'GstateLong_speed', 'GintLong_speed');
    save(fullfile(o.outdir, 'Ganho_hold_phi.mat'),   'GstateLat', 'Gintlat');
    save(fullfile(o.outdir, 'Ganho_hold_psi.mat'),   'GstateLat_psi', 'Gintlat_psi');
    copyfile(fullfile(o.raizN, 'Dados_Trim.mat'), fullfile(o.outdir, 'Dados_Trim.mat'));
    LQRY3 = R; LQRY3.quando = char(datetime('now')); %#ok<NASGU>
    save(fullfile(o.outdir, 'LQRY_v3.mat'), 'LQRY3');
    fid = fopen(fullfile(o.outdir, 'relatorio_projeto.txt'), 'w');
    fprintf(fid, 'LQRy v3 (re-sintese implementavel) — %s\n', char(datetime('now')));
    fprintf(fid, 'lim: %s\nTi: %s\n\n', jsonencode(lim), jsonencode(Ti));
    fprintf(fid, '%s\n', rep{:}); fclose(fid);
    if o.verbose, fprintf('\nganhos salvos em %s\n', o.outdir); end
end
end

%% ======================================================================
function [K, J, ok] = sof_refine(A, B, C, Q, Rw, K0, X, ativo)
% Realimentacao de saida estatica u = -K*y, y = C*x: minimiza J = tr(P X),
% P de Ac'P + P Ac + Q + C'K'RKC = 0 (LQRy, Stevens & Lewis). Parte de K0
% (LQR de estado completo truncado); se K0 nao estabiliza, encolhe os ganhos
% "extras" ate estabilizar. fminsearch (poucos parametros).
ok = true;
if ~ativo, K = K0; J = sof_cost(K0, A, B, C, Q, Rw, X); ok = isfinite(J) && J < 1e8; return; end
J0 = sof_cost(K0, A, B, C, Q, Rw, X);
if J0 >= 1e8
    % homotopia simples: reduz K0 ate estabilizar
    for f = [0.7 0.5 0.3 0.2 0.1 0.05 0.02 0.01]
        if sof_cost(f*K0, A, B, C, Q, Rw, X) < 1e8, K0 = f*K0; break; end
    end
end
opt = optimset('Display', 'off', 'MaxFunEvals', 40000, 'MaxIter', 20000, 'TolX', 1e-9, 'TolFun', 1e-9);
sz = size(K0);
f = @(k) sof_cost(reshape(k, sz), A, B, C, Q, Rw, X);
[k, J] = fminsearch(f, K0(:), opt);
[k, J] = fminsearch(f, k, opt);                    % 2a passada (reinicia o simplex)
K = reshape(k, sz);
if J >= 1e8, ok = false; end
end

function J = sof_cost(K, A, B, C, Q, Rw, X)
Ac = A - B*K*C; e = eig(Ac); mr = max(real(e));
if ~all(isfinite(e)) || mr >= -1e-4, J = 1e9*(1 + max(mr, 0)); return; end
P = lyap(Ac', Q + C'*K'*Rw*K*C);
J = trace(P*X);
if ~isfinite(J) || J < 0, J = 1e9; end
end

function [Acl, nomes] = long_completo(Al, Bl_thr, Bl_de, surf, tau_thr, tm, Td, Gs_th, Gi_th, Gs_a, Gi_a, Gs_v, Gi_v)
% Malha longitudinal completa: theta Hold + Alt Hold + Vel Hold, atuadores,
% motor (tau tm) e atraso Td (Pade 1a ordem em cada comando).
% x = [dVT dalpha q dtheta dH | de_deg | xi_th | xi_H | thr_lag | T_motor | xi_V | zde | zthr]
% zde/zthr: Pade de 1a ordem e^{-sTd} ~ (1 - sTd/2)/(1 + sTd/2) = -1 + (4/Td)/(s + 2/Td):  z' = -(2/Td) z + u ,  y = (4/Td) z - u
n = 13; Acl = zeros(n);
iV=1; ia=2; iq=3; ith=4; iH=5; ide=6; ixt=7; ixH=8; itl=9; iTm=10; ixV=11; izd=12; izt=13;
Acl(1:5,1:5) = Al; Acl(1:5,ide) = Bl_de; Acl(1:5,iTm) = Bl_thr/100;
% theta_ref = Gs_a*[x5] + Gi_a*xi_H ; xi_th' = theta - theta_ref
Acl(ixt, 1:5) = -Gs_a; Acl(ixt, ith) = Acl(ixt, ith) + 1; Acl(ixt, ixH) = -Gi_a;
Acl(ixH, iH) = 1;
% de_cmd = Gs_th*[x4 de] + Gi_th*xi_th  -> atraso -> atuador
de_cmd = zeros(1,n); de_cmd([1:4 ide]) = Gs_th; de_cmd(ixt) = Gi_th;
if Td > 0
    Acl(izd,:) = de_cmd; Acl(izd,izd) = Acl(izd,izd) - 2/Td;
    de_del = -de_cmd; de_del(izd) = de_del(izd) + 4/Td;
else
    de_del = de_cmd; Acl(izd,izd) = -1e3;
end
Acl(ide,:) = Acl(ide,:) + surf*de_del; Acl(ide,ide) = Acl(ide,ide) - surf;
% thr_cmd = Gs_v*[x4] + Gi_v*xi_V -> atraso -> lag tau_thr -> motor tm
th_cmd = zeros(1,n); th_cmd(1:4) = Gs_v; th_cmd(ixV) = Gi_v;
if Td > 0
    Acl(izt,:) = th_cmd; Acl(izt,izt) = Acl(izt,izt) - 2/Td;
    th_del = -th_cmd; th_del(izt) = th_del(izt) + 4/Td;
else
    th_del = th_cmd; Acl(izt,izt) = -1e3;
end
Acl(itl,:) = Acl(itl,:) + th_del/tau_thr; Acl(itl,itl) = Acl(itl,itl) - 1/tau_thr;
Acl(iTm,itl) = 1/tm; Acl(iTm,iTm) = -1/tm;
Acl(ixV,iV) = 1;
nomes = {'dVT','dalpha','q','dtheta','dH','de','xi_th','xi_H','thr_lag','T_motor','xi_V','z_de','z_thr'};
end

function [Acl, nomes] = lat_completo(Ala, Bla, surf, Td, Gs_phi, Gi_phi, Gs_psi, Gi_psi)
% x = [beta p r phi psi | da dr | xi_phi xi_beta xi_psi | zda zdr]
n = 12; Acl = zeros(n);
Acl(1:5,1:5) = Ala; Acl(1:5,6:7) = Bla;
% phi_ref = Gs_psi*[x5] + Gi_psi*xi_psi ; xi_phi' = phi - phi_ref ; xi_beta' = beta ; xi_psi' = psi
Acl(8,1:5) = -Gs_psi; Acl(8,4) = Acl(8,4) + 1; Acl(8,10) = -Gi_psi;
Acl(9,1) = 1; Acl(10,5) = 1;
% [da;dr]_cmd = Gs_phi*[beta p r phi] + Gi_phi*[xi_phi; xi_beta]
cmd = zeros(2,n); cmd(:,1:4) = Gs_phi; cmd(:,8:9) = Gi_phi;
for k = 1:2
    iz = 10+k; ia = 5+k;
    if Td > 0
        Acl(iz,:) = cmd(k,:); Acl(iz,iz) = Acl(iz,iz) - 2/Td;
        del = -cmd(k,:); del(iz) = del(iz) + 4/Td;
    else
        del = cmd(k,:); Acl(iz,iz) = -1e3;
    end
    Acl(ia,:) = Acl(ia,:) + surf*del; Acl(ia,ia) = Acl(ia,ia) - surf;
end
nomes = {'beta','p','r','phi','psi','da','dr','xi_phi','xi_beta','xi_psi','z_da','z_dr'};
end

function m = margem_malha(Al, Bl_thr, Bl_de, surf, tau_thr, tm, Gs_th, Gi_th, Gs_a, Gi_a, Gs_v, Gi_v, qual)
% Margens de uma malha por vez, com a quebra na ENTRADA DA PLANTA (depois do
% atuador/motor — onde moram o atraso do laco X-Plane e a saturacao), a outra
% malha fechada. L(s) = -(u_out/u_in), u_out = deflexao/empuxo entregue. Sem
% atraso; motor tm. DM = margem de atraso [s] = PM/wPM.
[Acl, ~] = long_completo(Al, Bl_thr, Bl_de, surf, tau_thr, tm, 0, Gs_th, Gi_th, Gs_a, Gi_a, Gs_v, Gi_v);
n = size(Acl,1); ide = 6; iTm = 10;
switch qual
    case 'de'
        Aol = Acl; Aol(1:5, ide) = 0;                  % planta deixa de ver o atuador
        Bin = zeros(n,1); Bin(1:5) = Bl_de;            % u_in = de [deg] na planta
        Cout = zeros(1,n); Cout(ide) = 1;              % u_out = de entregue pelo atuador
    case 'thr'
        Aol = Acl; Aol(1:5, iTm) = 0;
        Bin = zeros(n,1); Bin(1:5) = Bl_thr/100;       % u_in = empuxo [%] na planta
        Cout = zeros(1,n); Cout(iTm) = 1;
end
keep = setdiff(1:n, [12 13]);                      % sem os estados de Pade (inertes)
sys = ss(Aol(keep,keep), Bin(keep), Cout(keep), 0);
L = -sys;
[gm, pm, wgm, wpm] = margin(L);
m = struct('GM_dB', 20*log10(gm), 'PM', pm, 'wGM', wgm, 'wPM', wpm, 'DM', pm*pi/180/max(wpm, 1e-9));
end

function m = margem_lat(Ala, Bla, surf, Gs_phi, Gi_phi, Gs_psi, Gi_psi, canal)
% quebra na entrada da planta (canal 1 = da, 2 = dr), psi Hold fechado
[Acl, ~] = lat_completo(Ala, Bla, surf, 0, Gs_phi, Gi_phi, Gs_psi, Gi_psi);
n = size(Acl,1);
ia = 5 + canal;
Aol = Acl; Aol(1:5, ia) = 0;
Bin = zeros(n,1); Bin(1:5) = Bla(:,canal);
Cout = zeros(1,n); Cout(ia) = 1;
keep = 1:10;
sys = ss(Aol(keep,keep), Bin(keep), Cout(keep), 0);
L = -sys;
[gm, pm, wgm, wpm] = margin(L);
m = struct('GM_dB', 20*log10(gm), 'PM', pm, 'wGM', wgm, 'wPM', wpm, 'DM', pm*pi/180/max(wpm, 1e-9));
end

function [zmin, wn] = zeta_min(e)
e = e(abs(e) > 1e-6);
z = -real(e)./abs(e);
[zmin, k] = min(z); wn = abs(e(k));
end

function d = dominantes(e, n)
[~, k] = sort(abs(real(e))); d = e(k(1:min(n, numel(e))));
end

function s = tf2str(b, a, c)
if nargin < 2, a = 'ok'; c = 'FALHOU'; end
if b, s = a; else, s = c; end
end
