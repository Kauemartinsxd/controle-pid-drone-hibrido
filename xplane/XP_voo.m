% XP_voo.m
% =============================================================
% Lancador do voo do DH no X-Plane — versao "teleporte no engate":
%
%   1) prepara o workspace (ganhos, socket, refs) e compila o modelo
%   2) ARMA a condicao inicial na global XP_IC (nao teleporta ainda)
%   3) roda a sim: o 1o sample do read_xp executa o teleporte com a
%      fisica RODANDO e o PID assume ~0.3 s depois — nao ha janela
%      de malha aberta para o DH (instavel em arfagem) divergir.
%
% Por que assim: teleporte com pauseSim NAO persiste (a posicao e'
% descartada ao despausar) e o sim() leva ~5 s para dar a partida —
% tempo de sobra para o DH do Plane Maker mergulhar de 150 m.
% (O Piper do Julio, docil, tolerava esse gap; o DH nao.)
%
% Uso:
%   XP_voo        % defaults: MSL 600 m (altitude do trim), 12 m/s, 60 s
%
% Config opcional (defina antes de rodar):
%   XP_msl0   [m]   altitude MSL inicial   (default 600 = he do trim)
%   XP_VT0    [m/s] velocidade inicial     (default Ve = 12)
%   XP_TimeXP [s]   duracao da corrida     (default 60)
%
% Pre-requisito: X-Plane com o DH em qualquer lugar (pode estar no
% solo), SEM tela de crash (se acidentou: Reset Flight antes).
% =============================================================

%% Config do usuario (preservada atraves do clear do DH_inicializacao)
if ~exist('XP_msl0','var')   || isempty(XP_msl0),   XP_msl0   = 600; end
if ~exist('XP_VT0','var')    || isempty(XP_VT0),    XP_VT0    = 12;  end
if ~exist('XP_TimeXP','var') || isempty(XP_TimeXP), XP_TimeXP = 60;  end
if ~exist('XP_VT_ref','var') || isempty(XP_VT_ref), XP_VT_ref = NaN; end  % NaN = Ve (default)
% overrides de ANCORA p/ campanhas de equivalencia (.acf mudando de trim):
if ~exist('XP_Xe8_deg','var')     || isempty(XP_Xe8_deg),     XP_Xe8_deg = NaN; end      % centro de theta_ref
if ~exist('XP_clamp_hi_deg','var')|| isempty(XP_clamp_hi_deg),XP_clamp_hi_deg = NaN; end % teto do clamp
% MANOBRAS G4 (degraus identicos aos do SIL — blocos Step_* do modelo,
% mesmos parametros do DH_inicializacao; defaults INERTES):
if ~exist('XP_h_step_final','var') || isempty(XP_h_step_final), XP_h_step_final = 0; end
if ~exist('XP_h_step_t','var')     || isempty(XP_h_step_t),     XP_h_step_t = 10;    end
if ~exist('XP_psi_step_deg','var') || isempty(XP_psi_step_deg), XP_psi_step_deg = 0; end
if ~exist('XP_psi_step_t','var')   || isempty(XP_psi_step_t),   XP_psi_step_t = 5;   end
if ~exist('XP_VT_step_delta','var')|| isempty(XP_VT_step_delta),XP_VT_step_delta = 0;end
if ~exist('XP_VT_step_t','var')    || isempty(XP_VT_step_t),    XP_VT_step_t = 5;    end
% DOUBLETS (G5): t2/t3 dos canais h e psi (blocos Step_*2/3 do modelo,
% mesmos parametros do DH_inicializacao; 1e9 = INERTE = degrau simples):
if ~exist('XP_h_step_t2','var')   || isempty(XP_h_step_t2),   XP_h_step_t2 = 1e9;   end
if ~exist('XP_h_step_t3','var')   || isempty(XP_h_step_t3),   XP_h_step_t3 = 1e9;   end
if ~exist('XP_psi_step_t2','var') || isempty(XP_psi_step_t2), XP_psi_step_t2 = 1e9; end
if ~exist('XP_psi_step_t3','var') || isempty(XP_psi_step_t3), XP_psi_step_t3 = 1e9; end
setpref('XP_DH','cfg',[XP_msl0 XP_VT0 XP_TimeXP XP_VT_ref XP_Xe8_deg XP_clamp_hi_deg ...
    XP_h_step_final XP_h_step_t XP_psi_step_deg XP_psi_step_t ...
    XP_VT_step_delta XP_VT_step_t ...
    XP_h_step_t2 XP_h_step_t3 XP_psi_step_t2 XP_psi_step_t3]);  % sobrevive ao clear

%% 1) Workspace completo (ganhos, socket, refs) — faz clear/bdclose
run(fullfile(fileparts(mfilename('fullpath')), 'XP_inicializacao.m'));

cfg = getpref('XP_DH','cfg');
XP_msl0 = cfg(1); XP_VT0 = cfg(2); TimeXP = cfg(3);
XP_VT_ref = NaN; if numel(cfg) >= 4, XP_VT_ref = cfg(4); end
XP_Xe8_deg = NaN; XP_clamp_hi_deg = NaN;
if numel(cfg) >= 6, XP_Xe8_deg = cfg(5); XP_clamp_hi_deg = cfg(6); end
if numel(cfg) >= 12   % manobras G4 (aplicadas sobre os params do DH_inicializacao)
    h_step_final = cfg(7);            h_step_t  = cfg(8);
    psi_ref_final = deg2rad(cfg(9));  psi_ref_t = cfg(10);
    VT_step_delta = cfg(11);          VT_step_t = cfg(12);
end
if numel(cfg) >= 16   % doublets G5 (t2/t3 dos blocos Step_*2/3; 1e9 = inerte)
    h_step_t2  = cfg(13);  h_step_t3  = cfg(14);
    psi_ref_t2 = cfg(15);  psi_ref_t3 = cfg(16);
end
rmpref('XP_DH','cfg');

%% 2) PRE-FLIGHT CHECK: aeronave inteira, nivelada no solo, X-Plane rodando
import XPlaneConnect.*
global GlobalSocket
r0 = double(getDREFs({'sim/flightmodel/position/elevation', ...
                      'sim/flightmodel/position/y_agl', ...
                      'sim/flightmodel/position/theta', ...
                      'sim/flightmodel/position/phi', ...
                      'sim/flightmodel/position/true_airspeed', ...
                      'sim/time/total_flight_time_sec'}, GlobalSocket));
pause(0.6);
t2 = double(getDREFs({'sim/time/total_flight_time_sec'}, GlobalSocket));
if t2 <= r0(6)
    error(['XP_voo: tempo do X-Plane CONGELADO (tela de crash/pausa). ' ...
        'Faca Reset Flight no X-Plane e rode de novo.']);
end
% O teleporte-no-engate normaliza posicao/atitude/velocidade/taxas a
% partir de QUALQUER estado com fisica rodando (mesma logica do
% XP_missao, 2026-08-20) — o pre-flight bloqueia simulador travado e
% reload que nao pegou.
if r0(2) >= 3 && r0(5) < 3
    % AGL alto com VT ~0 = wreck parado no morro: o File->Open Aircraft
    % nao pegou e o motor esta morto (voo sairia como planeio invalido —
    % visto em 2026-08-30 e 2026-08-31).
    error(['XP_voo: AGL %.1f m com VT %.1f m/s — reload que NAO pegou ' ...
        '(wreck no morro, motor morto). Faca File -> Open Aircraft de novo ' ...
        'e confira o aviao NA PISTA com a helice girando.'], r0(2), r0(5));
end
if ~(r0(2) < 3 && r0(5) < 3)
    fprintf(['XP_voo: engatando A PARTIR DE VOO (AGL=%.1f m, VT=%.1f m/s, ' ...
        'phi=%.1f deg) — o teleporte normaliza o estado.\n'], r0(2), r0(5), r0(4));
end
fprintf('XP_voo: pre-flight OK (X-Plane rodando).\n');
ground_msl = r0(1) - r0(2);
if XP_msl0 - ground_msl < 120
    warning(['Solo local em %.0f m MSL: MSL0 = %.0f m daria so %.0f m AGL. ' ...
        'Subindo para %.0f m MSL (150 m AGL).'], ground_msl, XP_msl0, ...
        XP_msl0-ground_msl, ground_msl+150);
    XP_msl0 = ground_msl + 150;
end

%% 3) Referencias e trim da condicao inicial — CONFIG DOURADA 2026-08-19
% h de realimentacao e' MSL (= -xD do modelo): voar a MSL 600 m
% coloca a malha EXATAMENTE no ponto de trim do projeto (he = 600).
% Ancoras adaptadas ao .acf v3 (massa 2.2 kg, superficies 25 deg,
% h-stab -2, asa +5, helice passo +3):
%   - Xe(8) = 2 deg  : atitude de trim real do DH no X-Plane
%   - clamp teto +3  : theta_ref max = 5 deg — protecao de energia
%     (acima de ~7 deg o arrasto deste .acf supera o empuxo; teto 6
%      permite overshoot ate o estol da asa)
% ANCORAS DO GEMEO v1 (2026-08-30): trim do .acf equivalente = trim do
% projeto (alpha/theta 14.4, de +7, thr 0.43) — ver EQUIVALENCIA_ACF.md
XP_thr0   = 0.43;                % throttle de trim do gemeo
XP_de0    = Ue(2)/deg2rad(25);   % profundor de trim (mat. = gemeo!) norm.
XP_pitch0 = 14;                  % atitude de trim [deg]
TrimInput = [0.43; Ue(2); 0; 0];
Xe(8) = deg2rad(14.3);
theta_ref_clamp = [-0.1745  deg2rad(3)];   % theta_ref [4.3, 17.3] (estol ~18.5)
if ~isnan(XP_Xe8_deg)
    Xe(8) = deg2rad(XP_Xe8_deg);
    XP_pitch0 = XP_Xe8_deg;                       % teleporta ja na atitude alvo
end
if ~isnan(XP_clamp_hi_deg), theta_ref_clamp(2) = deg2rad(XP_clamp_hi_deg); end
h_ref  = XP_msl0;               % altitude MSL alvo
he     = XP_msl0;               % bias dos PIDs b=0 no ponto de voo
VT_ref = Ve;                    % 12 m/s — ponto de projeto
if ~isnan(XP_VT_ref), VT_ref = XP_VT_ref; end   % override p/ mapa de trim

%% 4) Compila o modelo e ARMA o teleporte-no-engate
mdl = 'modelo_XP_DH_CL';
load_system(mdl);
set_param(mdl, 'SimulationCommand', 'update');

global XP_IC
XP_IC = struct('target_msl', XP_msl0, 'h0_agl', XP_msl0-ground_msl, ...
               'VT0', XP_VT0, 'psi0', NaN, 'thr0', XP_thr0, ...
               'de0', XP_de0, 'pitch0', XP_pitch0);
fprintf(['XP_voo: modelo compilado, teleporte ARMADO (MSL %.0f m = AGL %.0f m, ' ...
    '%.1f m/s).\nEngatando...\n'], XP_msl0, XP_msl0-ground_msl, XP_VT0);

%% 5) Voa
out = sim(mdl);

%% 6) Salva o voo (dados + configuracao) para analise posterior
voosDir = fullfile(rootDir, 'xplane', 'voos');
if ~isfolder(voosDir), mkdir(voosDir); end
voo = struct();
voo.quando     = datestr(now, 'yyyy-mm-dd HH:MM:SS');
voo.Y          = squeeze(out.Y_xp);      % 13 x T: [VT p q r phi theta psi h beta t_xp xN xE psi_abs]
voo.U          = out.U_xp;               % T x 4:  [thr de da dr]
voo.t          = out.tout(:);
voo.t_xplane   = out.t_xplane_log(:);
if isprop(out,'theta_ref_log') || isfield(out,'theta_ref_log')
    voo.theta_ref = out.theta_ref_log;   % saida do C_alt (pos-clamp)
end
voo.cfg = struct('XP_msl0',XP_msl0, 'XP_VT0',XP_VT0, 'TimeXP',TimeXP, ...
    'h_step',[h_step_final h_step_t], 'psi_step',[rad2deg(psi_ref_final) psi_ref_t], ...
    'VT_step',[VT_step_delta VT_step_t], ...
    'h_dbl_t23',[h_step_t2 h_step_t3], 'psi_dbl_t23',[psi_ref_t2 psi_ref_t3], ...
    'h_ref',h_ref, 'he',he, 'VT_ref',VT_ref, 'TrimInput',TrimInput, ...
    'C_theta',C_theta, 'C_vel',C_vel, 'C_alt',C_alt, 'C_phi',C_phi, ...
    'Kq',Kq, 'Kp',Kp, 'Kr',Kr, 'K_heading',K_heading, ...
    'theta_ref_clamp',theta_ref_clamp, 'tau_ref',tau_ref, ...
    'act',act, 'eng',eng);
vooFile = fullfile(voosDir, ['XP_voo_' datestr(now,'yyyymmdd_HHMMSS') '.mat']);
save(vooFile, 'voo');
fprintf('Voo salvo em: %s\n', vooFile);

%% 7) Resumo
Y = squeeze(out.Y_xp); U = out.U_xp; t = out.tout(:);
dtx = out.t_xplane_log(end) - out.t_xplane_log(1);
fprintf('\n===== XP_voo: resumo =====\n');
fprintf('Sync : sim %.1f s | X-Plane avancou %.1f s | razao %.3f\n', t(end), dtx, dtx/max(t(end),eps));
fprintf('VT   : engate %.1f | min %.1f | max %.1f | final %.2f m/s (ref %.1f)\n', ...
    Y(1,1), min(Y(1,:)), max(Y(1,:)), Y(1,end), VT_ref);
fprintf('h AGL: engate %.1f | min %.1f | max %.1f | final %.2f m (ref %.1f)\n', ...
    Y(8,1), min(Y(8,:)), max(Y(8,:)), Y(8,end), h_ref);
fprintf('theta: %.1f a %.1f deg | phi max |%.1f| deg | psi max |%.1f| deg\n', ...
    rad2deg(min(Y(6,:))), rad2deg(max(Y(6,:))), ...
    rad2deg(max(abs(Y(5,:)))), rad2deg(max(abs(Y(7,:)))));
fprintf('thr  : %.2f a %.2f | de: %+.1f a %+.1f deg\n', ...
    min(U(:,1)), max(U(:,1)), rad2deg(min(U(:,2))), rad2deg(max(U(:,2))));
fprintf('Figuras: rode plot_XP_DH\n');
