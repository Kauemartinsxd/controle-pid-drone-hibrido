% XP_inicializacao.m
% =============================================================
% Prepara o workspace para voar o DH no X-Plane com a cascata PID
% (modelo xplane/modelo_XP_DH_CL.slx).
%
% FLUXO DE USO:
%   0) X-Plane aberto, DH (asa-fixa) EM VOO ~nivelado perto de
%      VT = 12 m/s (o ponto de projeto dos ganhos)
%   1) XP_inicializacao          <- este script (captura o trim atual)
%   2) out = sim('modelo_XP_DH_CL');
%   3) plot_XP_DH                (figuras da corrida)
%
% O que este script faz alem do DH_inicializacao:
%   - adiciona os paths do XPlaneConnect (repo do Julio) e desta pasta
%   - abre/testa a conexao UDP com o X-Plane
%   - captura o TRIM REAL (comandos e estado atuais do X-Plane) e
%     sobrescreve TrimInput / h_ref / refs de psi para engate bumpless
%
% Convencoes (iguais ao SIL):
%   - psi relativo ao engate (xp_read_dh zera a proa na 1a amostra)
%   - h = AGL do X-Plane; h_ref = altitude do momento da captura
%   - VT_ref = Ve (12 m/s, ponto de projeto)
% =============================================================

%% 1) Inicializacao padrao do DH (ganhos, servos, refs no trim)
% ATENCAO: DH_inicializacao faz clear/clc — nada pode ser definido antes.
run(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'DH_inicializacao.m'));

%% 2) Paths do X-Plane
xpDir     = fullfile(rootDir, 'xplane');            % rootDir vem do DH_inicializacao
julioRoot = fullfile(fileparts(rootDir), 'trabalho_julio', ...
    'PIPER-1-6-roll_back', 'PIPER-1-6-roll_back');
addpath(xpDir);
addpath(fullfile(julioRoot, 'xplane', 'XPlaneConnect-master', 'MATLAB'));
import XPlaneConnect.*

%% 3) Conexao com o X-Plane
global GlobalSocket
if ~isempty(GlobalSocket)
    try, closeUDP(GlobalSocket); catch, end
    GlobalSocket = [];
end
clear xp_read_dh xp_send_dh   % zera persistents (psi relativo, socket)
GlobalSocket = openUDP('127.0.0.1', 49009);

%% 4) Captura do estado e do trim atuais
drefs = { ...
    'sim/flightmodel/position/true_airspeed', ...
    'sim/flightmodel/position/elevation', ...    % h MSL (mesma ref do xp_read_dh)
    'sim/flightmodel/position/theta', ...
    'sim/flightmodel/position/phi', ...
    'sim/flightmodel/position/psi'};
st = double(getDREFs(drefs, GlobalSocket));
VT_now = st(1);  h_now = st(2);

ctrl = double(getCTRL(0, GlobalSocket));   % [elev ail rud thr ...] normalizado
max_deflection = 0.4363;                   % 25 deg
TrimInput = [ctrl(4); ...                  % throttle  [0,1]
             ctrl(1)*max_deflection; ...   % elevator  [rad]
             ctrl(2)*max_deflection; ...   % aileron   [rad]
             ctrl(3)*max_deflection];      % rudder    [rad]

%% 5) Referencias bumpless (sobrescreve os defaults do DH_inicializacao)
h_ref  = h_now;          % segura a altitude do engate
VT_ref = Ve;             % ponto de projeto (12 m/s)
% psi: xp_read_dh reporta proa RELATIVA (0 = proa do engate); os defaults
% psi_ref_init = psi_ref_final = 0 ja significam "segurar proa atual".

% IMPORTANTE (setpoint weighting b=0): os PIDs 2DOF de C_alt/C_vel usam
% bias -he/-Ve e b=0 — o termo P atua sobre -(y - bias), nao sobre o erro.
% Com he=600 fixo e o DH voando em outra altitude AGL, o termo P do C_alt
% ficaria com um offset enorme (theta_ref no clamp ate o integrador
% compensar). Redefinimos he para a ALTITUDE DE ENGATE: o C_alt passa a
% operar em torno do ponto de voo real, engate bumpless.
% (Ve fica em 12: o P do C_vel deve mesmo atuar contra desvios do ponto
%  de projeto — por isso engate com |VT - 12| < ~1.5 m/s.)
he = h_now;

%% 6) Parametros da simulacao X-Plane
Ts_xplane = 0.05;        % [s] taxa da interface (20 Hz, igual ao Julio)
TimeXP    = 120;         % [s] duracao default da corrida

%% 7) Resumo
fprintf('\n=== XP_inicializacao: pronto para voar no X-Plane ===\n');
fprintf('Estado capturado:  VT = %.2f m/s | h AGL = %.1f m | theta = %.1f deg | phi = %.1f deg\n', ...
    VT_now, h_now, st(3), st(4));
fprintf('Trim capturado:    thr = %.3f | de = %+.2f deg | da = %+.2f deg | dr = %+.2f deg\n', ...
    TrimInput(1), rad2deg(TrimInput(2)), rad2deg(TrimInput(3)), rad2deg(TrimInput(4)));
fprintf('Referencias:       h_ref = %.1f m | VT_ref = %.1f m/s | psi_ref = proa atual (relativa 0)\n', ...
    h_ref, VT_ref);
if VT_now < 8
    warning(['VT atual = %.1f m/s — o DH parece estar no solo ou muito lento. ' ...
        'Decole e estabilize perto de 12 m/s antes de rodar a sim.'], VT_now);
end
fprintf('\nProximo passo:  out = sim(''modelo_XP_DH_CL'');   (com a janela do modelo FECHADA)\n');
