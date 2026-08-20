% XP_smoke_test.m
% =============================================================
% Teste de fumaca da conexao MATLAB <-> X-Plane (LEITURA apenas,
% nao envia nenhum comando). Pre-requisitos:
%   - X-Plane aberto com o DH (asa-fixa) carregado
%   - Plugin XPlaneConnect instalado no X-Plane
% Usa a biblioteca XPlaneConnect do repo do Julio (PIPER-1-6).
% =============================================================

clc;

%% Paths — biblioteca XPlaneConnect (lado MATLAB) do repo do Julio
julioRoot = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), ...
    'trabalho_julio', 'PIPER-1-6-roll_back', 'PIPER-1-6-roll_back');
xpcMatlab = fullfile(julioRoot, 'xplane', 'XPlaneConnect-master', 'MATLAB');
assert(isfolder(xpcMatlab), 'Biblioteca XPlaneConnect nao encontrada em: %s', xpcMatlab);
addpath(xpcMatlab);
import XPlaneConnect.*

%% Conexao (default 127.0.0.1:49009 — mesma do Julio)
fprintf('Abrindo conexao UDP com o X-Plane...\n');
socket = openUDP();

%% Leitura dos estados que a cascata PID do DH consome
drefs = { ...
    'sim/flightmodel/position/true_airspeed', ...   % VT [m/s]
    'sim/flightmodel/position/theta', ...           % pitch [deg]
    'sim/flightmodel/position/Qrad', ...            % q [rad/s]
    'sim/flightmodel/position/y_agl', ...           % h AGL [m]
    'sim/flightmodel/position/phi', ...             % roll [deg]
    'sim/flightmodel/position/Prad', ...            % p [rad/s]
    'sim/flightmodel/position/psi', ...             % heading [deg]
    'sim/flightmodel/position/Rrad', ...            % r [rad/s]
    'sim/flightmodel/position/elevation', ...       % h MSL [m]
    'sim/time/total_flight_time_sec'};              % tempo de voo [s]
result = double(getDREFs(drefs, socket));

fprintf('\n=== Estado atual do DH no X-Plane ===\n');
fprintf('  VT     = %7.2f m/s\n',  result(1));
fprintf('  theta  = %7.2f deg   q = %7.3f rad/s\n', result(2), result(3));
fprintf('  h AGL  = %7.1f m     h MSL = %7.1f m\n', result(4), result(9));
fprintf('  phi    = %7.2f deg   p = %7.3f rad/s\n', result(5), result(6));
fprintf('  psi    = %7.2f deg   r = %7.3f rad/s\n', result(7), result(8));
fprintf('  t_voo  = %7.1f s\n',   result(10));

%% Leitura dos comandos atuais (posicao dos atuadores no X-Plane)
ctrl = getCTRL(0, socket);   % 0 = aeronave do usuario
fprintf('\n=== Comandos atuais (normalizados) ===\n');
fprintf('  elevator = %+.3f | aileron = %+.3f | rudder = %+.3f | throttle = %.3f\n', ...
    double(ctrl(1)), double(ctrl(2)), double(ctrl(3)), double(ctrl(4)));

fprintf('\nConexao OK — leitura funcionando.\n');
