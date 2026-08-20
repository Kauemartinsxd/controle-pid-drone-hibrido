% XP_malha_aberta.m
% =============================================================
% Caracterizacao em MALHA ABERTA do DH no X-Plane (sem Simulink,
% sem controlador): teleporta para MSL 600 a 12 m/s com controles
% fixos no trim e grava a resposta livre a ~10 Hz por ate 20 s.
%
% Objetivo: medir a divergencia (roll/pitch) do modelo do Plane
% Maker e a sequencia de crash/reset do X-Plane, com telemetria
% continua e sem nenhum efeito do harness Simulink.
%
% Saida: xplane/voos/XP_malha_aberta_<timestamp>.mat + resumo.
% =============================================================

xpDir = fileparts(mfilename('fullpath'));
addpath(xpDir);
addpath(fullfile(fileparts(fileparts(xpDir)), 'trabalho_julio', ...
    'PIPER-1-6-roll_back', 'PIPER-1-6-roll_back', 'xplane', ...
    'XPlaneConnect-master', 'MATLAB'));
import XPlaneConnect.*
global GlobalSocket
try, closeUDP(GlobalSocket); catch, end
GlobalSocket = openUDP('127.0.0.1', 49009, 0, 500);

thr0 = 0.28;      % trim de throttle do modelo matematico (Ue(1))
VT0  = 12;
MSL0 = 600;

%% Estado inicial + teleporte
r0 = double(getDREFs({'sim/flightmodel/position/latitude', ...
    'sim/flightmodel/position/longitude', ...
    'sim/flightmodel/position/elevation', ...
    'sim/flightmodel/position/y_agl', ...
    'sim/flightmodel/position/psi'}, GlobalSocket));
fprintf('Pre-teleporte: MSL %.1f m (AGL %.1f), proa %.0f deg\n', r0(3), r0(4), r0(5));

hdg = deg2rad(r0(5));
sendPOSI([r0(1), r0(2), MSL0, 0, 0, r0(5), -998], 0, GlobalSocket);
pause(0.1);
sendDREF('sim/flightmodel/position/local_vx',  VT0*sin(hdg), GlobalSocket);
sendDREF('sim/flightmodel/position/local_vy',  0,            GlobalSocket);
sendDREF('sim/flightmodel/position/local_vz', -VT0*cos(hdg), GlobalSocket);
sendCTRL([0, 0, 0, thr0, -998, -998], 0, GlobalSocket);

%% Leitura continua ~10 Hz por ate 20 s
drefs = { ...
    'sim/flightmodel/position/true_airspeed', ...   % 1 VT
    'sim/flightmodel/position/Prad', ...            % 2 p
    'sim/flightmodel/position/Qrad', ...            % 3 q
    'sim/flightmodel/position/Rrad', ...            % 4 r
    'sim/flightmodel/position/phi', ...             % 5 phi deg
    'sim/flightmodel/position/theta', ...           % 6 theta deg
    'sim/flightmodel/position/psi', ...             % 7 psi deg
    'sim/flightmodel/position/elevation', ...       % 8 h MSL
    'sim/time/total_flight_time_sec'};              % 9 t_xp
N = 200; D = nan(N, 10); t0 = tic; nfail = 0;
for k = 1:N
    try
        raw = double(getDREFs(drefs, GlobalSocket));
        D(k,:) = [toc(t0), raw(:)'];
    catch
        nfail = nfail + 1;
        try, closeUDP(GlobalSocket); catch, end
        GlobalSocket = [];
        try, GlobalSocket = openUDP('127.0.0.1', 49009, 0, 500); catch, end
    end
    if ~isnan(D(k,2)) && D(k,2) < 2 && D(k,1) > 3
        fprintf('VT < 2 m/s (crash/reset/repouso) em t=%.1f s — parando.\n', D(k,1));
        break;
    end
    pause(0.08);
end
D = D(~isnan(D(:,1)), :);

%% Salva e resume
voosDir = fullfile(xpDir, 'voos');
if ~isfolder(voosDir), mkdir(voosDir); end
fn = fullfile(voosDir, ['XP_malha_aberta_' datestr(now,'yyyymmdd_HHMMSS') '.mat']);
save(fn, 'D', 'thr0', 'VT0', 'MSL0');
fprintf('Salvo: %s (%d amostras, %d falhas de leitura)\n', fn, size(D,1), nfail);

fprintf('\n  t[s]  | h MSL |  VT  |  phi   | theta  |  p[deg/s] |  q     | t_xp\n');
step = max(1, round(size(D,1)/25));
for k = 1:step:size(D,1)
    fprintf(' %6.2f | %5.0f | %4.1f | %+6.1f | %+6.1f | %+8.1f | %+6.1f | %.1f\n', ...
        D(k,1), D(k,9), D(k,2), D(k,6), D(k,7), rad2deg(D(k,3)), rad2deg(D(k,4)), D(k,10));
end
