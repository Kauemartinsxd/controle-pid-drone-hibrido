% XP_sonda_equivalencia.m
% =============================================================
% FASE A da campanha de EQUIVALENCIA .acf <-> modelo da Ana (drone
% real): sondas de malha aberta no X-Plane para levantar
%   (1) LATERAL a ~12 m/s: resposta de rolagem a degrau de aileron
%       (p_ss/da, tau_roll) e dutch roll a pulso de leme (freq/zeta
%       da oscilacao de r);
%   (2) POLAR DE PLANEIO (thr=0): para cada profundor fixo, o drone
%       assenta num trim de planeio -> (VT, sink, alpha, theta).
% Cada ponto re-teleporta (o DH e' instavel em malha aberta).
% Convencoes/manhas do harness: ver xp_read_dh.m / PENDENCIA_MOTOR.md.
% Saida: xplane/voos/XP_sonda_equiv_<ts>.mat
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

r0g = double(getDREFs({'sim/flightmodel/position/latitude', ...
    'sim/flightmodel/position/longitude','sim/flightmodel/position/psi'}, GlobalSocket));

drefs_eq = { ...
    'sim/flightmodel/position/true_airspeed', ...   % 1 VT
    'sim/flightmodel/position/Prad', ...            % 2 p
    'sim/flightmodel/position/Qrad', ...            % 3 q
    'sim/flightmodel/position/Rrad', ...            % 4 r
    'sim/flightmodel/position/phi', ...             % 5 phi deg
    'sim/flightmodel/position/theta', ...           % 6 theta deg
    'sim/flightmodel/position/beta', ...            % 7 beta deg
    'sim/flightmodel/position/alpha', ...           % 8 alpha deg
    'sim/flightmodel/position/vh_ind', ...          % 9 vario m/s
    'sim/time/total_flight_time_sec'};              % 10 t_xp

S = struct();

%% (1a) rolagem: degraus de aileron a 12 m/s
% ATENCAO sinal: xp_send_dh INVERTE o aileron (convencao do modelo DH,
% Cl_da<0). O "da" aqui segue a MESMA convencao do modelo da Ana.
S.roll = struct('da',{},'D',{});
for da = [5 -5]
    fprintf('sonda rolagem: da = %+g deg...\n', da);
    eq_tele(r0g, 12, 4, 0.5, 2, 0, 0);
    pause(1.0);
    sendDREF('sim/flightmodel/position/Prad', 0, GlobalSocket);
    sendDREF('sim/flightmodel/position/Rrad', 0, GlobalSocket);
    pause(0.2);
    xp_send_dh([0.5; deg2rad(2); deg2rad(da); 0]);
    D = eq_grava(drefs_eq, 1.6, 20);
    S.roll(end+1) = struct('da', da, 'D', D);
end

%% (1b) dutch roll: pulso de leme 0.5 s e solta, grava 4 s
S.dr = struct('dr',{},'D',{});
for dr = [5 -5]
    fprintf('sonda dutch roll: pulso dr = %+g deg...\n', dr);
    eq_tele(r0g, 12, 4, 0.5, 2, 0, 0);
    pause(1.0);
    sendDREF('sim/flightmodel/position/Prad', 0, GlobalSocket);
    sendDREF('sim/flightmodel/position/Rrad', 0, GlobalSocket);
    xp_send_dh([0.5; deg2rad(2); 0; deg2rad(dr)]);
    pause(0.5);
    xp_send_dh([0.5; deg2rad(2); 0; 0]);
    D = eq_grava(drefs_eq, 4.0, 20);
    S.dr(end+1) = struct('dr', dr, 'D', D);
end

%% (2) polar de planeio: thr=0, profundor fixo -> trim de planeio
S.polar = struct('de',{},'D',{});
for de = [0 2 4 6 8]
    fprintf('sonda polar: thr=0, de = %+g deg...\n', de);
    eq_tele(r0g, 13, 0, 0, de, 0, 0);
    pause(3.0);                      % assenta no trim de planeio
    D = eq_grava(drefs_eq, 5.0, 10);
    S.polar(end+1) = struct('de', de, 'D', D);
    m = mean(D(:,2:end), 1);
    fprintf('   VT %.1f | sink %+.2f | alpha %+.1f | theta %+.1f (medias 5 s)\n', ...
        m(1), m(9), m(8), m(6));
end

% deixa o drone manso
eq_tele(r0g, 12, 3, 0.45, 2, 0, 0);

voosDir = fullfile(xpDir, 'voos');
fn = fullfile(voosDir, ['XP_sonda_equiv_' datestr(now,'yyyymmdd_HHMMSS') '.mat']);
save(fn, 'S', 'drefs_eq');
fprintf('Sondas salvas em: %s\n', fn);

%% ------------------- funcoes locais -------------------
function eq_tele(r0, VT0, pitch0, thr, de, da, dr)
    import XPlaneConnect.*
    global GlobalSocket
    hdg = deg2rad(r0(3));
    sendPOSI([r0(1), r0(2), 600, pitch0, 0, r0(3), -998], 0, GlobalSocket);
    pause(0.05);
    sendDREF('sim/flightmodel/position/local_vx',  VT0*sin(hdg), GlobalSocket);
    sendDREF('sim/flightmodel/position/local_vy',  0,            GlobalSocket);
    sendDREF('sim/flightmodel/position/local_vz', -VT0*cos(hdg), GlobalSocket);
    sendDREF('sim/flightmodel/position/Prad', 0, GlobalSocket);
    sendDREF('sim/flightmodel/position/Qrad', 0, GlobalSocket);
    sendDREF('sim/flightmodel/position/Rrad', 0, GlobalSocket);
    xp_send_dh([thr; deg2rad(de); deg2rad(da); deg2rad(dr)]);
end

function D = eq_grava(drefs, T, Fs)
    import XPlaneConnect.*
    global GlobalSocket
    N = round(T*Fs); D = nan(N, 11); t0 = tic;
    for k = 1:N
        try
            raw = double(getDREFs(drefs, GlobalSocket));
            D(k,:) = [toc(t0), raw(:)'];
        catch
            try, closeUDP(GlobalSocket); catch, end
            GlobalSocket = [];
            try, GlobalSocket = openUDP('127.0.0.1', 49009, 0, 500); catch, end
        end
        dt = k/Fs - toc(t0);
        if dt > 0, pause(dt); end
    end
    D = D(~isnan(D(:,1)),:);
end
