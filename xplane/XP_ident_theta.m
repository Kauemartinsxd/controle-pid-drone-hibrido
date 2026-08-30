% XP_ident_theta.m
% =============================================================
% FASE 0 (PLANO_GUIAGEM) — identificacao da dinamica de arfagem
% q/delta_e do DH no X-Plane (.acf v3) por degraus de profundor
% em malha aberta, para o retune do C_theta.
%
% Metodo (mesma metodologia do retuning 2026-08-10 da dissertacao,
% adaptada a planta X-Plane):
%   - Cada corrida: teleporta ao ponto de operacao (MSL 600, VT 12,
%     pitch 2 deg, thr 0.8, de +2 deg), 1.2 s de settle, RE-ZERA
%     atitude/taxas (estado inicial repetivel), 0.3 s, e aplica o
%     degrau de profundor; grava q/theta/VT a ~20 Hz por 2.5 s.
%   - O DH e' INSTAVEL em malha aberta: uma corrida com degrau 0
%     (baseline) mede a deriva propria do ponto de operacao; a
%     resposta linear ao degrau e' (corrida - baseline), valida nos
%     primeiros ~2 s (curto periodo), antes dos modos lentos.
%   - Degraus: +-2 e +-3 deg (amplitudes moderadas: alpha de trim
%     do DH e' alto, degraus maiores estolam).
%
% Saida: xplane/voos/XP_ident_theta_<timestamp>.mat com todas as
% corridas. O ajuste 2a ordem + pidtune ficam em XP_retune_Ctheta.m.
%
% Manhas de harness (ver xp_read_dh.m): socket timeout 500 ms,
% reabre apos falha (fila UDP defasada), teleporte com fisica
% RODANDO, taxas zeradas no engate.
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

%% Ponto de operacao (regime do voo asa5, 2026-08-19)
MSL0    = 600;
VT0     = 12;
pitch0  = 14;             % [deg]
thr_op  = 0.50;
de_op   = 5.0;           % [deg] profundor fisico no ponto de operacao (iter-2)

steps   = [0 +2 -2 +3 -3];   % [deg] degraus (0 = baseline da deriva)
T_set   = 1.2;            % [s] settle antes do re-zero
T_rec   = 2.5;            % [s] gravacao pos-degrau
Fs      = 20;             % [Hz] alvo de amostragem

drefs = { ...
    'sim/flightmodel/position/Qrad', ...            % 1 q [rad/s]
    'sim/flightmodel/position/theta', ...           % 2 theta [deg]
    'sim/flightmodel/position/true_airspeed', ...   % 3 VT [m/s]
    'sim/flightmodel/position/elevation', ...       % 4 h MSL [m]
    'sim/time/total_flight_time_sec'};              % 5 t_xp [s]

runs = struct('step_deg',{},'D',{});

r0 = double(getDREFs({'sim/flightmodel/position/latitude', ...
    'sim/flightmodel/position/longitude', ...
    'sim/flightmodel/position/psi'}, GlobalSocket));
psi0 = r0(3);
hdg  = deg2rad(psi0);

for ir = 1:numel(steps)
    dstep = steps(ir);
    fprintf('--- corrida %d/%d: degrau de = %+g deg ---\n', ir, numel(steps), dstep);

    % 1) teleporte ao ponto de operacao (fisica rodando)
    sendPOSI([r0(1), r0(2), MSL0, pitch0, 0, psi0, -998], 0, GlobalSocket);
    pause(0.05);
    sendDREF('sim/flightmodel/position/local_vx',  VT0*sin(hdg), GlobalSocket);
    sendDREF('sim/flightmodel/position/local_vy',  0,            GlobalSocket);
    sendDREF('sim/flightmodel/position/local_vz', -VT0*cos(hdg), GlobalSocket);
    sendDREF('sim/flightmodel/position/Prad', 0, GlobalSocket);
    sendDREF('sim/flightmodel/position/Qrad', 0, GlobalSocket);
    sendDREF('sim/flightmodel/position/Rrad', 0, GlobalSocket);
    xp_send_dh([thr_op; deg2rad(de_op); 0; 0]);

    % 2) settle
    pause(T_set);

    % 3) re-zero de atitude/taxas (estado inicial repetivel; o DH
    %    instavel deriva durante o settle)
    sendPOSI([r0(1), r0(2), MSL0, pitch0, 0, psi0, -998], 0, GlobalSocket);
    sendDREF('sim/flightmodel/position/local_vx',  VT0*sin(hdg), GlobalSocket);
    sendDREF('sim/flightmodel/position/local_vy',  0,            GlobalSocket);
    sendDREF('sim/flightmodel/position/local_vz', -VT0*cos(hdg), GlobalSocket);
    sendDREF('sim/flightmodel/position/Prad', 0, GlobalSocket);
    sendDREF('sim/flightmodel/position/Qrad', 0, GlobalSocket);
    sendDREF('sim/flightmodel/position/Rrad', 0, GlobalSocket);
    pause(0.3);

    % 4) degrau + gravacao
    xp_send_dh([thr_op; deg2rad(de_op + dstep); 0; 0]);
    N = round(T_rec*Fs);
    D = nan(N, 6);                        % [t_wall q theta VT h t_xp]
    t0 = tic;
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
    runs(end+1) = struct('step_deg', dstep, 'D', D); %#ok<SAGROW>
    fprintf('    %d amostras | theta %+.1f -> %+.1f deg | VT %.1f -> %.1f\n', ...
        size(D,1), D(1,3), D(end,3), D(1,4), D(end,4));
end

% deixa o DH num estado recuperavel (nivelado com potencia de regime)
sendPOSI([r0(1), r0(2), MSL0, pitch0, 0, psi0, -998], 0, GlobalSocket);
sendDREF('sim/flightmodel/position/Qrad', 0, GlobalSocket);
xp_send_dh([thr_op; deg2rad(de_op); 0; 0]);

voosDir = fullfile(xpDir, 'voos');
if ~isfolder(voosDir), mkdir(voosDir); end
fn = fullfile(voosDir, ['XP_ident_theta_' datestr(now,'yyyymmdd_HHMMSS') '.mat']);
save(fn, 'runs', 'MSL0', 'VT0', 'pitch0', 'thr_op', 'de_op', 'steps', 'Fs');
fprintf('Identificacao salva em: %s\n', fn);
