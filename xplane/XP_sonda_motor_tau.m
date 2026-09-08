% XP_sonda_motor_tau.m — constante de tempo do motor do XP9 em VOO.
% Teleporta a 600 m / 15 m/s, segura com o trim classico (aquecimento do
% xp_read_dh, XP_IC.warmup), depois aplica um DEGRAU de manete
% (thr_trim -> thr_trim + 0.4) e loga TRQ / RPM / empuxo a 20 Hz por 12 s.
% Saida: struct MT (t, thr, trq, rpm, thrust, VT) salvo em voos/XP_sonda_motor_tau_*.mat
xpDir  = 'C:\Users\kaue\Documents\Dissertacao_Mestrado\controle-pid-drone-hibrido\xplane';
julioX = fullfile('C:\Users\kaue\Documents\Dissertacao_Mestrado', 'trabalho_julio', ...
    'PIPER-1-6-roll_back', 'PIPER-1-6-roll_back', 'xplane', 'XPlaneConnect-master', 'MATLAB');
addpath(xpDir); addpath(julioX); import XPlaneConnect.*
global GlobalSocket XP_IC XP_TRIM_FOUND XP_TRIM_DELTA
clear xp_read_dh xp_send_dh; XP_TRIM_DELTA = []; XP_TRIM_FOUND = [];
if ~exist('XP_mt_reload','var') || isempty(XP_mt_reload), XP_mt_reload = true; end
if XP_mt_reload, XP_auto_reload = true; xp_reload_acf; pause(1); end
try, closeUDP(GlobalSocket); catch, end
GlobalSocket = openUDP('127.0.0.1', 49009, 0, 500);
r0 = double(getDREFs({'sim/flightmodel/position/elevation','sim/flightmodel/position/y_agl'}, GlobalSocket));
ground = r0(1) - r0(2);
XP_IC = struct('target_msl', 600, 'h0_agl', 600 - ground, 'VT0', 15, 'psi0', NaN, ...
    'thr0', 0.337, 'de0', deg2rad(2.22)/deg2rad(25), 'pitch0', 8.0, 'warmup', 40);
y = xp_read_dh(1, 0);          % teleporte + aquecimento (trim real em XP_TRIM_FOUND)
thr0 = XP_TRIM_FOUND.thr; de0n = XP_TRIM_FOUND.de_deg/15;
fprintf('sonda motor: trim thr %.3f de %.2f deg, VT %.1f, h %.0f\n', thr0, XP_TRIM_FOUND.de_deg, XP_TRIM_FOUND.VT, XP_TRIM_FOUND.h);
drefs = {'sim/flightmodel/engine/ENGN_TRQ', 'sim/flightmodel/engine/ENGN_tacrad', ...
         'sim/flightmodel/engine/POINT_thrust', 'sim/flightmodel/position/true_airspeed', ...
         'sim/flightmodel/engine/ENGN_thro', 'sim/flightmodel/position/theta'};
DT = 0.05; T1 = 3; T2 = 15; MT = []; t0 = tic; k = 0;
while toc(t0) < T2
    k = k + 1; t = toc(t0);
    thr = thr0; if t >= T1, thr = min(1, thr0 + 0.4); end
    sendCTRL([de0n, 0, 0, thr, -998, -998], 0, GlobalSocket);
    try
        v = getDREFs(drefs, GlobalSocket); row = zeros(1, numel(drefs));
        for j = 1:numel(drefs), q = v{j}; row(j) = double(q(1)); end
    catch, row = nan(1, numel(drefs)); end
    MT(k, :) = [t, thr, row]; %#ok<AGROW>
    resto = k*DT - toc(t0); if resto > 0, pause(resto); end
end
sendCTRL([de0n, 0, 0, thr0, -998, -998], 0, GlobalSocket);
MTs = struct('t', MT(:,1), 'thr', MT(:,2), 'trq', MT(:,3), 'rpm', MT(:,4), 'thrust', MT(:,5), ...
             'VT', MT(:,6), 'thro_dref', MT(:,7), 'theta', MT(:,8), 'trim', rmfield(XP_TRIM_FOUND, 'log'));
save(fullfile(xpDir, 'voos', ['XP_sonda_motor_tau_' datestr(now, 'yyyymmdd_HHMMSS') '.mat']), 'MTs');
% constante de tempo (63 %) de TRQ, RPM e empuxo apos o degrau
for nm = {'trq', 'rpm', 'thrust'}
    x = MTs.(nm{1}); tt = MTs.t; i1 = find(tt >= T1, 1); x0 = mean(x(tt > T1-1 & tt < T1)); xf = mean(x(tt > T2-2));
    k63 = find(x(i1:end) >= x0 + 0.632*(xf - x0), 1); k95 = find(x(i1:end) >= x0 + 0.95*(xf - x0), 1);
    if isempty(k63), t63 = NaN; else, t63 = tt(i1+k63-1) - T1; end
    if isempty(k95), t95 = NaN; else, t95 = tt(i1+k95-1) - T1; end
    fprintf('%-7s %8.3f -> %8.3f | tau63 = %.2f s | t95 = %.2f s\n', nm{1}, x0, xf, t63, t95);
end
fprintf('VT %.1f -> %.1f m/s em %.0f s\n', MTs.VT(find(MTs.t>=T1,1)), MTs.VT(end), T2-T1);
