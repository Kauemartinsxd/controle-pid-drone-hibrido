% XP_confirma_pid_taxa.m
% =============================================================
% PID em cascata da dissertacao no gemeo v2, MESMA missao dos voos do LQRy
% de 16/09/2026 (oval GUI de 6 WPs a 15 m/s, R_accept 100 m), com o laco
% X-Plane a 100 Hz — decisao da equipe: o PID passa a ser tratado a 100 Hz.
% Mede no mesmo voo a taxa de atualizacao real (xp_relatorio_taxa).
%
% Fluxo (o motor eletrico do XP9 tem ~150 s por load do .acf):
%   1) mede fps e RTT do canal;
%   2) AGUARDA o Open Aircraft no X-Plane (tempo de voo do XP zera);
%   3) 12 s depois engata (XP_missao faz pre-flight, teleporte e voo);
%   4) relatorio de taxa.
%
% Config opcional: CONF_Ts_io (0.01 | 0.05), CONF_VT0 (12), CONF_tag.
% =============================================================

here = fileparts(mfilename('fullpath'));
addpath(here);
julioX = fullfile('C:\Users\kaue\Documents\Dissertacao_Mestrado', 'trabalho_julio', ...
    'PIPER-1-6-roll_back', 'PIPER-1-6-roll_back', 'xplane', 'XPlaneConnect-master', 'MATLAB');
addpath(julioX);
import XPlaneConnect.*
if ~exist('CONF_Ts_io', 'var') || isempty(CONF_Ts_io), CONF_Ts_io = 0.01; end
if ~exist('CONF_VT0', 'var')   || isempty(CONF_VT0),   CONF_VT0 = 12;     end
if ~exist('CONF_tag', 'var')   || isempty(CONF_tag)
    CONF_tag = sprintf('PID_oval15_confirma_taxa_%.0fHz', 1/CONF_Ts_io);
end
% XP_inicializacao faz clear: tudo o que precisa sobreviver vai num .mat
pre_file = fullfile(tempdir, 'xp_confirma_pid_pre.mat');

%% 1) canal e simulador antes do voo
global GlobalSocket
try, closeUDP(GlobalSocket); catch, end
GlobalSocket = openUDP('127.0.0.1', 49009, 0, 500);
frp = double(getDREFs({'sim/operation/misc/frame_rate_period', 'sim/time/total_flight_time_sec'}, GlobalSocket));
fprintf('X-Plane antes do voo: %.0f fps (%.2f ms por frame), t_xp = %.1f s\n', 1/max(frp(1),1e-6), 1e3*frp(1), frp(2));
drefs13 = {'sim/flightmodel/position/true_airspeed','sim/flightmodel/position/Prad', ...
    'sim/flightmodel/position/Qrad','sim/flightmodel/position/Rrad', ...
    'sim/flightmodel/position/phi','sim/flightmodel/position/theta', ...
    'sim/flightmodel/position/psi','sim/flightmodel/position/elevation', ...
    'sim/flightmodel/position/beta','sim/time/total_flight_time_sec', ...
    'sim/flightmodel/position/local_x','sim/flightmodel/position/local_z', ...
    'sim/flightmodel/position/alpha'};
rtt = zeros(200,1);
for k = 1:200, t1 = tic; getDREFs(drefs13, GlobalSocket); rtt(k) = toc(t1); end
rs = sort(rtt);
fprintf('RTT getDREFs (13 drefs): mediana %.2f ms | p95 %.2f ms | max %.2f ms\n', ...
    1e3*median(rtt), 1e3*rs(round(0.95*numel(rs))), 1e3*max(rtt));
pre = struct('fps_antes', 1/max(frp(1),1e-6), 'frp', frp(1), 'rtt', rtt, ...
             'Ts_io', CONF_Ts_io, 'tag', CONF_tag);
save(pre_file, 'pre');

%% 2) espera o Open Aircraft (tempo de voo do XP volta a zero) e o aviao inicializar
T_DREF = {'sim/time/total_flight_time_sec'};
fprintf('\n>>> AGUARDANDO Open Aircraft no X-Plane (o tempo de voo do XP vai zerar)...\n');
t_prev = double(getDREFs(T_DREF, GlobalSocket)); t_prev = t_prev(1);
t_wait0 = tic; reload_ok = false; t_now = t_prev;
while toc(t_wait0) < 900
    pause(0.5);
    try, t_now = double(getDREFs(T_DREF, GlobalSocket)); t_now = t_now(1); catch, continue; end
    if t_now < t_prev - 1, reload_ok = true; break; end
    t_prev = max(t_prev, t_now);
end
if ~reload_ok, error('XP_confirma_pid_taxa: nenhum Open Aircraft detectado em 15 min.'); end
fprintf('>>> RELOAD detectado (t_xp %.1f s). Esperando o aviao inicializar...\n', t_now);
while true
    pause(0.5);
    try, t_now = double(getDREFs(T_DREF, GlobalSocket)); t_now = t_now(1); catch, continue; end
    if t_now >= 12, break; end
end
fprintf('>>> GO: t_xp %.1f s. Engatando agora.\n', t_now);
try, closeUDP(GlobalSocket); catch, end

%% 3) voo: oval do LQRy (GUI), PID a CONF_Ts_io
XP_WPs_frame = [256    0  600  15;      % mesmo oval dos voos do LQRy de 16/09
                416  160  600  15;
                256  320  600  15;
                  0  320  600  15;
               -160  160  600  15;
                  0    0  600  15];
XP_R_accept    = 100;
XP_msl0        = 600;
XP_VT0         = CONF_VT0;              % o PID engata a 12 e acelera ate a velocidade do WP
XP_Ts_io       = CONF_Ts_io;
XP_autoNL      = false;
XP_auto_reload = false;
XP_tag         = CONF_tag;
clear XP_TimeXP
XP_missao                                % faz clear internamente; 'voo' e 'vooFile' sobrevivem

%% 4) relatorio de taxa (variaveis de antes do clear vem do .mat)
import XPlaneConnect.*
global GlobalSocket
addpath(fileparts(mfilename('fullpath')));
S = load(fullfile(tempdir, 'xp_confirma_pid_pre.mat')); pre = S.pre;
try
    frp2 = double(getDREFs({'sim/operation/misc/frame_rate_period'}, GlobalSocket));
    pre.fps_depois = 1/max(frp2(1),1e-6); pre.frp2 = frp2(1);
catch
end
xp_relatorio_taxa(voo, vooFile, pre.Ts_io, pre);
