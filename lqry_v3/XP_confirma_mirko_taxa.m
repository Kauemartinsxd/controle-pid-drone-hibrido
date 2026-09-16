% XP_confirma_mirko_taxa.m
% =============================================================
% Voo de CONFIRMACAO do LQRy do Mirko (psi Hold novo de 2026-09-10) no gemeo
% v2, oval da GUI a 15 m/s, laco X-Plane a 100 Hz — mesma receita do voo
% XP_missao_20260910_112440_LQRYmirko_psiNovo_oval_semvento_reloadOK — e,
% no MESMO voo, medicao da taxa de atualizacao real do laco X-Plane <-> Simulink.
%
% Pre-requisitos (manuais, com o Kaue presente):
%   - X-Plane 9 aberto com o DH-Lon-REV-03.acf recem-carregado (File -> Open
%     Aircraft), motor vivo, sem pausa;
%   - vento zerado (opcional; o voo das 11:08 foi com vento e fechou 6/6).
%
% Uso (no MATLAB aberto):
%   run('C:\Users\kaue\Documents\dissertacao\dissertacao\controle-pid-drone-hibrido\lqry_v3\XP_confirma_mirko_taxa.m')
%
% Saida: o .mat do voo (formato XP_missao) + um .txt com o relatorio de taxa
% ao lado dele, com o sufixo _taxa.txt.
% =============================================================

here = fileparts(mfilename('fullpath'));
addpath(here);

%% 1) Receita do voo de 2026-09-10 (README da verificacao, secao 7)
XP3_ganhos_dir = fullfile(here, 'LQRy_Guiagem', 'ganhos_mirko');   % psi Hold novo do Mirko
XP3_ganhos     = 'orig';          % rotulo: controlador do Mirko intacto
XP3_VT         = 15;
if ~exist('CONF_Ts_io', 'var') || isempty(CONF_Ts_io), CONF_Ts_io = 0.01; end
XP3_Ts_io      = CONF_Ts_io;      % laco X-Plane: 0.01 = 100 Hz | 0.05 = 20 Hz (CONF_Ts_io antes do run)
XP3_thr0       = 0.42;            % ancoras de trim do gemeo v2 a 15 m/s
XP3_de0_deg    = 2.6;
XP3_pitch0     = 9;
XP3_autoNL     = 0;               % so o X-Plane desta vez
XP3_tag        = sprintf('LQRYmirko_psiNovo_oval_confirma_taxa_reload_%.0fHz', 1/XP3_Ts_io);
clear XP3_WPs_frame XP3_WPs_NE XP3_TimeXP
XP_auto_reload = false;           % reload manual (Kaue) — regra da secao 7.1

%% 2) Fotografia do X-Plane ANTES do voo (fps e tempo)
import XPlaneConnect.*
julioX = fullfile('C:\Users\kaue\Documents\Dissertacao_Mestrado', 'trabalho_julio', ...
    'PIPER-1-6-roll_back', 'PIPER-1-6-roll_back', 'xplane', 'XPlaneConnect-master', 'MATLAB');
addpath(julioX);
global GlobalSocket
try, closeUDP(GlobalSocket); catch, end
GlobalSocket = openUDP('127.0.0.1', 49009, 0, 500);
frp = double(getDREFs({'sim/operation/misc/frame_rate_period', ...
                       'sim/time/total_flight_time_sec'}, GlobalSocket));
fps_antes = 1/max(frp(1), 1e-6);
fprintf('X-Plane antes do voo: %.0f fps (%.2f ms por frame), t_xp = %.1f s\n', ...
    fps_antes, 1e3*frp(1), frp(2));

% RTT do getDREFs com o conjunto de 13 drefs do xp_read_dh (200 amostras)
drefs13 = {'sim/flightmodel/position/true_airspeed','sim/flightmodel/position/Prad', ...
    'sim/flightmodel/position/Qrad','sim/flightmodel/position/Rrad', ...
    'sim/flightmodel/position/phi','sim/flightmodel/position/theta', ...
    'sim/flightmodel/position/psi','sim/flightmodel/position/elevation', ...
    'sim/flightmodel/position/beta','sim/time/total_flight_time_sec', ...
    'sim/flightmodel/position/local_x','sim/flightmodel/position/local_z', ...
    'sim/flightmodel/position/alpha'};
rtt = zeros(200,1);
for k = 1:200
    t1 = tic; getDREFs(drefs13, GlobalSocket); rtt(k) = toc(t1);
end
fprintf('RTT getDREFs (13 drefs): mediana %.2f ms | p95 %.2f ms | max %.2f ms\n', ...
    1e3*median(rtt), 1e3*q_(rtt, 0.95), 1e3*max(rtt));

%% 2b) Espera o Kaue recarregar o aviao (File -> Open Aircraft) e o motor entrar em regime
% Motivo: o motor eletrico do XP9 tem ~90-150 s de estoque por load
% (PENDENCIA_MOTOR.md). O MATLAB leva ~2 min para subir; se o aviao ja
% estiver carregado, o estoque acaba no meio do oval (voo 15:46 de hoje e
% 11:16 de 10/09). Entao o script sobe primeiro e so engata depois de um
% load novo, detectado pelo tempo de voo do X-Plane voltando a zero.
T_DREF = {'sim/time/total_flight_time_sec'};
Q_DREF = {'sim/flightmodel/engine/ENGN_TRQ'};
fprintf('\n>>> AGUARDANDO Open Aircraft no X-Plane (o tempo de voo do XP vai zerar)...\n');
t_prev = double(getDREFs(T_DREF, GlobalSocket)); t_prev = t_prev(1);
t_wait0 = tic; reload_ok = false; t_now = t_prev;
while toc(t_wait0) < 900
    pause(0.5);
    try
        t_now = double(getDREFs(T_DREF, GlobalSocket)); t_now = t_now(1);
    catch
        continue;                       % XP sem responder durante o load
    end
    if t_now < t_prev - 1, reload_ok = true; break; end
    t_prev = max(t_prev, t_now);
end
if ~reload_ok, error('XP_confirma_mirko_taxa: nenhum Open Aircraft detectado em 15 min.'); end
fprintf('>>> RELOAD detectado (t_xp %.1f s). Esperando o aviao inicializar e o motor entrar em regime...\n', t_now);
trq_go = 0;
while true
    pause(0.5);
    try
        t_now = double(getDREFs(T_DREF, GlobalSocket)); t_now = t_now(1);
        q = getDREFs(Q_DREF, GlobalSocket); trq_go = double(q(1)); trq_go = trq_go(1);
    catch
        continue;
    end
    % no chao com manete zero o TRQ e ~0: basta o aviao ter inicializado.
    % O teste real do motor (manete 0,4 por 1 s) e o pre-flight do lancador.
    if t_now >= 12, break; end
end
fprintf('>>> GO: t_xp %.1f s (TRQ em idle %.2f). Engatando agora.\n', t_now, trq_go);
try, closeUDP(GlobalSocket); catch, end   % fila limpa; XP_missao_lqry3 reabre o socket

%% 3) Voa (XP_missao_lqry3 faz pre-flight, engate, missao, salva e plota)
XP_missao_lqry3

%% 4) Taxa de atualizacao REAL medida no voo
addpath(julioX); addpath(here);   % XP_missao_lqry3 restaura o path no fim
import XPlaneConnect.*
% voo.t        = grade do Simulink (Ts_io)
% voo.t_xplane = tempo do X-Plane visto em cada amostra da grade
% t_cpu        = tempo de parede da sim() (dentro de voo.cfg)
tx  = voo.t_xplane(:);
tsim = voo.t(:);
n   = numel(tx);
novo = [true; diff(tx) > 0];            % amostras em que a leitura trouxe tempo NOVO
n_novo = nnz(novo);
dur_xp = tx(end) - tx(1);
dur_sim = tsim(end) - tsim(1);
dtx = diff(tx(novo));                   % intervalo entre leituras distintas (tempo do XP)
frp2 = double(getDREFs({'sim/operation/misc/frame_rate_period'}, GlobalSocket));
fps_depois = 1/max(frp2(1), 1e-6);

L = {};
L{end+1} = sprintf('Voo: %s', XP3_lastfile);
L{end+1} = sprintf('Engate a t_xp %.1f s apos o load do .acf (motor ligado desde o load); estol/departure, se houver, acima', tx(1));
L{end+1} = sprintf('Ts_io pedido: %.3f s (%.0f Hz) | solver ode4 Ts = %.3f s', XP3_Ts_io, 1/XP3_Ts_io, Ts);
L{end+1} = sprintf('Amostras na grade do Simulink: %d em %.1f s de sim', n, dur_sim);
L{end+1} = sprintf('Leituras com tempo NOVO do X-Plane: %d de %d (%.1f %%)', n_novo, n, 100*n_novo/n);
L{end+1} = sprintf('Taxa efetiva de leitura: %.1f Hz (leituras novas / duracao XP)', (n_novo-1)/max(dur_xp, eps));
L{end+1} = sprintf('Intervalo entre leituras novas (tempo XP): mediana %.1f ms | p90 %.1f ms | p99 %.1f ms | max %.1f ms', ...
    1e3*median(dtx), 1e3*q_(dtx,0.90), 1e3*q_(dtx,0.99), 1e3*max(dtx));
L{end+1} = sprintf('Repeticoes (mesma leitura devolvida): %d (%.1f %%)', n - n_novo, 100*(n-n_novo)/n);
L{end+1} = sprintf('Sincronia: sim %.1f s | X-Plane %.1f s | razao XP/sim %.4f | parede %.1f s (razao parede/sim %.3f)', ...
    dur_sim, dur_xp, dur_xp/max(dur_sim,eps), voo.cfg.t_cpu, voo.cfg.t_cpu/max(dur_sim,eps));
L{end+1} = sprintf('X-Plane fps: antes %.0f | depois %.0f (frame %.2f / %.2f ms)', fps_antes, fps_depois, 1e3*frp(1), 1e3*frp2(1));
L{end+1} = sprintf('Frames de fisica por leitura (aprox.): %.1f', median(dtx)/frp(1));
L{end+1} = sprintf('RTT getDREFs antes do voo: mediana %.2f ms | p95 %.2f ms | max %.2f ms', ...
    1e3*median(rtt), 1e3*q_(rtt,0.95), 1e3*max(rtt));

fprintf('\n===== TAXA DE ATUALIZACAO MEDIDA =====\n');
fprintf('%s\n', L{:});
[p, f] = fileparts(XP3_lastfile);
fid = fopen(fullfile(p, [f '_taxa.txt']), 'w');
fprintf(fid, '%s\n', L{:}); fclose(fid);
fprintf('Relatorio salvo em %s\n', fullfile(p, [f '_taxa.txt']));

% histograma do intervalo entre leituras novas
fig = figure('Name', 'Taxa do laco X-Plane', 'Color', 'w');
histogram(1e3*dtx, 0:1:max(50, ceil(1e3*max(dtx))));
xlabel('intervalo entre leituras novas do X-Plane [ms]'); ylabel('amostras');
title(sprintf('Ts_{io} %.0f Hz: mediana %.1f ms, %.1f Hz efetivo', 1/XP3_Ts_io, 1e3*median(dtx), (n_novo-1)/dur_xp));
grid on
exportgraphics(fig, fullfile(p, [f '_taxa.png']), 'Resolution', 120);

function q = q_(x, p)
% quantil simples (sem Statistics Toolbox)
x = sort(x(:)); n = numel(x);
q = x(max(1, min(n, round(p*n))));
end
