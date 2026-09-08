function y = xp_read_dh(cmd, t_sim)
%XP_READ_DH Le do X-Plane os estados consumidos pela cascata PID do DH.
%
% Adaptado de ins_read_xplane.m (Julio Machado, PIPER-1-6) para o
% conjunto minimo de sinais do controle do DH em modo asa-fixa.
%
% Output y (14x1):
%   y(1)  = VT       velocidade aerodinamica [m/s]
%   y(2)  = p        taxa de rolamento [rad/s]
%   y(3)  = q        taxa de arfagem [rad/s]
%   y(4)  = r        taxa de guinada [rad/s]
%   y(5)  = phi      rolamento [rad]
%   y(6)  = theta    arfagem [rad]
%   y(7)  = psi      proa RELATIVA ao engate, continua (unwrap) [rad]
%   y(8)  = h        altitude MSL [m] (= -xD do modelo, ponto de trim he)
%   y(9)  = beta     derrapagem [rad] (nao usado pelas malhas; log)
%   y(10) = t_xplane tempo de voo do X-Plane [s]
%   y(11) = xN       posicao Norte RELATIVA ao engate [m]
%   y(12) = xE       posicao Leste RELATIVA ao engate [m]
%   y(13) = psi_abs  proa ABSOLUTA [rad, [0,2pi) como o X-Plane reporta]
%   y(14) = alpha    angulo de ataque [rad] (estado dos holds do LQRY)
%
% Posicao NE (guiagem por waypoints): convencao OpenGL do XP9
% (padrao do ins_read_xplane do Julio): local_x = LESTE, local_z = SUL
%   xE = local_x - x0;  xN = -(local_z - z0)
% com a ancora (x0,z0) capturada na 1a leitura boa apos o engate
% (cmd==1 zera a ancora, ou seja, POS-teleporte — mesmo padrao do psi
% relativo). A LOS da guiagem e' calculada em NE absoluto com psi_abs;
% o psi relativo do canal 7 continua alimentando o heading hold.
%
% Input cmd:
%   0 = leitura normal
%   1 = PRIMEIRA leitura do engate (1o sample do modelo Simulink):
%       - reseta o acumulador de psi (proa relativa re-zera aqui)
%       - se a global XP_IC estiver armada (struct de XP_voo), executa o
%         TELEPORTE com a fisica RODANDO (com pauseSim a posicao nao
%         persiste) e ESPERA a leitura confirmar a nova altitude antes
%         de liberar o 1o sample — o PID ja engata com dado real.
%
% Input t_sim: tempo de simulacao do Simulink [s] — usado para o PACING
%   DE TEMPO REAL: o Simulation Pacing do Simulink NAO atua em sims via
%   sim() (so no botao Play), entao a sim rodaria ~12x mais rapido que o
%   X-Plane, inundando o UDP e defasando todas as leituras. Aqui cada
%   chamada BLOQUEIA ate o relogio de parede alcancar t_sim (ancora
%   zerada no engate) -> sim 1:1 com o X-Plane, fila UDP saudavel.
%
% Robustez (licoes das primeiras corridas):
%   - timeout UDP de 500 ms (o default de 100 ms estoura no engasgo de
%     cenario pos-teleporte);
%   - leitura que falha NUNCA devolve zeros: devolve a ultima leitura
%     boa e REABRE o socket (apos timeout a resposta atrasada fica na
%     fila e todas as leituras seguintes viriam defasadas).

    global GlobalSocket;
    import XPlaneConnect.*;

    persistent psi_acc psi_prev y_good wall_clock n_call xz0 psi_eng0;
    global XP_LIVE   % [xN xE psi_engate psi_atual t_xp] p/ rastreio ao vivo (GUI)

    if isempty(y_good), y_good = zeros(14,1); end
    y = y_good;

    %% Conexao (socket compartilhado com xp_send_dh)
    if ~ensure_socket(), return; end

    %% Engate (1o sample): reseta proa relativa e, se armado, TELEPORTA
    if nargin > 0 && cmd == 1
        psi_acc = [];
        psi_prev = [];
        xz0 = [];                    % re-ancora a posicao NE no engate
        y_good = zeros(14,1);
        global XP_IC;
        if isstruct(XP_IC)
            try
                % RETRY (2026-08-20): logo apos um Open Aircraft / reset o
                % X-Plane fica ~segundos sem responder UDP (tela de load) —
                % sem retry o teleporte falhava, desarmava, e a corrida
                % inteira rodava com o drone parado no chao.
                r0 = [];
                for k_try = 1:6
                    try
                        r0 = double(getDREFs({ ...
                            'sim/flightmodel/position/latitude', ...
                            'sim/flightmodel/position/longitude', ...
                            'sim/flightmodel/position/elevation', ...
                            'sim/flightmodel/position/y_agl', ...
                            'sim/flightmodel/position/psi'}, GlobalSocket));
                        break;
                    catch
                        fprintf('xp_read_dh: X-Plane sem resposta (tent. %d/6) — aguardando...\n', k_try);
                        pause(1.5);
                        reopen_socket();
                    end
                end
                if isempty(r0)
                    error('X-Plane nao respondeu apos 6 tentativas');
                end
                % NAO tentar "religar" o motor eletrico via dref aqui
                % (battery/ENGN_running/tacrad/pmax): testado exaustivamente
                % em 2026-08-29 — o estado do motor do XP9 e' um latch
                % fragil, os writes ora religam ora TRAVAM a helice de vez.
                % Motor morto (TRQ=0, so windmill) = pedir File->Open
                % Aircraft na UI (unico religamento confiavel).
                ground_msl = r0(3) - r0(4);
                if isfield(XP_IC,'target_msl') && ~isempty(XP_IC.target_msl)
                    target_msl = XP_IC.target_msl;
                else
                    target_msl = ground_msl + XP_IC.h0_agl;
                end
                psi0 = XP_IC.psi0;
                if isnan(psi0), psi0 = r0(5); end
                hdg = psi0*pi/180;

                pitch0 = 0;
                if isfield(XP_IC,'pitch0'), pitch0 = XP_IC.pitch0; end  % atitude de trim [deg]
                sendPOSI([r0(1), r0(2), target_msl, pitch0, 0, psi0, -998], 0, GlobalSocket);
                pause(0.1);
                sendDREF('sim/flightmodel/position/local_vx',  XP_IC.VT0*sin(hdg), GlobalSocket);
                sendDREF('sim/flightmodel/position/local_vy',  0,                  GlobalSocket);
                sendDREF('sim/flightmodel/position/local_vz', -XP_IC.VT0*cos(hdg), GlobalSocket);
                % Zera as taxas de rotacao (engate limpo mesmo se a
                % aeronave estava girando/espiralando antes do teleporte)
                sendDREF('sim/flightmodel/position/Prad', 0, GlobalSocket);
                sendDREF('sim/flightmodel/position/Qrad', 0, GlobalSocket);
                sendDREF('sim/flightmodel/position/Rrad', 0, GlobalSocket);
                de0 = 0;
                if isfield(XP_IC,'de0'), de0 = XP_IC.de0; end   % trim de profundor normalizado
                sendCTRL([de0, 0, 0, XP_IC.thr0, -998, -998], 0, GlobalSocket);

                % Espera a telemetria CONFIRMAR a nova altitude (o X-Plane
                % pode engasgar carregando cenario apos o sendPOSI)
                ok = false;
                for k = 1:30                       % ate ~4.5 s
                    pause(0.15);
                    raw = try_read();
                    if ~isempty(raw) && abs(raw(8) - target_msl) < 100
                        ok = true;
                        break;
                    end
                end
                if ok
                    fprintf('xp_read_dh: TELEPORTE confirmado — MSL %.0f m (alvo %.0f), VT %.1f m/s, proa %.0f deg.\n', ...
                        raw(8), target_msl, raw(1), psi0);
                else
                    disp('xp_read_dh: AVISO — teleporte nao confirmado pela telemetria.');
                end
                % RE-ZERA atitude/velocidade/taxas imediatamente antes de
                % liberar o 1o sample: durante o loop de confirmacao acima
                % (~1-1.5 s) o profundor fica fixo em de0 e o DH (instavel)
                % faz pitch-up ate theta ~9 deg — o PID entao engatava com
                % transitorio grande de theta, VT despencava e o voo caia
                % na armadilha do 2o regime (VT~10, thr 1.0, sink 1.7 m/s).
                % Repetir o POSI+vel+taxas aqui entrega o engate SEMPRE em
                % (pitch0, VT0, q=0) — visto 2026-08-20.
                sendPOSI([r0(1), r0(2), target_msl, pitch0, 0, psi0, -998], 0, GlobalSocket);
                sendDREF('sim/flightmodel/position/local_vx',  XP_IC.VT0*sin(hdg), GlobalSocket);
                sendDREF('sim/flightmodel/position/local_vy',  0,                  GlobalSocket);
                sendDREF('sim/flightmodel/position/local_vz', -XP_IC.VT0*cos(hdg), GlobalSocket);
                sendDREF('sim/flightmodel/position/Prad', 0, GlobalSocket);
                sendDREF('sim/flightmodel/position/Qrad', 0, GlobalSocket);
                sendDREF('sim/flightmodel/position/Rrad', 0, GlobalSocket);

                % AQUECIMENTO (2026-09-02, LQRY_XPLANE.md ADENDO 10): o
                % teleporte derruba o RPM do motor (spool ~20 s) e o LQRY
                % engatava ~1 s depois com deficit de empuxo -> throttle
                % satura -> windup -> departure. Com XP_IC.warmup > 0, um
                % trim classico (de<-theta, thr<-VT, asas niveladas; o do
                % VR_replay_xp) segura a aeronave ate o motor entrar em
                % regime e a velocidade/razao de subida convergirem; o trim
                % ENCONTRADO vira offset (XP_TRIM_DELTA, somado no
                % xp_send_dh) sobre as ancoras do modelo => engate
                % bumpless na planta REAL, sem tocar no controlador.
                if isfield(XP_IC, 'warmup') && XP_IC.warmup > 0
                    global XP_TRIM_DELTA XP_TRIM_FOUND
                    Tw = XP_IC.warmup; DTw = 0.05; th_ref0 = deg2rad(pitch0);
                    % Pareamento "atras da curva de potencia" (alpha_trim 8-14 deg):
                    % MANETE segura altitude/razao de subida; THETA segura a
                    % velocidade. (manete<-V, testado 2026-09-02 12:43, trocou
                    % altitude por velocidade: thr->0, sink -1,9 m/s, engate a 534 m.)
                    Kq = 0.50; Kth = 0.80; Kith = 0.25; Kp = 0.40; Kphi = 0.60;
                    Kt_h = 0.004; Kt_hd = 0.10; Kt_i = 0.02;   % manete <- (h, hdot), integral em hdot
                    Kv_th = deg2rad(1.5);                       % theta_ref <- (VT - VT0) [rad por m/s]
                    de_n = de0; ith = 0; thr = XP_IC.thr0; ithr = 0; h_ant = NaN; hdot_f = 0;
                    tw0 = tic; tprev = 0; nconv = 0; conv = false; TRW = zeros(0, 7);
                    % manete por JANELAS (motor do XP9 tem tau ~3,5 s medido:
                    % lei continua oscilava): a cada Tw_win s corrige a manete
                    % pela razao de subida media da janela (secante).
                    Tw_win = 8; Kw = 0.10; t_win = 0; hd_acc = 0; hd_n = 0; n_win = 0;
                    ivt = 0; Kv_i = deg2rad(0.3);          % integral lenta de theta_ref <- VT
                    % modo SIMPLES (XP_IC.warm_simple): manete FIXA na ancora e
                    % theta_ref = pitch0 por Tw s — so' para o motor sair do
                    % spool pos-teleporte (RPM tau 3,5 s, t95 5,3 s medidos)
                    % sem gastar a energia do eletrico (~130 s) procurando trim.
                    simple = isfield(XP_IC, 'warm_simple') && XP_IC.warm_simple;
                    while toc(tw0) < Tw
                        rw = try_read();
                        if isempty(rw), pause(DTw); continue; end
                        t = toc(tw0); dt = max(t - tprev, 1e-3); tprev = t;
                        if ~isnan(h_ant), hdot_f = 0.9*hdot_f + 0.1*(rw(8) - h_ant)/dt; end
                        h_ant = rw(8);
                        hdot_ref = max(-1.0, min(1.0, 0.05*(target_msl - rw(8))));
                        if t - t_win > 2, hd_acc = hd_acc + hdot_f; hd_n = hd_n + 1; end   % media da janela (descarta 2 s de transitorio)
                        if ~simple && t - t_win >= Tw_win
                            hd_w = hd_acc/max(hd_n,1); n_win = n_win + 1;
                            thr  = max(0, min(1, thr + Kw*(hdot_ref - hd_w)));
                            fprintf('xp_read_dh: aquecimento janela %d: hdot %+.2f (ref %+.2f) VT %.1f h %.0f theta %.1f -> thr %.3f\n', n_win, hd_w, hdot_ref, rw(1), rw(8), rw(6), thr);
                            if n_win >= 2 && abs(hd_w - hdot_ref) < 0.3 && abs(rw(1) - XP_IC.VT0) < 0.4 && abs(rw(8) - target_msl) < 8
                                conv = true; break;
                            end
                            t_win = t; hd_acc = 0; hd_n = 0;
                        end
                        ivt = max(-deg2rad(5), min(deg2rad(5), ivt + Kv_i*(rw(1) - XP_IC.VT0)*dt));
                        th_ref = th_ref0 + max(-deg2rad(8), min(deg2rad(8), Kv_th*(rw(1) - XP_IC.VT0) + ivt));
                        if simple, th_ref = th_ref0; thr = XP_IC.thr0; end
                        eth = deg2rad(rw(6)) - th_ref; eph = deg2rad(rw(5));
                        ith  = max(-0.5, min(0.5, ith + Kith*(-eth)*dt));
                        de_n = max(-1, min(1, ith - Kq*rw(3) - Kth*eth));
                        da_n = max(-1, min(1, -Kp*rw(2) - Kphi*eph));
                        sendCTRL([de_n, da_n, 0, thr, -998, -998], 0, GlobalSocket);
                        TRW(end+1, :) = [t, de_n, thr, rw(1), hdot_f, rw(8), rw(6)]; %#ok<AGROW>
                        resto = size(TRW,1)*DTw - toc(tw0); if resto > 0, pause(resto); end
                    end
                    ult = TRW(TRW(:,1) > toc(tw0) - 2, :);
                    lim_e = deg2rad(15);
                    try
                        lims = double(getDREFs({'sim/aircraft/controls/acf_elev_up'}, GlobalSocket));
                        if lims(1) > 5 && lims(1) < 60, lim_e = deg2rad(lims(1)); end
                    catch
                    end
                    de_f_rad = mean(ult(:,2))*lim_e;       % normalizado pelo curso REAL -> rad
                    de0_rad  = de0*deg2rad(25);            % XP_IC.de0 vem normalizado por 25 deg (lancador)
                    XP_TRIM_FOUND = struct('thr', mean(ult(:,3)), 'de_deg', rad2deg(de_f_rad), ...
                        'VT', mean(ult(:,4)), 'hdot', mean(ult(:,5)), 'h', mean(ult(:,6)), ...
                        'theta_deg', mean(ult(:,7)), 'conv', conv, 'T', toc(tw0), 'log', TRW);
                    XP_TRIM_DELTA = [XP_TRIM_FOUND.thr - XP_IC.thr0; de_f_rad - de0_rad; 0; 0];
                    if conv, s_conv = 'convergiu'; else, s_conv = 'NAO convergiu'; end
                    fprintf('xp_read_dh: AQUECIMENTO %.1f s (%s) — trim real: thr %.3f (ancora %.3f), de %+.2f deg (ancora %+.2f), VT %.1f, hdot %+.2f, theta %.1f deg, h %.0f.\n', ...
                        XP_TRIM_FOUND.T, s_conv, XP_TRIM_FOUND.thr, XP_IC.thr0, ...
                        XP_TRIM_FOUND.de_deg, rad2deg(de0_rad), XP_TRIM_FOUND.VT, XP_TRIM_FOUND.hdot, XP_TRIM_FOUND.theta_deg, XP_TRIM_FOUND.h);
                end
            catch ME
                disp(['xp_read_dh: falha no teleporte - ' ME.message]);
            end
            XP_IC = [];    % desarma (teleporta so uma vez)
            % Os timeouts do loop de confirmacao podem deixar respostas
            % orfas na fila UDP — a partir dai TODA leitura viria velha
            % (defasada N respostas). Socket novo = fila limpa.
            reopen_socket();
        end
        wall_clock = tic;   % ancora do pacing: t_sim=0 = agora
        n_call = 0;
    end
    if isempty(n_call), n_call = 0; end
    n_call = n_call + 1;

    %% PACING de tempo real: espera o relogio de parede alcancar t_sim
    if nargin > 1 && ~isempty(wall_clock)
        atraso = t_sim - toc(wall_clock);
        if atraso > 0
            pause(atraso);
        end
    end

    %% Leitura normal
    raw = try_read();
    if isempty(raw)
        % Falha: reabre o socket (limpa fila UDP defasada) e tenta 1x
        reopen_socket();
        raw = try_read();
    end
    if isempty(raw)
        y = y_good;                      % segura a ultima leitura boa
        return;
    end

    %% Conversoes e unwrap de psi
    psi_meas = deg2rad(raw(7));                 % [0, 2*pi)
    if isempty(psi_acc)
        psi_acc  = 0;                           % 1a amostra: proa relativa = 0
        psi_prev = psi_meas;
        psi_eng0 = psi_meas;                    % proa de engate (frame do mapa)
    else
        dpsi = psi_meas - psi_prev;
        dpsi = mod(dpsi + pi, 2*pi) - pi;       % delta em (-pi, pi]
        psi_acc  = psi_acc + dpsi;
        psi_prev = psi_meas;
    end

    %% Posicao NE relativa ao engate (OpenGL: local_x=Leste, local_z=Sul)
    if isempty(xz0)
        xz0 = [raw(11) raw(12)];                % ancora na 1a leitura boa
    end
    xE =  (raw(11) - xz0(1));
    xN = -(raw(12) - xz0(2));

    % SINAL DE BETA (2026-09-02, sonda de leme em voo): leme XPC +0.5 ->
    % r +3,4 deg/s, dpsi +7 deg (nariz p/ DIREITA) e beta do X-Plane +4,7 deg.
    % No modelo (beta = asin(v/VT)) nariz p/ direita da' v<0 => beta<0.
    % O dref 'beta' do XP9 e' portanto o NEGATIVO da convencao do modelo.
    % O PID nao usa beta (so' log); o LQRY realimenta beta com o MAIOR
    % ganho da linha lateral (-4,7) => com o sinal errado vira
    % realimentacao POSITIVA de derrapagem (wing rock em voo reto).
    global XP_beta_sign
    if isempty(XP_beta_sign), XP_beta_sign = -1; end   % -1 = corrigido; +1 = comportamento antigo
    y = [raw(1); raw(2); raw(3); raw(4); ...
         deg2rad(raw(5)); deg2rad(raw(6)); psi_acc; ...
         raw(8); XP_beta_sign*deg2rad(raw(9)); raw(10); ...
         xN; xE; psi_meas; deg2rad(raw(13))];
    y_good = y;
    XP_LIVE = [xN, xE, psi_eng0, psi_meas, raw(10)];   % rastreio ao vivo (GUI)

    % DEBUG: primeiros N samples do engate
    if n_call <= 20
        fprintf('[read %2d] h=%6.1f VT=%5.1f phi=%+6.1f theta=%+6.1f t_xp=%.2f\n', ...
            n_call, raw(8), raw(1), raw(5), raw(6), raw(10));
    end

    %% ----------------- funcoes locais -----------------
    function ok = ensure_socket()
        ok = true;
        if isempty(GlobalSocket)
            try
                GlobalSocket = openUDP('127.0.0.1', 49009, 0, 500);
                disp('xp_read_dh: Conexao X-Plane aberta (timeout 500 ms).');
            catch ME2
                disp(['xp_read_dh: Falha ao conectar - ' ME2.message]);
                ok = false;
                return;
            end
        end
        if ~isa(GlobalSocket, 'gov.nasa.xpc.XPlaneConnect')
            ok = false;
        end
    end

    function reopen_socket()
        try, closeUDP(GlobalSocket); catch, end
        GlobalSocket = [];
        try
            GlobalSocket = openUDP('127.0.0.1', 49009, 0, 500);
            disp('xp_read_dh: socket reaberto (fila UDP limpa).');
        catch ME2
            disp(['xp_read_dh: falha ao reabrir socket - ' ME2.message]);
        end
    end

    function raw = try_read()
        raw = [];
        try
            drefs = { ...
                'sim/flightmodel/position/true_airspeed', ... % 1: VT [m/s]
                'sim/flightmodel/position/Prad', ...          % 2: p [rad/s]
                'sim/flightmodel/position/Qrad', ...          % 3: q [rad/s]
                'sim/flightmodel/position/Rrad', ...          % 4: r [rad/s]
                'sim/flightmodel/position/phi', ...           % 5: roll [deg]
                'sim/flightmodel/position/theta', ...         % 6: pitch [deg]
                'sim/flightmodel/position/psi', ...           % 7: heading [deg]
                'sim/flightmodel/position/elevation', ...     % 8: h MSL [m]
                'sim/flightmodel/position/beta', ...          % 9: sideslip [deg]
                'sim/time/total_flight_time_sec', ...         % 10: t [s]
                'sim/flightmodel/position/local_x', ...       % 11: OpenGL x = LESTE [m]
                'sim/flightmodel/position/local_z', ...       % 12: OpenGL z = SUL [m]
                'sim/flightmodel/position/alpha'};            % 13: AoA [deg]
            raw = double(getDREFs(drefs, GlobalSocket));
        catch
            raw = [];
        end
    end
end
