function y = xp_read_dh(cmd, t_sim)
%XP_READ_DH Le do X-Plane os estados consumidos pela cascata PID do DH.
%
% Adaptado de ins_read_xplane.m (Julio Machado, PIPER-1-6) para o
% conjunto minimo de sinais do controle do DH em modo asa-fixa.
%
% Output y (13x1):
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

    persistent psi_acc psi_prev y_good wall_clock n_call xz0;

    if isempty(y_good), y_good = zeros(13,1); end
    y = y_good;

    %% Conexao (socket compartilhado com xp_send_dh)
    if ~ensure_socket(), return; end

    %% Engate (1o sample): reseta proa relativa e, se armado, TELEPORTA
    if nargin > 0 && cmd == 1
        psi_acc = [];
        psi_prev = [];
        xz0 = [];                    % re-ancora a posicao NE no engate
        y_good = zeros(13,1);
        global XP_IC;
        if isstruct(XP_IC)
            try
                r0 = double(getDREFs({ ...
                    'sim/flightmodel/position/latitude', ...
                    'sim/flightmodel/position/longitude', ...
                    'sim/flightmodel/position/elevation', ...
                    'sim/flightmodel/position/y_agl', ...
                    'sim/flightmodel/position/psi'}, GlobalSocket));
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

    y = [raw(1); raw(2); raw(3); raw(4); ...
         deg2rad(raw(5)); deg2rad(raw(6)); psi_acc; ...
         raw(8); deg2rad(raw(9)); raw(10); ...
         xN; xE; psi_meas];
    y_good = y;

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
                'sim/flightmodel/position/local_z'};          % 12: OpenGL z = SUL [m]
            raw = double(getDREFs(drefs, GlobalSocket));
        catch
            raw = [];
        end
    end
end
