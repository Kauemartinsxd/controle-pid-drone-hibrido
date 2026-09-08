function status = xp_send_dh(u)
%XP_SEND_DH Envia os comandos do piloto automatico do DH ao X-Plane.
%
% Adaptado de ins_send_xplane.m (Julio Machado, PIPER-1-6), mas com a
% ORDEM DO DH (mesma da S-Function / TrimInput):
%
%   u(1) = throttle [0, 1]
%   u(2) = elevator [rad, +-0.4363]
%   u(3) = aileron  [rad, +-0.4363]
%   u(4) = rudder   [rad, +-0.4363]
%
% Converte superficies para o normalizado [-1, 1] do X-Plane
% (+-25 deg = +-0.4363 rad -> +-1) e envia via sendCTRL.

    global GlobalSocket;
    import XPlaneConnect.*;

    persistent max_elev max_ail max_rudd;

    status = 0;

    if isempty(GlobalSocket)
        try
            GlobalSocket = openUDP('127.0.0.1', 49009);
            disp('xp_send_dh: Conexao X-Plane aberta.');
        catch
            return;
        end
    end
    if ~isa(GlobalSocket, 'gov.nasa.xpc.XPlaneConnect')
        return;
    end

    try
        % Deflexoes maximas REAIS da aeronave (Plane Maker), lidas uma
        % vez dos DataRefs — o [-1,+1] normalizado mapeia para ESTES
        % limites, nao para os +-25 deg do modelo matematico. (No DH
        % atual: +-15 deg em todas as superficies.)
        if isempty(max_elev)
            lims = double(getDREFs({ ...
                'sim/aircraft/controls/acf_elev_up', ...
                'sim/aircraft/controls/acf_ail1_up', ...
                'sim/aircraft/controls/acf_rudd_lr'}, GlobalSocket));
            % VALIDACAO (2026-08-20): apos timeout UDP a fila pode entregar
            % uma resposta ORFA de outra query — ja vimos "29/0/0" aqui
            % (ail 0 => divisao por zero => lateral morto o voo inteiro).
            % Limite de superficie plausivel: 5..60 deg. Fora disso NAO
            % memoriza (tenta de novo na proxima chamada).
            if any(lims < 5) || any(lims > 60)
                fprintf('xp_send_dh: limites INVALIDOS lidos (%s) — descartando e retentando.\n', ...
                    mat2str(lims, 3));
                return;
            end
            max_elev = deg2rad(lims(1));
            max_ail  = deg2rad(lims(2));
            max_rudd = deg2rad(lims(3));
            fprintf('xp_send_dh: limites reais lidos — elev %.0f, ail %.0f, rudd %.0f deg\n', ...
                rad2deg(max_elev), rad2deg(max_ail), rad2deg(max_rudd));
        end

        % OFFSET DE TRIM REAL (2026-09-02): encontrado pelo aquecimento do
        % xp_read_dh (XP_IC.warmup) — diferenca entre o trim da planta real
        % e as ancoras do modelo. Vazio = comportamento original.
        global XP_TRIM_DELTA
        if ~isempty(XP_TRIM_DELTA), u = u(:) + XP_TRIM_DELTA(:); end

        throttle = max(0, min(1, u(1)));
        elevator = max(-1, min(1, u(2) / max_elev));
        % AILERON INVERTIDO: no modelo DH Cl_da < 0 (da positivo rola p/
        % ESQUERDA, por isso os ganhos negativos do C_phi); no X-Plane
        % aileron positivo rola p/ DIREITA. Sem esta inversao o C_phi
        % vira realimentacao POSITIVA de rolamento (divergencia).
        aileron  = max(-1, min(1, -u(3) / max_ail));
        % Leme NAO inverte: Cn_dr = +0.183 no modelo DH (dr positivo =
        % nariz p/ direita), mesma convencao do X-Plane.
        rudder   = max(-1, min(1, u(4) / max_rudd));

        % XPC: [elevator, aileron, rudder, throttle, gear, flaps]
        % -998 = "nao alterar"
        ctrl_data = [elevator, aileron, rudder, throttle, -998, -998];
        sendCTRL(ctrl_data, 0, GlobalSocket);
        status = 1;
    catch ME
        disp(['xp_send_dh: Erro no envio - ' ME.message]);
    end
end
