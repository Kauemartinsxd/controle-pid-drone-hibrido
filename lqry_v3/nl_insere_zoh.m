function info = nl_insere_zoh(mdl, Ts_io, n_delay)
%NL_INSERE_ZOH  Emula a taxa de I/O do laco X-Plane dentro do modelo NL (em memoria).
%
%   info = nl_insere_zoh('modelo_NL_LQRY_GUIA', Ts_io)
%   info = nl_insere_zoh(mdl, Ts_io, n_delay)
%
% No modelo NL o controlador ve a planta continua (passo do ode4, 10 ms) —
% e o "sensor infinitamente rapido" do SIL do Mirko. No X-Plane, o
% controlador so recebe medida nova e so entrega comando novo a cada Ts_io
% (charts read_xp/send_xp). Esta funcao poe o MESMO segurador de ordem zero
% nos dois caminhos, dentro de Planta:
%
%   Mux_Y14 --> [ZOH Ts_io] --> Demux_XP        (medidas para o controlador)
%   alpha_protection --> [ZOH Ts_io] --> Dmx_cmd4  (comando para os atuadores)
%
% Os logs Log_Y_xp / Log_U_xp continuam no sinal continuo (a planta de
% verdade). n_delay (opcional, default 0) acrescenta n amostras de atraso
% de transporte no caminho da medida, para emular a latencia do laco real
% (RTT + calculo): 1 amostra a 100 Hz ~ 10 ms.
%
% O .slx NAO e salvo: as edicoes vivem so na copia carregada em memoria,
% como faz o lqry_v3_prepara_modelo.

    if nargin < 3 || isempty(n_delay), n_delay = 0; end
    P = [mdl '/Planta'];
    info = struct('Ts_io', Ts_io, 'n_delay', n_delay);

    % --- medidas: Mux_Y14 -> ZOH (-> Delay) -> Demux_XP ---
    delete_line(P, 'Mux_Y14/1', 'Demux_XP/1');
    add_block('simulink/Discrete/Zero-Order Hold', [P '/ZOH_med'], ...
        'SampleTime', num2str(Ts_io), 'Position', [1400 360 1440 390]);
    if n_delay > 0
        add_block('simulink/Discrete/Delay', [P '/Delay_med'], ...
            'DelayLength', num2str(n_delay), 'SampleTime', num2str(Ts_io), ...
            'InitialCondition', '0', 'Position', [1460 360 1500 390]);
        add_line(P, 'Mux_Y14/1',  'ZOH_med/1',   'autorouting', 'on');
        add_line(P, 'ZOH_med/1',  'Delay_med/1', 'autorouting', 'on');
        add_line(P, 'Delay_med/1', 'Demux_XP/1', 'autorouting', 'on');
    else
        add_line(P, 'Mux_Y14/1', 'ZOH_med/1',  'autorouting', 'on');
        add_line(P, 'ZOH_med/1', 'Demux_XP/1', 'autorouting', 'on');
    end

    % --- comandos: alpha_protection -> ZOH -> Dmx_cmd4 ---
    delete_line(P, 'alpha_protection/1', 'Dmx_cmd4/1');
    add_block('simulink/Discrete/Zero-Order Hold', [P '/ZOH_cmd'], ...
        'SampleTime', num2str(Ts_io), 'Position', [560 300 590 330]);
    add_line(P, 'alpha_protection/1', 'ZOH_cmd/1',  'autorouting', 'on');
    add_line(P, 'ZOH_cmd/1',          'Dmx_cmd4/1', 'autorouting', 'on');

    fprintf('nl_insere_zoh: I/O do controlador amostrada a %.3f s (%.0f Hz), atraso %d amostra(s).\n', ...
        Ts_io, 1/Ts_io, n_delay);
end
