function info = xp_set_ts_io(mdl, Ts_io)
%XP_SET_TS_IO  Taxa da janela de I/O com o X-Plane (read_xp / send_xp), em memoria.
%
%   info = xp_set_ts_io('modelo_XP_DH_GUIA', 0.01)   % laco a 100 Hz
%
% Os blocos MATLAB Function read_xp e send_xp dos modelos modelo_XP_* tem
% o periodo gravado LITERALMENTE no script ("Ts = 0.05;") e so chamam
% xp_read_dh / xp_send_dh quando floor(t/Ts) muda. Mudar apenas o
% SampleTime do bloco NAO muda a taxa real (ADENDO 8 do LQRY_XPLANE.md).
% Esta funcao ajusta os dois: SampleTime e o literal do script. O .slx
% nao e salvo — vale para a copia carregada em memoria.
%
% Decisao da equipe (2026-09-16): o PID passa a voar com o laco a 100 Hz
% (Ts_io = 0,01 s), o mesmo do LQRy. O solver (ode4, 5 ms) nao muda.
%
% Mesma edicao que lqry_v3_prepara_modelo faz para o LQRy, mas procurando
% os charts pelo NOME em qualquer nivel do modelo (no GUIA do PID eles
% ficam na raiz; no LQRY2 ficam em Planta/).

    if nargin < 2 || isempty(Ts_io), Ts_io = 0.01; end
    rt = sfroot;
    info = struct('Ts_io', Ts_io, 'charts', {{}});
    for nm = {'read_xp', 'send_xp'}
        chs = rt.find('-isa', 'Stateflow.EMChart', 'Name', nm{1});
        ch = [];
        for k = 1:numel(chs)
            if startsWith(chs(k).Path, [mdl '/']), ch = chs(k); break; end
        end
        assert(~isempty(ch), 'xp_set_ts_io: chart %s nao encontrado em %s', nm{1}, mdl);
        ch.SampleTime = num2str(Ts_io);
        scr = ch.Script;
        scr2 = regexprep(scr, 'Ts\s*=\s*[0-9.]+\s*;', sprintf('Ts = %g;', Ts_io), 'once');
        assert(~strcmp(scr, scr2) || contains(scr, sprintf('Ts = %g;', Ts_io)), ...
            'xp_set_ts_io: nao achei "Ts = ...;" no script de %s', ch.Path);
        ch.Script = scr2;
        info.charts{end+1} = ch.Path;
    end
    fprintf('xp_set_ts_io: %s — laco X-Plane a %.3f s (%.0f Hz) em read_xp/send_xp.\n', mdl, Ts_io, 1/Ts_io);
end
