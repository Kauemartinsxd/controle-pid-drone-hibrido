function L = xp_relatorio_taxa(voo, vooFile, Ts_io, pre)
%XP_RELATORIO_TAXA  Taxa de atualizacao REAL do laco X-Plane medida num voo.
%
%   L = xp_relatorio_taxa(voo, vooFile, Ts_io, pre)
%
% voo.t        grade de log do Simulink (tout ou Ts_io)
% voo.t_xplane tempo do X-Plane visto em cada amostra
% pre          struct opcional com .fps_antes .fps_depois .frp .frp2 .rtt
%              (medidos antes/depois do voo)
% Escreve <vooFile>_taxa.txt e <vooFile>_taxa.png (histograma).
%
% "Leituras novas" = amostras em que t_xplane avancou. Quando o log e mais
% rapido que a janela de I/O (ex.: tout a 5 ms com Ts_io 10 ms), metade das
% amostras REPETE a leitura anterior por construcao — o numero que importa
% e a taxa de leituras novas e o espaçamento entre elas.

    if nargin < 4, pre = struct(); end
    g = @(f, d) getfield_or(pre, f, d);
    tx = voo.t_xplane(:); t = voo.t(:);
    n = numel(tx); novo = [true; diff(tx) > 0]; n_novo = nnz(novo);
    dur_xp = tx(end) - tx(1); dur_sim = t(end) - t(1);
    dtx = diff(tx(novo)); dts = sort(dtx);
    q = @(p) 1e3*dts(max(1, min(numel(dts), round(p*numel(dts)))));
    dt_log = median(diff(t));
    L = {};
    L{end+1} = sprintf('Voo: %s', vooFile);
    L{end+1} = sprintf('Engate a t_xp %.1f s apos o load do .acf', tx(1));
    L{end+1} = sprintf('Ts_io pedido: %.3f s (%.0f Hz) | grade de log %.3f s', Ts_io, 1/Ts_io, dt_log);
    L{end+1} = sprintf('Amostras no log: %d em %.1f s de sim', n, dur_sim);
    L{end+1} = sprintf('Leituras com tempo NOVO do X-Plane: %d de %d (%.1f %%; esperado ~%.0f %% pela grade)', ...
        n_novo, n, 100*n_novo/n, 100*min(1, dt_log/Ts_io));
    L{end+1} = sprintf('Taxa efetiva de leitura: %.1f Hz (leituras novas / duracao XP)', (n_novo-1)/max(dur_xp, eps));
    L{end+1} = sprintf('Intervalo entre leituras novas (tempo XP): mediana %.1f ms | p90 %.1f ms | p99 %.1f ms | max %.1f ms', ...
        1e3*median(dtx), q(0.90), q(0.99), 1e3*max(dtx));
    L{end+1} = sprintf('Sincronia: sim %.1f s | X-Plane %.1f s | razao XP/sim %.4f', dur_sim, dur_xp, dur_xp/max(dur_sim, eps));
    if isfield(pre, 'fps_antes')
        L{end+1} = sprintf('X-Plane fps: antes %.0f | depois %.0f (frame %.2f / %.2f ms)', ...
            g('fps_antes', NaN), g('fps_depois', NaN), 1e3*g('frp', NaN), 1e3*g('frp2', NaN));
        L{end+1} = sprintf('Frames de fisica por leitura (aprox.): %.1f', median(dtx)/g('frp', NaN));
    end
    if isfield(pre, 'rtt')
        rtt = sort(pre.rtt(:));
        L{end+1} = sprintf('RTT getDREFs antes do voo: mediana %.2f ms | p95 %.2f ms | max %.2f ms', ...
            1e3*median(rtt), 1e3*rtt(max(1, round(0.95*numel(rtt)))), 1e3*max(rtt));
    end
    fprintf('\n===== TAXA DE ATUALIZACAO MEDIDA =====\n'); fprintf('%s\n', L{:});
    [p, f] = fileparts(vooFile);
    fid = fopen(fullfile(p, [f '_taxa.txt']), 'w'); fprintf(fid, '%s\n', L{:}); fclose(fid);
    fig = figure('Color', 'w', 'Position', [100 100 900 420]); try, fig.Theme = 'light'; catch, end
    histogram(1e3*dtx, 0:1:max(50, ceil(1e3*max(dtx))), 'FaceColor', [0.2 0.45 0.7]);
    xlabel('intervalo entre leituras novas do X-Plane [ms]'); ylabel('amostras'); grid on
    title(sprintf('T_s de I/O pedido: %.0f Hz  |  mediana %.1f ms  |  %.1f Hz efetivo', 1/Ts_io, 1e3*median(dtx), (n_novo-1)/dur_xp));
    exportgraphics(fig, fullfile(p, [f '_taxa.png']), 'Resolution', 150); close(fig);
    fprintf('Relatorio salvo em %s\n', fullfile(p, [f '_taxa.txt']));
end

function v = getfield_or(s, f, d)
    if isstruct(s) && isfield(s, f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end
