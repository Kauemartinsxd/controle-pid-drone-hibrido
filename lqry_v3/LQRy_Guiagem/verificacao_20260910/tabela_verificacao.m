function T = tabela_verificacao()
% TABELA_VERIFICACAO  Le todos os verif_*.mat de resultados/ e imprime a tabela-resumo
% (capturas por dist min, capturas SEQUENCIAIS, fim automatico, departure, saturacoes).
vdir = fileparts(mfilename('fullpath')); fs = dir(fullfile(vdir, 'resultados', 'verif_*.mat'));
rows = {};
for k = 1:numel(fs)
    S = load(fullfile(fs(k).folder, fs(k).name)); R = S.R;
    [n_seq, t_hit] = captura_sequencial(S.t, S.Y, S.WPs, S.R_accept);
    fim_auto = R.t_end < 199.9;
    rows(end+1, :) = {R.tag, R.mdl, R.missao, R.eng_tau, sprintf('%d/%d', R.n_hit, R.N_WPs), sprintf('%d/%d', n_seq, R.N_WPs), fim_auto, R.departure, ...
        round(R.t_end, 1), round(100*R.frac_sat_thr), round(100*R.frac_alem15), round(R.phi_max, 1), round(R.alpha(2), 1), round(R.h(1), 1), round(R.h(2), 1), round(R.VT(1), 1), round(R.VT(2), 1), sprintf('%.0f', t_hit(find(~isnan(t_hit), 1, 'last')))};
end
T = cell2table(rows, 'VariableNames', {'tag', 'modelo', 'missao', 'tau_motor', 'cap_distmin', 'cap_sequencial', 'fim_auto', 'departure', 't_fim', 'manete_fora01_pct', 'sup_alem15_pct', 'phi_max', 'alpha_max', 'h_min', 'h_max', 'VT_min', 'VT_max', 't_ultimo_WP'});
disp(T);
end
