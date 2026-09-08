function fn_out = VR_merge_campanhas(padrao, nome_saida, lista_esperada)
%VR_MERGE_CAMPANHAS Junta blocos VR_replay_*.mat numa campanha unica.
%
%   fn = VR_merge_campanhas('VR_replay_20260908_*.mat', 'VR_replay_GEMEO_FIX_MERGED.mat', [5:14 19:22])
%
% Carrega todos os arquivos voos/<padrao> em ordem cronologica, mantem a
% ULTIMA repeticao de cada segmento (iseg) e salva voos/<nome_saida> com as
% mesmas variaveis do VR_replay_xp (R, sinais, esc, VR_curso_real, ...) para
% o VR_plot_comp (VR_arq_replay = fn). Avisa se faltar algum segmento de
% lista_esperada. Campanhas de 2026-09-08 em diante gravam 'esc'
% (escala de curso) — blocos antigos sem 'esc' sao aceitos (esc = 1).
    xpDir = fileparts(mfilename('fullpath')); voosDir = fullfile(xpDir, 'voos');
    d = dir(fullfile(voosDir, padrao));
    assert(~isempty(d), 'VR_merge_campanhas: nenhum arquivo %s em voos/', padrao);
    [~, ord] = sort([d.datenum]); d = d(ord);
    R = []; meta = struct();
    for i = 1:numel(d)
        C = load(fullfile(voosDir, d(i).name));
        if ~isfield(C, 'R') || isempty(C.R), continue; end
        for k = 1:numel(C.R)
            r = C.R(k);
            if isempty(R), R = r; continue; end
            ja = find([R.iseg] == r.iseg, 1);
            if isempty(ja), R(end+1) = r; else, R(ja) = r; end %#ok<AGROW>
        end
        for f = {'sinais','VR_ganho','esc','VR_curso_real','lims','MSL0','TRIM_T'}
            if isfield(C, f{1}), meta.(f{1}) = C.(f{1}); end
        end
        fprintf('  %s: %d segmentos (%s)\n', d(i).name, numel(C.R), mat2str([C.R.iseg]));
    end
    [~, ord] = sort([R.iseg]); R = R(ord);
    if nargin >= 3 && ~isempty(lista_esperada)
        falta = setdiff(lista_esperada, [R.iseg]);
        if ~isempty(falta), fprintf(2, 'VR_merge_campanhas: FALTAM os segmentos %s\n', mat2str(falta)); end
    end
    meta.R = R; meta.VR_lista = [R.iseg];
    fn_out = fullfile(voosDir, nome_saida);
    save(fn_out, '-struct', 'meta');
    fprintf('Campanha unificada: %s (%d segmentos: %s)\n', fn_out, numel(R), mat2str([R.iseg]));
end
