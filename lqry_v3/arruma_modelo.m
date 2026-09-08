function arruma_modelo(mdlFile)
% ARRUMA_MODELO  Reorganiza o nivel raiz de um modelo (Simulink.BlockDiagram.arrangeSystem),
% conferindo antes/depois que a lista de conexoes (bloco:porta -> bloco:porta) e' IDENTICA,
% e salva. So' muda posicoes; nenhum bloco, parametro ou linha e' alterado.
%
%   arruma_modelo('C:\...\lqry_v3\modelo_NL_LQRY_GUIA.slx')
[~, mdl] = fileparts(mdlFile);
if bdIsLoaded(mdl), bdclose(mdl); end
load_system(mdlFile);
antes = conexoes(mdl); nb0 = numel(find_system(mdl, 'SearchDepth', 1, 'Type', 'block'));
Simulink.BlockDiagram.arrangeSystem(mdl);
depois = conexoes(mdl); nb1 = numel(find_system(mdl, 'SearchDepth', 1, 'Type', 'block'));
assert(isequal(antes, depois), 'arruma_modelo: a lista de conexoes MUDOU — nao salvo');
assert(nb0 == nb1, 'arruma_modelo: numero de blocos mudou — nao salvo');
save_system(mdl);
bdclose(mdl);
fprintf('%s: %d blocos, %d conexoes conferidas, layout refeito e salvo.\n', mdl, nb1, numel(depois));
end

function c = conexoes(mdl)
% lista ordenada "src_block:porta -> dst_block:porta" de todas as linhas do nivel raiz
lh = find_system(mdl, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
c = {};
for k = 1:numel(lh)
    sp = get_param(lh(k), 'SrcPortHandle'); if sp < 0, continue; end
    dp = get_param(lh(k), 'DstPortHandle');
    src = sprintf('%s:%d', get_param(get_param(sp, 'Parent'), 'Name'), get_param(sp, 'PortNumber'));
    for d = dp(:)'
        if d < 0, continue; end
        c{end+1} = sprintf('%s -> %s:%d', src, get_param(get_param(d, 'Parent'), 'Name'), get_param(d, 'PortNumber')); %#ok<AGROW>
    end
end
c = sort(strrep(c, newline, ' '));
end
