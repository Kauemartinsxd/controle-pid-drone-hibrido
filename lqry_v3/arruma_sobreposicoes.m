function n_mov = arruma_sobreposicoes(mdlFile, varargin)
% ARRUMA_SOBREPOSICOES  Resolve blocos sobrepostos no nivel raiz movendo SO' os blocos
% "inseridos" (guiagem, sondas, saturacoes) para o espaco livre mais proximo, e re-roteia
% as linhas deles. Confere que a lista de conexoes nao muda. Nao mexe no layout do Mirko.
%
%   arruma_sobreposicoes('...\modelo_NL_LQRY_GUIA.slx')            % salva
%   arruma_sobreposicoes(f, 'salvar', false, 'margem', 20)
p = inputParser; p.addParameter('salvar', true); p.addParameter('margem', 18); p.addParameter('passo', 20);
p.addParameter('espaco', 0);      % >0: blocos INSERIDOS a menos de 'espaco' px de um vizinho tambem sao afastados
p.parse(varargin{:}); o = p.Results;
[~, mdl] = fileparts(mdlFile);
if ~bdIsLoaded(mdl), load_system(mdlFile); end
antes = conexoes(mdl);
inseridos = {'Add_', 'From_', 'Goto_', 'TW_probe', 'Sat_thetaref', 'C_', 'Log_', 'Guidance_Star', 'Clock', 'Mux_fim', 'Cond_fim'};
n_mov = 0;
for it = 1:80
    [b, P] = blocos(mdl);
    ins = cellfun(@(x) eh_inserido(get_param(x, 'Name'), inseridos), b);
    par = [];
    for i = 1:numel(b), for j = i+1:numel(b)
        m = 0; if o.espaco > 0 && (ins(i) || ins(j)), m = o.espaco; end
        if sobrepoe(P(i,:), P(j,:), m), par = [i j]; break; end %#ok<AGROW>
    end, if ~isempty(par), break; end, end
    if isempty(par), break; end
    i = par(1); j = par(2);
    if ins(j) && ~ins(i), k = j; else, k = i; end          % move o inserido (ou o 1o, se ambos)
    pos = P(k,:);
    outros = P; outros(k,:) = [];
    melhor = []; dmin = inf; mg = max(o.margem, o.espaco);
    for dx = -12*o.passo:o.passo:12*o.passo
        for dy = -8*o.passo:o.passo:8*o.passo
            cand = pos + [dx dy dx dy];
            livre = true;
            for q = 1:size(outros,1), if sobrepoe(cand, outros(q,:), mg), livre = false; break; end, end
            if livre && hypot(dx, dy) < dmin, dmin = hypot(dx, dy); melhor = cand; end
        end
    end
    if isempty(melhor), warning('sem espaco livre p/ %s', get_param(b{k},'Name')); break; end
    set_param(b{k}, 'Position', melhor);
    reroteia(mdl, b{k});
    fprintf('  movido %-26s %s -> %s  (sobrepunha %s)\n', strrep(get_param(b{k},'Name'),newline,' '), mat2str(pos), mat2str(melhor), strrep(get_param(b{par(par~=k)},'Name'),newline,' '));
    n_mov = n_mov + 1;
end
depois = conexoes(mdl);
assert(isequal(antes, depois), 'arruma_sobreposicoes: a lista de conexoes MUDOU — nao salvo');
[b, P] = blocos(mdl); n = 0;
for i = 1:numel(b), for j = i+1:numel(b), if sobrepoe(P(i,:), P(j,:), 0), n = n + 1; end, end, end
fprintf('%s: %d blocos movidos, %d sobreposicoes restantes, %d conexoes conferidas.\n', mdl, n_mov, n, numel(depois));
if o.salvar, save_system(mdl); fprintf('salvo.\n'); end
end

function tf = sobrepoe(a, b, m)
tf = a(1) - m < b(3) && b(1) - m < a(3) && a(2) - m < b(4) && b(2) - m < a(4);
end

function tf = eh_inserido(nome, lista)
tf = any(cellfun(@(s) startsWith(nome, s), lista));
end

function [b, P] = blocos(mdl)
b = find_system(mdl, 'SearchDepth', 1, 'Type', 'block');
b = b(~strcmp(b, mdl));
P = cell2mat(get_param(b, 'Position'));
end

function reroteia(mdl, blk)
% apaga e recria (com autorouting) as linhas ligadas as portas do bloco
ph = get_param(blk, 'PortHandles');
% entradas: linha chega de uma fonte (pode ser ramo de uma linha em arvore)
for h = ph.Inport(:)'
    l = get_param(h, 'Line'); if l < 0, continue; end
    src = get_param(l, 'SrcPortHandle'); if src < 0, continue; end
    delete_line(l);
    add_line(mdl, src, h, 'autorouting', 'smart');
end
% saidas: linha (possivelmente em arvore) para varios destinos
for h = ph.Outport(:)'
    l = get_param(h, 'Line'); if l < 0, continue; end
    dst = get_param(l, 'DstPortHandle'); dst = dst(dst > 0);
    delete_line(l);
    for d = dst(:)', add_line(mdl, h, d, 'autorouting', 'smart'); end
end
end

function c = conexoes(mdl)
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
