function D = diff_modelo_mirko(fGuia)
% DIFF_MODELO_MIRKO  Diferenca de CONEXOES entre o CL_NL_DH_18_jun_2026 do Mirko e um
% modelo de guiagem derivado dele (modelo_NL_LQRY_GUIA / modelo_XP_LQRY2_GUIA):
% ligacoes do Mirko que sumiram/foram redirecionadas e ligacoes novas que tocam blocos dele.
if nargin < 1, fGuia = fullfile(fileparts(mfilename('fullpath')), 'modelo_NL_LQRY_GUIA.slx'); end
fM = 'C:\Users\kaue\Documents\Dissertacao_Mestrado\lqry_mirko_atualizado\Nova pasta\CL_NL_DH_18_jun_2026.slx';
mM = 'CL_NL_DH_18_jun_2026'; [~, mG] = fileparts(fGuia);
if bdIsLoaded(mM), bdclose(mM); end; w = warning('off', 'all'); load_system(fM); warning(w);
if ~bdIsLoaded(mG), load_system(fGuia); end
nm = @(m) cellfun(@(b) strrep(get_param(b,'Name'),newline,' '), find_system(m,'SearchDepth',1,'Type','block'), 'UniformOutput', false);
bM = nm(mM); bG = nm(mG); novos = setdiff(bG, bM);
cM = conex(mM); cG = conex(mG);
D.cortadas = setdiff(cM, cG);
novas = setdiff(cG, cM); D.novas_tocando_mirko = {};
for q = 1:numel(novas)
    t = regexp(novas{q}, '^(.*):\d+ -> (.*):\d+$', 'tokens'); a = t{1}{1}; b = t{1}{2};
    if ~(ismember(a, novos) && ismember(b, novos)), D.novas_tocando_mirko{end+1} = novas{q}; end
end
fprintf('LIGACOES DO MIRKO CORTADAS/REDIRECIONADAS (%d):\n', numel(D.cortadas)); fprintf('  %s\n', D.cortadas{:});
fprintf('LIGACOES NOVAS QUE TOCAM BLOCOS DO MIRKO (%d):\n', numel(D.novas_tocando_mirko)); fprintf('  %s\n', D.novas_tocando_mirko{:});
bdclose(mM);
end

function c = conex(m)
lh = find_system(m,'SearchDepth',1,'FindAll','on','Type','line'); c = {};
for q = 1:numel(lh)
    sp = get_param(lh(q),'SrcPortHandle'); if sp < 0, continue; end; dp = get_param(lh(q),'DstPortHandle');
    s = sprintf('%s:%d', strrep(get_param(get_param(sp,'Parent'),'Name'),newline,' '), get_param(sp,'PortNumber'));
    for d = dp(:)'
        if d < 0, continue; end
        c{end+1} = sprintf('%s -> %s:%d', s, strrep(get_param(get_param(d,'Parent'),'Name'),newline,' '), get_param(d,'PortNumber')); %#ok<AGROW>
    end
end
c = sort(c);
end
