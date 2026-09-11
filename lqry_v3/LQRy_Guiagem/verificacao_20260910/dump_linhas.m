function dump_linhas(sys)
% DUMP_LINHAS  Lista as ligacoes (SrcBlock:porta -> DstBlock:porta) de um (sub)sistema via get_param(sys,'Lines').
fprintf('\n----- ligacoes de %s -----\n', sys);
L = get_param(sys, 'Lines');
for k = 1:numel(L), rec(L(k), ''); end
end
function rec(l, src)
if isempty(src) && ~isempty(l.SrcBlock), src = sprintf('%s:%s', nm(l.SrcBlock), num2str(l.SrcPort)); end
if ~isempty(l.DstBlock)
    db = l.DstBlock; dp = l.DstPort;
    if iscell(db), for q = 1:numel(db), fprintf('    %s -> %s:%s\n', src, nm(db{q}), num2str(dp{q})); end
    else, for q = 1:numel(db), fprintf('    %s -> %s:%s\n', src, nm(db(q)), num2str(dp(q))); end, end
end
if isfield(l, 'Branch') && ~isempty(l.Branch), for q = 1:numel(l.Branch), rec(l.Branch(q), src); end, end
end
function s = nm(b)
if isnumeric(b), s = strrep(get_param(b, 'Name'), newline, ' '); else, s = strrep(b, newline, ' '); end
end
