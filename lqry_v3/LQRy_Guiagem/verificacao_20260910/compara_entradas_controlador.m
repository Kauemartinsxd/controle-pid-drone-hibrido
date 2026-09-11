function compara_entradas_controlador()
% COMPARA_ENTRADAS_CONTROLADOR  Confere, entrada por entrada, se cada sinal que alimenta o
% controlador (11 saidas da Planta + 4 referencias da guiagem) chega aos MESMOS blocos/portas
% no CL_NL_DH_GUIA_Original (raiz) e no CL_NL_DH_GUIA (via Inports do subsistema Control LQRy).
o = 'CL_NL_DH_GUIA_Original'; n = 'CL_NL_DH_GUIA'; c = [n '/Control LQRy'];
ign = {'ToWorkspace', 'Scope', 'Terminator', 'Mux', 'Gain', 'SubSystem', 'Outport', 'Goto'};
fontes = {};
for p = 1:11, fontes{end+1} = {[o '/Planta'], p}; end
for f = {'From_PsiRefGuia', 'From_DhGuia', 'From_DvGuia', 'From_PhiRefGuia'}, fontes{end+1} = {[o '/' f{1}], 1}; end
nok = 0; ndif = 0;
for q = 1:numel(fontes)
    blk_o = fontes{q}{1}; p = fontes{q}{2}; blk_n = strrep(blk_o, o, n);
    so = sort(dst_via_subsys(blk_o, p, '', ign)); sn = sort(dst_via_subsys(blk_n, p, c, ign));
    if isequal(so, sn), st = 'OK'; nok = nok + 1; else, st = '*** DIFERE ***'; ndif = ndif + 1; end
    fprintf('%-18s porta %2d: %-14s Original -> {%s} | novo -> {%s}\n', strrep(blk_o, [o '/'], ''), p, st, strjoin(so, ', '), strjoin(sn, ', '));
end
fprintf('=> entradas iguais %d | diferentes %d\n', nok, ndif);
end
function D = dst_via_subsys(blk, port, sub, ign)
D = {}; ph = get_param(blk, 'PortHandles'); h = ph.Outport(port); l = get_param(h, 'Line'); if l < 0, return; end
dp = get_param(l, 'DstPortHandle');
for k = 1:numel(dp)
    if dp(k) < 0, continue; end
    b = get_param(dp(k), 'Parent'); bt = get_param(b, 'BlockType');
    if strcmp(bt, 'SubSystem') && ~isempty(sub) && strcmp(b, sub)
        m = get_param(dp(k), 'PortNumber');
        inps = find_system(sub, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'BlockType', 'Inport');
        pn = cellfun(@(x) str2double(get_param(x, 'Port')), inps);
        inp = inps(pn == m);
        if isempty(inp), D{end+1} = sprintf('(Inport %d NAO ENCONTRADO em %s)', m, sub); else, D = [D, dst_via_subsys(inp{1}, 1, '', ign)]; end
    elseif ~any(strcmp(bt, ign))
        D{end+1} = sprintf('%s:%d', strrep(get_param(b, 'Name'), newline, ' '), get_param(dp(k), 'PortNumber'));
    end
end
end
