function dump_sistema(sys)
% DUMP_SISTEMA  Lista blocos (com parametros relevantes) e ligacoes de um (sub)sistema Simulink.
P = containers.Map();
P('Gain') = {'Gain'}; P('Constant') = {'Value'}; P('Sum') = {'Inputs'}; P('Saturate') = {'LowerLimit','UpperLimit'};
P('RateLimiter') = {'RisingSlewLimit','FallingSlewLimit'}; P('StateSpace') = {'A','B','C','D'}; P('TransferFcn') = {'Numerator','Denominator'};
P('Integrator') = {'InitialCondition','LimitOutput','LowerSaturationLimit','UpperSaturationLimit'}; P('Switch') = {'Criteria','Threshold'};
P('Product') = {'Inputs'}; P('Goto') = {'GotoTag'}; P('From') = {'GotoTag'}; P('Inport') = {'Port'}; P('Outport') = {'Port'};
P('S-Function') = {'FunctionName','Parameters'}; P('Fcn') = {'Expr'}; P('Mux') = {'Inputs'}; P('Demux') = {'Outputs'};
P('ToWorkspace') = {'VariableName','SampleTime'}; P('Memory') = {'InitialCondition'}; P('Logic') = {'Operator'}; P('Reference') = {'SourceBlock'};
P('DataTypeConversion') = {'OutDataTypeStr'};
fprintf('\n===== %s =====\n', sys);
blks = find_system(sys, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Type', 'Block');
blks = setdiff(blks, {sys});
for k = 1:numel(blks)
    bt = get_param(blks{k}, 'BlockType'); nm = strrep(get_param(blks{k}, 'Name'), newline, ' '); extra = '';
    if isKey(P, bt)
        pl = P(bt);
        for q = 1:numel(pl)
            try, extra = [extra sprintf(' %s=%s', pl{q}, strrep(get_param(blks{k}, pl{q}), newline, ' '))]; catch, end
        end
    end
    fprintf('  [%s] %s%s\n', bt, nm, extra);
end
ln = find_system(sys, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
for k = 1:numel(ln)
    sb = get_param(ln(k), 'SrcBlockHandle'); if sb < 0, continue; end
    sp = get_param(ln(k), 'SrcPortHandle');
    src = sprintf('%s:%d', strrep(get_param(sb, 'Name'), newline, ' '), get_param(sp, 'PortNumber'));
    dst = ''; dp = get_param(ln(k), 'DstPortHandle');
    for d = dp(:)'
        if d < 0, continue; end
        dst = [dst sprintf(' -> %s:%d', strrep(get_param(get_param(d, 'Parent'), 'Name'), newline, ' '), get_param(d, 'PortNumber'))];
    end
    fprintf('    %s%s\n', src, dst);
end
end
