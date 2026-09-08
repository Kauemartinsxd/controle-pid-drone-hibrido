function info = lqry_v3_prepara_modelo(mdl, opt)
% LQRY_V3_PREPARA_MODELO  Edicoes EM MEMORIA (o .slx nao e' salvo) que
% transformam o modelo LQRY2 (X-Plane ou NL) no "LQRy v3 de missao":
%   1) satura o phi_ref do psi Hold em +-opt.phi_max_deg (Tabela 4 do artigo:
%      |phi| <= 30 deg) — bloco Sat_phiref_v3 entre Sum17 e Switch2;
%   2) anti-windup por clamping nos 6 integradores (limite = alcance util do
%      atuador / da referencia dividido pelo ganho integral da planta i);
%   3) clamp do theta_ref (bloco Sat_thetaref_envelope ja existente; usa a
%      variavel XP_clamp_lqry do workspace);
%   4) neutraliza o pulso de V_T embutido (Steps 'ref vel' em 10/20 s -> 1e9);
%   5) (opcional) fim de missao automatico: 5 s apos entrar no circulo do
%      ultimo WP (mesma logica dos modelos GUIA do PID);
%   6) (opcional, so' X-Plane) taxa do laco read_xp/send_xp = opt.Ts_io.
%
% opt (struct, tudo opcional): phi_max_deg (25), antiwindup (1), fim_auto (1),
%   Ts_io ([] = nao mexe), guia_psi_off (0), i (planta p/ dimensionar os clamps;
%   default = base 'i'), ganhos = struct com GintLong..Gintlat_psi (default = base)
% DEFAULTS = ESTRUTURA DO MIRKO INTACTA ("so' ganhos", decisao do Kaue 2026-09-03):
%   phi_max_deg [] (sem saturacao de phi_ref), antiwindup 0, ref_prop 0, theta_e_ff 0,
%   ic_bumpless 1 (condicao inicial dos integradores = engate bumpless; harness, como no
%   XP_missao_lqry2), vt_pulse_off 1 e fim_auto 1 (harness de missao, fora do controlador).
% As opcoes "estruturais" (phi_max_deg, ref_prop, theta_e_ff, antiwindup) ficam como
% EXPERIMENTO opt-in.
if nargin < 2, opt = struct(); end
g = @(f, d) getfield_or(opt, f, d);
info = struct();
% ---------- 1) saturacao do phi_ref (OPT-IN: altera a estrutura) ----------
phi_max_deg = g('phi_max_deg', []);
if ~isempty(phi_max_deg)
    phi_max = deg2rad(phi_max_deg);
    if ~getSimulinkBlockHandle([mdl '/Sat_phiref_v3']) || getSimulinkBlockHandle([mdl '/Sat_phiref_v3']) < 0
        delete_line(mdl, 'Sum17/1', 'Switch2/3');
        pos = get_param([mdl '/Switch2'], 'Position');
        add_block('simulink/Discontinuities/Saturation', [mdl '/Sat_phiref_v3'], ...
            'UpperLimit', num2str(phi_max), 'LowerLimit', num2str(-phi_max), ...
            'Position', [pos(1)-90 pos(2)+40 pos(1)-60 pos(2)+60]);
        add_line(mdl, 'Sum17/1', 'Sat_phiref_v3/1', 'autorouting', 'on');
        add_line(mdl, 'Sat_phiref_v3/1', 'Switch2/3', 'autorouting', 'on');
    else
        set_param([mdl '/Sat_phiref_v3'], 'UpperLimit', num2str(phi_max), 'LowerLimit', num2str(-phi_max));
    end
    info.phi_max_deg = phi_max_deg;
end
% ---------- 1b) condicao inicial dos integradores (engate bumpless; harness) ----------
% Na estrutura do Mirko o Alt Hold devolve theta_ref = GsA*dx + GiA*xi_H, que vale 0 no
% trim; para theta_ref(0) = theta de engate, xi_H(0) = theta0/GiA (com dx = 0). Os demais
% integradores partem de 0 (desvios nulos no engate). E' o que o XP_missao_lqry2 ja fazia
% (XP_IC_int_alt = (theta0 - GsA*x)/GiA), so' que aqui em valor numerico, sem depender
% de variaveis do workspace nos blocos.
if g('ic_bumpless', 1)
    i = g('i', evalin('base', 'i'));
    G = g('ganhos', []);
    if isempty(G)
        for v = {'GintLong','GintLong_Alt','GintLong_speed','Gintlat','Gintlat_psi'}
            G.(v{1}) = evalin('base', v{1});
        end
    end
    theta0 = g('theta0', evalin('base', 'Plantas(i).Xe(8)'));
    xi_alt0 = theta0/double(G.GintLong_Alt{i});
    set_param([mdl '/Integrator2'], 'InitialCondition', num2str(xi_alt0, '%.10g'));
    for b = {'Integrator', 'Integrator1', 'Integrator3', 'Integrator4', 'Integrator5'}
        set_param([mdl '/' b{1}], 'InitialCondition', '0');
    end
    info.ic = struct('theta0_deg', theta0*180/pi, 'xi_alt0', xi_alt0);
end
% ---------- 2) anti-windup (clamping) (OPT-IN) ----------
if g('antiwindup', 0)
    i = g('i', evalin('base', 'i'));
    G = g('ganhos', []);
    if isempty(G)
        for v = {'GintLong','GintLong_Alt','GintLong_speed','Gintlat','Gintlat_psi'}
            G.(v{1}) = evalin('base', v{1});
        end
    end
    GiT = abs(double(G.GintLong{i})); GiA = abs(double(G.GintLong_Alt{i})); GiS = abs(double(G.GintLong_speed{i}));
    Gil = abs(double(G.Gintlat{i}));  Gip = abs(double(G.Gintlat_psi{i}));
    lims = struct( ...
        'Integrator',  25/max(GiT, eps), ...                 % theta Hold: |Gi*xi| <= 25 deg de profundor
        'Integrator2', deg2rad(20)/max(GiA, eps), ...        % Alt Hold:   <= 20 deg de theta_ref
        'Integrator3', 100/max(GiS, eps), ...                % Vel Hold:   <= 100 % de manete
        'Integrator4', deg2rad(15)/max(max(Gil(:,1)), eps), ... % phi Hold (int phi): <= 15 deg de superficie
        'Integrator1', deg2rad(15)/max(max(Gil(:,2)), eps), ... % phi Hold (int beta)
        'Integrator5', deg2rad(25)/max(Gip, eps));           % psi Hold:   <= 25 deg de phi_ref
    fn = fieldnames(lims);
    for k = 1:numel(fn)
        set_param([mdl '/' fn{k}], 'LimitOutput', 'on', ...
            'UpperSaturationLimit', num2str(lims.(fn{k})), 'LowerSaturationLimit', num2str(-lims.(fn{k})));
    end
    info.aw = lims;
end
% ---------- 3b) referencia no termo proporcional ("b = 1"): o estado rastreado entra como ERRO ----------
% Estrutura do Mirko: u = Gs*(x - x_trim) + Gi*int(y - r) — a referencia so' entra pelo
% integrador, o termo proporcional regula o estado para o TRIM (theta_e, H = 0, psi = 0...).
% Para referencias grandes (curvas de 90 deg, +-20 m) o integrador teria de vencer o termo
% proporcional (e o anti-windup o impede). Aqui o estado rastreado passa a ser (y - r):
%   theta Hold: Mux2/4 <- Sum1 (theta - theta_ref), TrimConstTheta(4) = 0
%   Alt   Hold: Mux5/5 <- Sum8 (H - H_ref)          (TrimConstAlt(5) ja e' 0)
%   Vel   Hold: Mux7/1 <- Sum10 (VT - VT_ref),      TrimConstSpeed(1) = 0
%   phi   Hold: Mux9/4 <- Sum2 (phi - phi_ref)
%   psi   Hold: Mux11/5 <- Sum18 (psi - psi_ref)
% Os ganhos v3 sao sintetizados com esta mesma estrutura (lqry_v3_projeto 'ref_prop').
if g('ref_prop', 0)
    troca = {'Planta/4', 'Mux2/4',  'Sum1/1';
             'Planta/10','Mux5/5',  'Sum8/1';
             'Planta/1', 'Mux7/1',  'Sum10/1';
             'Planta/8', 'Mux9/4',  'Sum2/1';
             'Planta/9', 'Mux11/5', 'Sum18/1'};
    for k = 1:size(troca, 1)
        try, delete_line(mdl, troca{k,1}, troca{k,2}); catch, end
        add_line(mdl, troca{k,3}, troca{k,2}, 'autorouting', 'on');
    end
    set_param([mdl '/TrimConstTheta'], 'Value', '[Plantas(i).Xe(1); atan(Plantas(i).Xe(3)/Plantas(i).Xe(1)); 0; 0; 0]');
    set_param([mdl '/TrimConstSpeed'], 'Value', '[0; atan(Plantas(i).Xe(3)/Plantas(i).Xe(1)); 0; Plantas(i).Xe(8)]');
    info.ref_prop = true;
end
% ---------- 3c) feed-forward do theta de trim no Alt Hold ----------
% Sum7 (theta_ref do Alt Hold) vale 0 no trim, mas Sum1 compara com theta ABSOLUTO: sem o
% termo theta_e o integrador de H precisa "descobrir" o trim (com GiA pequeno, afunda
% dezenas de metros). theta_ref = theta_e_ff + Sum7 (theta_e_ff no workspace; default =
% Plantas(i).Xe(8); no X-Plane usar o pitch de trim medido do gemeo).
if g('theta_e_ff', 0)
    if ~evalin('base', 'exist(''theta_e_ff'',''var'')'), evalin('base', 'theta_e_ff = Plantas(i).Xe(8);'); end
    dst = 'Switch1/3';
    delete_line(mdl, 'Sum7/1', dst);
    pos = get_param([mdl '/Sum7'], 'Position');
    add_block('simulink/Sources/Constant', [mdl '/C_theta_e_ff'], 'Value', 'theta_e_ff', 'Position', [pos(1) pos(2)+60 pos(1)+40 pos(2)+80]);
    add_block('simulink/Math Operations/Sum', [mdl '/Sum_theta_e_ff'], 'Inputs', '++', 'Position', [pos(3)+30 pos(2) pos(3)+50 pos(4)]);
    add_line(mdl, 'Sum7/1', 'Sum_theta_e_ff/1', 'autorouting', 'on');
    add_line(mdl, 'C_theta_e_ff/1', 'Sum_theta_e_ff/2', 'autorouting', 'on');
    add_line(mdl, 'Sum_theta_e_ff/1', dst, 'autorouting', 'on');
    try   % a sonda de theta_ref passa a ver o valor absoluto
        delete_line(mdl, 'Sum7/1', 'TW_probe_thetaref/1');
        add_line(mdl, 'Sum_theta_e_ff/1', 'TW_probe_thetaref/1', 'autorouting', 'on');
    catch
    end
    info.theta_e_ff = true;
end
% ---------- 4) pulso de V_T embutido (Steps 'ref vel' 10 s / 20 s) ----------
n_off = 0;
if g('vt_pulse_off', 1)
    stp = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Step');
    for k = 1:numel(stp)
        nm = strtrim(get_param(stp{k}, 'Name'));
        if strcmp(nm, 'ref vel')
            tm = str2double(get_param(stp{k}, 'Time'));
            if tm == 10 || tm == 20, set_param(stp{k}, 'Time', '1e9'); n_off = n_off + 1; end
        end
    end
end
info.steps_vt_neutralizados = n_off;
% ---------- guiagem de proa on/off ----------
if g('guia_psi_off', 0)
    ph_g = get_param([mdl '/Goto_PsiRefGuia'], 'PortHandles'); l_g = get_param(ph_g.Inport(1), 'Line');
    src_g = get_param(l_g, 'SrcPortHandle'); delete_line(l_g);
    pos_g = get_param([mdl '/Goto_PsiRefGuia'], 'Position');
    add_block('simulink/Sources/Constant', [mdl '/C_psi_guia_off'], 'Value', '0', 'Position', pos_g + [-120 0 -120 0]);
    add_line(mdl, 'C_psi_guia_off/1', 'Goto_PsiRefGuia/1');
    add_block('simulink/Sinks/Terminator', [mdl '/T_psi_guia_off'], 'Position', pos_g + [0 60 0 60]);
    add_line(mdl, src_g, get_param([mdl '/T_psi_guia_off'], 'PortHandles').Inport(1));
end
% ---------- 5) fim de missao automatico ----------
if g('fim_auto', 1) && (~getSimulinkBlockHandle([mdl '/Stop_fim_missao']) || getSimulinkBlockHandle([mdl '/Stop_fim_missao']) < 0)
    x = 1700; y = 900;
    add_block('simulink/Signal Routing/From', [mdl '/From_fim_XN'], 'GotoTag', 'XN', 'Position', [x y x+60 y+20]);
    add_block('simulink/Signal Routing/From', [mdl '/From_fim_XE'], 'GotoTag', 'XE', 'Position', [x y+40 x+60 y+60]);
    add_block('simulink/Signal Routing/Mux', [mdl '/Mux_fim'], 'Inputs', '3', 'Position', [x+100 y-10 x+105 y+70]);
    add_block('simulink/User-Defined Functions/Fcn', [mdl '/Cond_fim_missao'], ...
        'Expr', '(u(1)>=N_WPs)&&(sqrt((u(2)-WPfim_N)^2+(u(3)-WPfim_E)^2)<=R_accept)', 'Position', [x+140 y+20 x+260 y+50]);
    add_block('simulink/Logic and Bit Operations/Logical Operator', [mdl '/OR_latch_fim'], 'Operator', 'OR', 'Position', [x+300 y+20 x+330 y+50]);
    add_block('simulink/Discrete/Memory', [mdl '/Mem_latch_fim'], 'Position', [x+300 y+80 x+330 y+100]);
    add_block('simulink/Signal Attributes/Data Type Conversion', [mdl '/Bool2Dbl_fim'], 'OutDataTypeStr', 'double', 'Position', [x+370 y+20 x+410 y+50]);
    add_block('simulink/Continuous/Integrator', [mdl '/Int_folga_fim'], 'Position', [x+450 y+20 x+480 y+50]);
    add_block('simulink/Logic and Bit Operations/Compare To Constant', [mdl '/Folga_5s'], 'relop', '>=', 'const', '5', 'Position', [x+520 y+20 x+560 y+50]);
    add_block('simulink/Sinks/Stop Simulation', [mdl '/Stop_fim_missao'], 'Position', [x+600 y+20 x+630 y+50]);
    % wp_idx_mon: descobre a porta do chart seguindo a linha que ja alimenta Log_wp_idx_log
    ph_w = get_param([mdl '/Log_wp_idx_log'], 'PortHandles'); l_w = get_param(ph_w.Inport(1), 'Line');
    k_w = get_param(get_param(l_w, 'SrcPortHandle'), 'PortNumber');
    add_line(mdl, sprintf('Guidance_Star/%d', k_w), 'Mux_fim/1', 'autorouting', 'on');
    add_line(mdl, 'From_fim_XN/1', 'Mux_fim/2', 'autorouting', 'on');
    add_line(mdl, 'From_fim_XE/1', 'Mux_fim/3', 'autorouting', 'on');
    add_line(mdl, 'Mux_fim/1', 'Cond_fim_missao/1', 'autorouting', 'on');
    add_line(mdl, 'Cond_fim_missao/1', 'OR_latch_fim/1', 'autorouting', 'on');
    add_line(mdl, 'OR_latch_fim/1', 'Mem_latch_fim/1', 'autorouting', 'on');
    add_line(mdl, 'Mem_latch_fim/1', 'OR_latch_fim/2', 'autorouting', 'on');
    add_line(mdl, 'OR_latch_fim/1', 'Bool2Dbl_fim/1', 'autorouting', 'on');
    add_line(mdl, 'Bool2Dbl_fim/1', 'Int_folga_fim/1', 'autorouting', 'on');
    add_line(mdl, 'Int_folga_fim/1', 'Folga_5s/1', 'autorouting', 'on');
    add_line(mdl, 'Folga_5s/1', 'Stop_fim_missao/1', 'autorouting', 'on');
    info.fim_auto = true;
end
% ---------- 6) taxa do laco X-Plane ----------
Ts_io = g('Ts_io', []);
if ~isempty(Ts_io)
    rt = sfroot;
    for nm = {'read_xp', 'send_xp'}
        ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/Planta/' nm{1}]);
        assert(~isempty(ch), 'lqry_v3_prepara_modelo: chart %s nao encontrado', nm{1});
        ch.SampleTime = num2str(Ts_io);
        scr = ch.Script;
        assert(contains(scr, 'Ts = 0.05;'), 'lqry_v3_prepara_modelo: "Ts = 0.05;" nao achado em %s', nm{1});
        ch.Script = strrep(scr, 'Ts = 0.05;', sprintf('Ts = %g;', Ts_io));
    end
    info.Ts_io = Ts_io;
end
end

function v = getfield_or(s, f, d)
if isfield(s, f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end
