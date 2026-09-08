function build_pacote_mirko(varargin)
% BUILD_PACOTE_MIRKO  Gera pacote_mirko_guiagem_NL/CL_NL_DH_GUIA.slx a partir do
% modelo_NL_LQRY_GUIA.slx: o LQRy do Mirko (estrutura intacta) com a guiagem LOS ligada
% DIRETO nas malhas. Remove os Steps de doublet do artigo, os Manual Switches de
% referencia, os somadores Add_*_guia, as sondas TW_probe_* e a saturacao de theta_ref;
% restaura Switch1 -> Sum1 e os indices {i} dos ganhos laterais; renomeia as variaveis
% de harness (U_trim, h0, xi_alt0); adiciona o fim de missao automatico.
%
%   build_pacote_mirko            % gera o .slx do pacote e confere portas soltas
p = inputParser; p.addParameter('forca', true); p.parse(varargin{:}); o = p.Results; %#ok<NASGU>
here = fileparts(mfilename('fullpath'));
src  = fullfile(here, 'modelo_NL_LQRY_GUIA.slx');
pdir = fullfile(here, 'pacote_mirko_guiagem_NL');
mdl  = 'CL_NL_DH_GUIA'; dst = fullfile(pdir, [mdl '.slx']);
for m = {mdl, 'modelo_NL_LQRY_GUIA'}, if bdIsLoaded(m{1}), bdclose(m{1}); end, end
copyfile(src, dst, 'f');
load_system(dst);
cnt = struct('apagados', 0, 'linhas', 0);

%% 1) chart de guiagem: v_ref ABSOLUTO (era dv_ref relativo) + nomes claros; mesma ordem de portas
ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', [mdl '/Guidance_Star']);
ch.Script = sprintf(['function [psi_ref, h_ref_rel, v_ref, wp_idx, dist, phi_ref] = Guidance_Star(xN, xE, psi_abs, psi_rel, WPs, R_accept, h_ref0, VT_ref0, K_bank_guia, phi_max_guia)\n' ...
 '%% Guiagem LOS por waypoints (algoritmo do PIPER-1-6 do Julio, adaptado ao DH).\n' ...
 '%% Entradas: posicao xN/xE [m], proa absoluta psi_abs e relativa ao engate psi_rel [rad].\n' ...
 '%% WPs = [N E h V] (uma linha por waypoint); R_accept = raio de captura [m].\n' ...
 '%% Saidas (entram DIRETO nos holds do LQRy):\n' ...
 '%%   psi_ref   [rad] proa de referencia no mesmo referencial (relativo ao engate) que o psi Hold realimenta\n' ...
 '%%   h_ref_rel [m]   altitude do WP - h_ref0 (o Alt Hold trabalha com H relativa ao engate)\n' ...
 '%%   v_ref     [m/s] velocidade do WP (absoluta) -> Vel Hold\n' ...
 '%%   phi_ref   [rad] bank-to-turn, so'' usado no modo phi Hold (phi_psi = 1); zero apos o ultimo WP\n' ...
 '%%   wp_idx, dist: monitor (WP alvo e distancia ate ele)\n' ...
 'persistent k hold_fim\n' ...
 'if isempty(k), k = 1; hold_fim = 0; end\n' ...
 'n = size(WPs, 1); if k > n, k = n; end\n' ...
 'dist = sqrt((WPs(k,1) - xN)^2 + (WPs(k,2) - xE)^2);\n' ...
 'if dist <= R_accept\n' ...
 '    if k < n, k = k + 1; else, hold_fim = 1; end\n' ...
 'end\n' ...
 'psi_los = atan2(WPs(k,2) - xE, WPs(k,1) - xN);\n' ...
 'dpsi = mod(psi_los - psi_abs + pi, 2*pi) - pi;\n' ...
 'psi_ref = psi_rel + dpsi;\n' ...
 'if hold_fim, phi_ref = 0; else, phi_ref = min(max(K_bank_guia*dpsi, -phi_max_guia), phi_max_guia); end\n' ...
 'h_ref_rel = WPs(k,3) - h_ref0;\n' ...
 'v_ref = WPs(k,4) + 0*VT_ref0;\n' ...
 'wp_idx = k;\n' ...
 'end\n']);
% religa as 6 saidas por numero de porta (a troca de nomes pode soltar as linhas)
dests = {'Goto_PsiRefGuia', 'Goto_DhGuia', 'Goto_DvGuia', 'Log_wp_idx_log', 'Log_dist_log', 'Goto_PhiRefGuia'};
phc = get_param([mdl '/Guidance_Star'], 'PortHandles');
for q = 1:6
    l = get_param(phc.Outport(q), 'Line'); if l > 0, delete_line(l); end
    add_line(mdl, sprintf('Guidance_Star/%d', q), [dests{q} '/1'], 'autorouting', 'on');
end

%% 2) apaga referencias de doublet, switches manuais, somadores, sondas, saturacao
apagar = {};
stp = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Step'); apagar = [apagar; stp(:)];
apagar = [apagar; strcat([mdl '/'], {'Sum4'; 'Sum5'; 'Sum14'; 'Sum19'; 'Sum11'; 'Sum15'; 'Sum16'; 'Constant1'; ...
    'Manual Switch'; 'Manual Switch2'; 'Manual Switch3'; 'Manual Switch4'; 'Manual Switch7'; ...
    sprintf('Degrees to%sRadians', newline); sprintf('Degrees to%sRadians1', newline); sprintf('Degrees to%sRadians2', newline); sprintf('Degrees to%sRadians5', newline); ...
    'Add_h_guia'; 'Add_phi_guia'; 'Add_psi_guia'; 'Add_v_guia'; ...
    'TW_probe_phiref'; 'TW_probe_refH'; 'TW_probe_refVT'; 'TW_probe_thetaref'; 'Sat_thetaref_envelope'})];
for k = 1:numel(apagar)
    b = apagar{k}; if getSimulinkBlockHandle(b) < 0, warning('nao achei %s', b); continue; end
    lh = get_param(b, 'LineHandles');
    for f = {'Inport', 'Outport'}
        for h = lh.(f{1})(:)', if h > 0, delete_line(h); cnt.linhas = cnt.linhas + 1; end, end
    end
    delete_block(b); cnt.apagados = cnt.apagados + 1;
end

%% 3) religa: guiagem direto nos holds + logs + modos theta Hold / beta
pos = @(b) get_param([mdl '/' b], 'Position');
add_block('simulink/Sources/Constant', [mdl '/theta_hold_ref'], 'Value', 'Plantas(i).Xe(8)', 'Position', pos('Switch1') + [-140 -60 -60 -70]);
add_block('simulink/Sources/Constant', [mdl '/beta_ref'], 'Value', '0', 'Position', pos('Sum12') + [-120 -10 -80 10]);
pr = pos('Mux3');  add_block('simulink/Math Operations/Gain', [mdl '/R2D_theta_ref'], 'Gain', '180/pi', 'Position', [pr(1)-90 pr(2)-40 pr(1)-50 pr(2)-20]);
pr = pos('Mux12'); add_block('simulink/Math Operations/Gain', [mdl '/R2D_psi_ref'],   'Gain', '180/pi', 'Position', [pr(1)-90 pr(4)+10 pr(1)-50 pr(4)+30]);
pr = pos('Mux10'); add_block('simulink/Math Operations/Gain', [mdl '/R2D_phi_ref'],   'Gain', '180/pi', 'Position', [pr(1)-90 pr(2)-40 pr(1)-50 pr(2)-20]);
lig = {'From_PsiRefGuia/1', 'Sum18/1';          % psi_ref -> psi Hold
       'From_DhGuia/1',     'Sum8/1';           % h_ref_rel -> Alt Hold
       'From_DhGuia/1',     'Sum/2';            % log H_NL: 600 + h_ref_rel
       'From_DvGuia/1',     'Sum10/1';          % v_ref -> Vel Hold
       'From_DvGuia/1',     'Mux6/1';           % log VT_NL: [ref VT]
       'From_PhiRefGuia/1', 'Switch2/1';        % phi_ref -> modo phi Hold
       'theta_hold_ref/1',  'Switch1/1';        % modo theta Hold: segura o theta de trim
       'Switch1/1',         'Sum1/1';           % theta_ref -> theta Hold (sem saturacao)
       'beta_ref/1',        'Sum12/1';          % beta_ref = 0
       'Switch1/1',         'R2D_theta_ref/1';  'R2D_theta_ref/1', 'Mux3/1';    % log theta_NL = [ref theta]
       'From_PsiRefGuia/1', 'R2D_psi_ref/1';    'R2D_psi_ref/1',   'Mux12/2';   % log psi_NL = [psi ref]
       'Switch2/1',         'R2D_phi_ref/1';    'R2D_phi_ref/1',   'Mux10/1'};  % log phi_NL = [ref phi]
for k = 1:size(lig, 1), add_line(mdl, lig{k,1}, lig{k,2}, 'autorouting', 'on'); end

%% 4) indices dos ganhos laterais de volta a {i}; variaveis de harness com nomes do pacote
set_param([mdl '/int'],     'Gain', 'Gintlat{i}');      set_param([mdl '/int1'],    'Gain', 'Gintlat_psi{i}');
set_param([mdl '/states'],  'Gain', 'GstateLat_psi{i}'); set_param([mdl '/states1'], 'Gain', 'GstateLat{i}');
set_param([mdl '/Integrator'],  'InitialCondition', '0');
set_param([mdl '/Integrator2'], 'InitialCondition', 'xi_alt0');   % = theta0/GintLong_Alt{i}: engate bumpless
set_param([mdl '/Integrator3'], 'InitialCondition', '0');
set_param([mdl '/Planta/U1_trim'],  'Value', 'U_trim');
set_param([mdl '/Planta/C_h_ref0'], 'Value', 'h0');
set_param([mdl '/Planta/Eng_Throttle'],   'InitialCondition', 'U_trim(1)');
set_param([mdl '/Planta/Servo_Elevator'], 'InitialCondition', 'U_trim(2)');
set_param([mdl '/Planta/Servo_Aileron'],  'InitialCondition', 'U_trim(3)');
set_param([mdl '/Planta/Servo_Rudder'],   'InitialCondition', 'U_trim(4)');

%% 5) fim de missao automatico (5 s apos entrar no circulo do ultimo WP)
addpath(here);
lqry_v3_prepara_modelo(mdl, struct('ic_bumpless', 0, 'fim_auto', 1, 'vt_pulse_off', 0, 'antiwindup', 0, 'ref_prop', 0, 'theta_e_ff', 0));

%% 6) portas soltas? anotacao; salva
ports = find_system(mdl, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'port');
soltas = {};
for h = ports(:)'
    if get_param(h, 'Line') < 0 && strcmp(get_param(h, 'PortType'), 'inport'), soltas{end+1} = [strrep(get_param(get_param(h,'Parent'),'Name'),newline,' ') ':' num2str(get_param(h,'PortNumber'))]; end %#ok<AGROW>
end
if ~isempty(soltas), fprintf('ENTRADAS SOLTAS: %s\n', strjoin(soltas, ', ')); else, fprintf('nenhuma entrada solta.\n'); end
add_block('built-in/Note', [mdl '/Nota_pacote'], 'Position', [-2300 -120 -2280 -100]);
set_param([mdl '/Nota_pacote'], 'Text', sprintf(['CL_NL_DH_GUIA: LQRy (Mirko, CL_NL_DH_18_jun_2026, estrutura intacta) + guiagem LOS por waypoints\n' ...
    'ligada DIRETO nos holds (psi_ref -> Sum18, h_ref_rel -> Sum8, v_ref -> Sum10, phi_ref -> Switch2). Steps de doublet removidos.\n' ...
    'Planta: modelo NL da Ana (sfunction_DH original) + atuadores (saturacao, rate limit, servo, motor). Rode guiagem_NL.m. Gerado em %s.'], char(datetime('now'))));
save_system(mdl, dst); bdclose(mdl);
fprintf('%s salvo: %d blocos apagados, %d linhas religadas.\n', dst, cnt.apagados, size(lig,1) + 6);
end
