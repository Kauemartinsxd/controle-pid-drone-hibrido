function mdlNL = lqry_v3_build_nl_model(varargin)
% LQRY_V3_BUILD_NL_MODEL  Cria modelo_NL_LQRY_GUIA.slx = modelo_XP_LQRY2_GUIA.slx
% (controlador LQRy + guiagem LOS intocados) com a Planta_XP trocada pela
% planta NL da Ana (sfunction_DH de mirko_run, com a planta hibrida `global HYB`)
% + cadeia de atuadores (saturacao, rate limit, servo, motor com tau
% configuravel) — o mesmo transplante que gerou modelo_NL_DH_GUIA a partir do
% modelo_XP_DH_GUIA (PID). Interface do subsistema Planta preservada
% (11 saidas + Gotos XN/XE/PsiAbs/PsiRel + logs Y_xp/U_xp/t_xplane_log).
%
%   lqry_v3_build_nl_model            % gera lqry_v3/modelo_NL_LQRY_GUIA.slx
%   lqry_v3_build_nl_model('forca', true)
%
% Variaveis de workspace que o modelo passa a exigir (o NL_missao_lqry3 define):
%   Xe_planta (14x1, trim da planta), coef_Sato, coef_Ana, Variacao_Iner,
%   act.rate act.tau eng.rate eng.tau, sat_surf_rad, XP_U_trim4, XP_h_ref0, ...
p = inputParser; p.addParameter('forca', false); p.parse(varargin{:}); o = p.Results;
here  = fileparts(mfilename('fullpath'));
xpDir = fullfile(fileparts(here), 'xplane');
src   = fullfile(xpDir, 'modelo_XP_LQRY2_GUIA.slx');
srcNL = fullfile(xpDir, 'modelo_NL_DH_GUIA.slx');
mdlNL = 'modelo_NL_LQRY_GUIA';
dst   = fullfile(here, [mdlNL '.slx']);
if exist(dst, 'file') && ~o.forca
    fprintf('%s ja existe (use ''forca'', true para refazer).\n', dst); return
end
for m = {mdlNL, 'modelo_XP_LQRY2_GUIA'}
    if bdIsLoaded(m{1}), bdclose(m{1}); end
end
copyfile(src, dst, 'f');
load_system(dst);
nl_ja_aberto = bdIsLoaded('modelo_NL_DH_GUIA');   % o Kaue costuma te-lo aberto: NAO fechar no fim
if ~nl_ja_aberto, load_system(srcNL); end
P = [mdlNL '/Planta'];

%% 1) remove a interface X-Plane
for b = {'read_xp', 'send_xp', 'Clock_XP', 'Clock_snd', 'Term_status'}
    blk = [P '/' b{1}];
    lh = get_param(blk, 'LineHandles');
    for f = {'Inport', 'Outport'}
        for h = lh.(f{1})(:)'
            if h > 0, delete_line(h); end
        end
    end
    delete_block(blk);
end
% relogio da simulacao = canal 10 ("t") do vetor de 14 (Log_t_xplane continua lendo Demux_XP/10)
add_block('simulink/Sources/Clock', [P '/Clock_sim'], 'Position', [40 520 70 550]);
% o comando (pos alpha_protection) ainda precisa chegar ao Log_U_xp se a linha em arvore foi apagada
lhU = get_param([P '/Log_U_xp'], 'LineHandles');
if lhU.Inport(1) < 0, add_line(P, 'alpha_protection/1', 'Log_U_xp/1', 'autorouting', 'on'); end

%% 2) cadeia de atuadores (copiada do modelo_NL_DH_GUIA) alimentada pelo comando absoluto (pos alpha_protection)
add_block('simulink/Signal Routing/Demux', [P '/Dmx_cmd4'], 'Outputs', '4', 'Position', [520 300 525 380]);
add_line(P, 'alpha_protection/1', 'Dmx_cmd4/1', 'autorouting', 'on');
nomes = {'Throttle', 'Elevator', 'Aileron', 'Rudder'};
x0 = 600;
for k = 1:4
    y = 260 + 60*k;
    add_block(['modelo_NL_DH_GUIA/Sat_' nomes{k}], [P '/Sat_' nomes{k}], 'Position', [x0 y x0+30 y+20]);
    add_block(['modelo_NL_DH_GUIA/RL_'  nomes{k}], [P '/RL_'  nomes{k}], 'Position', [x0+60 y x0+90 y+20]);
    if k == 1
        add_block('modelo_NL_DH_GUIA/Eng_Throttle', [P '/Eng_Throttle'], 'Position', [x0+120 y x0+170 y+20]);
        lag = 'Eng_Throttle';
    else
        add_block(['modelo_NL_DH_GUIA/Servo_' nomes{k}], [P '/Servo_' nomes{k}], 'Position', [x0+120 y x0+170 y+20]);
        lag = ['Servo_' nomes{k}];
    end
    add_line(P, sprintf('Dmx_cmd4/%d', k), ['Sat_' nomes{k} '/1'], 'autorouting', 'on');
    add_line(P, ['Sat_' nomes{k} '/1'], ['RL_' nomes{k} '/1'], 'autorouting', 'on');
    add_line(P, ['RL_' nomes{k} '/1'], [lag '/1'], 'autorouting', 'on');
end
% limites de curso parametrizados (X-Plane: +-15 deg) e IC dos atuadores no trim (engate bumpless)
for k = 2:4
    set_param([P '/Sat_' nomes{k}], 'UpperLimit', 'sat_surf_rad', 'LowerLimit', '-sat_surf_rad');
    set_param([P '/Servo_' nomes{k}], 'InitialCondition', sprintf('XP_U_trim4(%d)', k));
end
set_param([P '/Eng_Throttle'], 'InitialCondition', 'XP_U_trim4(1)');
add_block('simulink/Signal Routing/Mux', [P '/Mux_u4act'], 'Inputs', '4', 'Position', [x0+220 300 x0+225 400]);
add_line(P, 'Eng_Throttle/1',   'Mux_u4act/1', 'autorouting', 'on');
add_line(P, 'Servo_Elevator/1', 'Mux_u4act/2', 'autorouting', 'on');
add_line(P, 'Servo_Aileron/1',  'Mux_u4act/3', 'autorouting', 'on');
add_line(P, 'Servo_Rudder/1',   'Mux_u4act/4', 'autorouting', 'on');

%% 3) planta NL (S-function do Mirko/mirko_run: aceita HYB) com vento zero
add_block('simulink/Sources/Constant', [P '/wind_NED'], 'Value', '[0;0;0]', 'Position', [x0+180 440 x0+220 460]);
add_block('simulink/Signal Routing/Mux', [P '/Mux_U7'], 'Inputs', '2', 'Position', [x0+260 330 x0+265 400]);
add_line(P, 'Mux_u4act/1', 'Mux_U7/1', 'autorouting', 'on');
add_line(P, 'wind_NED/1',  'Mux_U7/2', 'autorouting', 'on');
add_block('simulink/User-Defined Functions/S-Function', [P '/Planta_NL_DH'], ...
    'FunctionName', 'sfunction_DH', 'Parameters', 'Xe_planta, coef_Sato, coef_Ana, Variacao_Iner', ...
    'Position', [x0+300 340 x0+400 390]);
add_line(P, 'Mux_U7/1', 'Planta_NL_DH/1', 'autorouting', 'on');
add_block('simulink/Signal Routing/Demux', [P '/Demux_NL18'], 'Outputs', '18', 'Position', [x0+440 200 x0+445 560]);
add_line(P, 'Planta_NL_DH/1', 'Demux_NL18/1', 'autorouting', 'on');
for k = [4 11 12 13 16 17 18]
    add_block('simulink/Sinks/Terminator', [P sprintf('/Term_y%d', k)], 'Position', [x0+480 180+20*k x0+500 195+20*k]);
    add_line(P, sprintf('Demux_NL18/%d', k), sprintf('Term_y%d/1', k), 'autorouting', 'on');
end
% xE reconstruido (copia do modelo_NL_DH_GUIA): xE_dot(VT, alpha, beta, phi, theta, psi) -> integrador
add_block('modelo_NL_DH_GUIA/xE_dot', [P '/xE_dot'], 'Position', [x0+520 600 x0+600 700]);
add_block('simulink/Continuous/Integrator', [P '/Int_xE'], 'Position', [x0+640 640 x0+670 670]);
add_line(P, 'Demux_NL18/1',  'xE_dot/1', 'autorouting', 'on');   % VT
add_line(P, 'Demux_NL18/2',  'xE_dot/2', 'autorouting', 'on');   % alpha
add_line(P, 'Demux_NL18/3',  'xE_dot/3', 'autorouting', 'on');   % beta
add_line(P, 'Demux_NL18/8',  'xE_dot/4', 'autorouting', 'on');   % phi
add_line(P, 'Demux_NL18/9',  'xE_dot/5', 'autorouting', 'on');   % theta
add_line(P, 'Demux_NL18/10', 'xE_dot/6', 'autorouting', 'on');   % psi
add_line(P, 'xE_dot/1', 'Int_xE/1', 'autorouting', 'on');

%% 4) vetor de 14 canais no formato do xp_read_dh: [VT p q r phi theta psi_rel h beta t xN xE psi_abs alpha]
add_block('simulink/Signal Routing/Mux', [P '/Mux_Y14'], 'Inputs', '14', 'Position', [x0+720 200 x0+725 560]);
srcs = {'Demux_NL18/1','Demux_NL18/5','Demux_NL18/6','Demux_NL18/7','Demux_NL18/8','Demux_NL18/9', ...
        'Demux_NL18/10','Demux_NL18/15','Demux_NL18/3','Clock_sim/1','Demux_NL18/14','Int_xE/1','Demux_NL18/10','Demux_NL18/2'};
for k = 1:14
    add_line(P, srcs{k}, sprintf('Mux_Y14/%d', k), 'autorouting', 'on');
end
add_line(P, 'Mux_Y14/1', 'Demux_XP/1', 'autorouting', 'on');
add_line(P, 'Mux_Y14/1', 'Log_Y_xp/1', 'autorouting', 'on');

%% 5) anotacao + salva
add_block('built-in/Note', [P '/Nota_NL'], 'Position', [40 40 60 60]);
set_param([P '/Nota_NL'], 'Text', sprintf(['PLANTA NL (modelo da Ana via sfunction_DH de mirko_run, planta hibrida global HYB)\n' ...
    'no lugar da Planta_XP. Cadeia de atuadores = modelo_NL_DH_GUIA (PID): Sat +-sat_surf_rad, RL act.rate,\n' ...
    'servo act.tau; manete Sat [0,1], RL eng.rate, lag eng.tau. Gerado por lqry_v3_build_nl_model.m em %s.'], char(datetime('now'))));
save_system(mdlNL, dst);
bdclose(mdlNL);
if ~nl_ja_aberto, bdclose('modelo_NL_DH_GUIA'); end
fprintf('modelo criado: %s\n', dst);
end
