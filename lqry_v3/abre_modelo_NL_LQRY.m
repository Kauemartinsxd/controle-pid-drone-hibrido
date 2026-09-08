function abre_modelo_NL_LQRY(varargin)
% ABRE_MODELO_NL_LQRY  Abre o modelo_NL_LQRY_GUIA.slx (LQRy do Mirko + guiagem LOS +
% planta NL da Ana) PRONTO PARA PLAY, com o workspace da missao oval carregado, e
% destaca em amarelo o caminho da guiagem: o chart Guidance_Star (LOS por waypoints),
% os Goto/From que levam psi_ref / dh_ref / dv_ref / phi_ref ate os holds, e os
% somadores (Add_*_guia) onde essas referencias entram nos integradores dos holds.
%
%   abre_modelo_NL_LQRY                 % ganhos ORIGINAIS do Mirko, oval da GUI
%   abre_modelo_NL_LQRY('ganhos','v3')  % ganhos v3
%   abre_modelo_NL_LQRY('chart', true)  % abre tambem o codigo do Guidance_Star
%
% Play direto no Simulink roda o .slx "cru" (sem as edicoes em memoria do NL_missao_lqry3:
% CI do integrador de H, fim automatico, pulso de V_T dos doublets neutralizado).
p = inputParser;
p.addParameter('ganhos', 'orig');
p.addParameter('VT', 15);
p.addParameter('chart', false);
p.parse(varargin{:}); o = p.Results;

here  = fileparts(mfilename('fullpath'));
xpDir = fullfile(fileparts(here), 'xplane');
mirko = 'C:\Users\kaue\Documents\Dissertacao_Mestrado\lqry_mirko_atualizado\reproducao_SIL\mirko_run';
raizN = 'C:\Users\kaue\Documents\Dissertacao_Mestrado\lqry_mirko_atualizado\Nova pasta';
addpath(xpDir); addpath(here); addpath(mirko);   % sfunction_DH de mirko_run (4 parametros)

%% workspace da missao (mesmo do NL_missao_lqry3, oval da GUI)
if strcmp(o.ganhos, 'v3'), gd = fullfile(here, 'ganhos'); else, gd = raizN; end
for f = {'Ganho_hold_theta','Ganho_hold_H','Ganho_hold_VT','Ganho_hold_phi','Ganho_hold_psi'}
    s = load(fullfile(gd, [f{1} '.mat'])); fn = fieldnames(s);
    for k = 1:numel(fn), assignin('base', fn{k}, s.(fn{k})); end
end
S = load(fullfile(raizN, 'Dados_Trim.mat')); Plantas = S.Plantas; assignin('base', 'Plantas', Plantas);
i = find([Plantas.Ve] == o.VT & [Plantas.He] == 600, 1);
v = struct('i', i, 'i_lat', i, 'Ts', 1/100, 'surfaces', 24, 'Variacao_Iner', 0, ...
    'VT_Throttle', 1, 'att_alt', 0, 'phi_psi', 0, ...                       % Caso 4
    'refPhi', 0, 'reftheta', 0, 'refAlt', 0, 'refPsi', 0, 'refVel', o.VT, ...
    'K_bank_guia', 0.1975, 'phi_max_guia', deg2rad(20), 'XP_clamp_lqry', [-pi pi], ...
    'prot_on', 0, 'alpha_prot', deg2rad(16), ...
    'Xe_planta', Plantas(i).Xe, 'XP_U_trim4', double(Plantas(i).Ue(1:4)), 'de_trim', double(Plantas(i).Ue(2)), ...
    'XP_h_ref0', 600, 'coef_Ana', 1, 'coef_Sato', 0, 'theta_e_ff', double(Plantas(i).Xe(8)), ...
    'act', struct('rate', deg2rad(150), 'tau', 0.05, 'bw', 20), 'eng', struct('rate', 1, 'tau', 0.3), ...
    'sat_surf_rad', deg2rad(15));
v.Xe_planta(12) = -600;
WPs = [256 0 600 15; 416 160 600 15; 256 320 600 15; 0 320 600 15; -160 160 600 15; 0 0 600 15];
v.WPs = WPs; v.R_accept = 100; v.h_ref0 = 600; v.VT_ref0 = o.VT;
v.N_WPs = size(WPs,1); v.WPfim_N = WPs(end,1); v.WPfim_E = WPs(end,2); v.TimeXP = 160;
v.XP_IC_int_speed = 0; v.XP_IC_int_theta = 0; v.XP_IC_int_alt = 0;
fn = fieldnames(v); for k = 1:numel(fn), assignin('base', fn{k}, v.(fn{k})); end
global HYB; HYB = [];

%% abre e destaca a guiagem
mdl = 'modelo_NL_LQRY_GUIA';
open_system(fullfile(here, [mdl '.slx']));
set_param(mdl, 'StopTime', num2str(v.TimeXP));
hilite_system(mdl, 'none');
for b = {'Guidance_Star', 'Goto_PsiRefGuia', 'Goto_DhGuia', 'Goto_DvGuia', 'Goto_PhiRefGuia', ...
         'From_PsiRefGuia', 'From_DhGuia', 'From_DvGuia', 'From_PhiRefGuia', ...
         'Add_psi_guia', 'Add_h_guia', 'Add_v_guia', 'Add_phi_guia', ...
         'From_GXN', 'From_GXE', 'From_GPsiAbs', 'From_GPsiRel'}
    try, hilite_system([mdl '/' b{1}], 'find'); catch, end
end
if o.chart, open_system([mdl '/Guidance_Star']); end
fprintf(['%s aberto com ganhos %s, planta %d (%g m/s), oval de 6 WPs.\n' ...
         'Destaque amarelo = guiagem: Guidance_Star -> Goto/From -> Add_*_guia -> integradores dos holds.\n' ...
         'Referencias: psi_ref_rel (LOS) entra no psi Hold (Sum18 -> Integrator5); dh_ref no Alt Hold (Sum8 -> Integrator2);\n' ...
         'dv_ref no Vel Hold (Sum10 -> Integrator3); phi_ref_guia so e usado no modo phi Hold (phi_psi = 1).\n'], mdl, o.ganhos, i, o.VT);
end
