%% sims_D_lqry — LQRy nos 3 cenarios, SEM nenhuma alteracao no controlador
%  nem nos modelos dele (so as variaveis de referencia do harness).
%  RODAR EM SESSAO PROPRIA: as duas arvores tem funcoes homonimas
%  (sfunction_DH, dyn_rigidbody_DH, modelo_DH, coef_DH, ...) e misturar
%  os paths produz resultado silenciosamente errado.

restoredefaultpath; rehash toolboxcache;   % garante que nada do PID esta no path
raiz = 'C:\Users\kaue\Documents\PID_DH\HIL_PID\Matlab';
work = fullfile(raiz, 'mirko_replica');
pasta = fullfile(raiz, 'Dados_mat_comparacao');
if ~exist(pasta,'dir'), mkdir(pasta); end
cd(work);
addpath(genpath('Control_NL_HIL'));
addpath(genpath('Modelo_DH_HIL'));
addpath(genpath('Dados_mat_SIL'));

load('Ganho_hold_theta.mat'); load('Ganho_hold_H.mat'); load('Ganho_hold_VT.mat');
load('Ganho_hold_phi.mat');   load('Ganho_hold_psi.mat'); load('Dados_Trim.mat');
Ts = 1/100; surfaces = 24; Ve = 12; he = 600; gammae = 0; R = 6371000;
coef_Ana = 1; coef_Sato = 0;
VT_Throttle = 1; phi_psi = 1; att_alt = 1;
refPsi = 0; refAlt = 0;

cen = {'vel',    0, 0, 15.2;      % {nome, refPhi, reftheta, refVel}
       'latero', 5, 0, 12.0;
       'long',   0, 5, 12.0;
       'comb',   5, 5, 12.0};

for c = 1:size(cen,1)
    refPhi = cen{c,2}; reftheta = cen{c,3}; refVel = cen{c,4};
    fprintf('LQRy %s (refPhi=%g reftheta=%g refVel=%g)...\n', cen{c,1}, refPhi, reftheta, refVel);
    out_sim = sim('CL_NL_DH_SIL_manobras', 'StopTime', '300');
    S = struct();
    for nm = {'theta_NL','VT_NL','H_NL','phi_NL','elev_NL','Throttle_NL'}
        S.(nm{1}) = out_sim.get(nm{1});
    end
    save(fullfile(pasta, sprintf('lqry_%s.mat', cen{c,1})), 'S');
end
fprintf('sims D (LQRy) concluidas.\n');
