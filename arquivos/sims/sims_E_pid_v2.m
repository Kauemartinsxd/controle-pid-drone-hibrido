%% sims_E_pid_v2 — PID (ganhos de registro, repo HEAD retunado) nos 4 cenarios
%  'comoesta'   : rig PID integral (servo real, clamp, pre-filtro)
%  'equiparado' : harness casado ao rig LQRy v2 (lag de superficie 24 rad/s,
%                 throttle tau=0.1 s, SEM rate limit, clamp off, pre-filtro off)
%  Saida: Dados_mat_comparacao\pid2_<cen>_<modo>.mat
run('C:\Users\kaue\Documents\PID_DH\controle-pid-drone-hibrido\DH_inicializacao.m');
raiz = 'C:\Users\kaue\Documents\PID_DH\HIL_PID\Matlab';
pasta = fullfile(raiz, 'Dados_mat_comparacao');
cd(raiz); PID_H_params;

base = struct('VT_step_delta',0,'VT_step_t',5,'VT_pulse_delta',0, ...
    'VT_pulse_t',1e9,'VT_pulse_t2',1e9, ...
    'phi_step_init',0,'phi_step_final',0,'phi_step_t',1e9,'phi_step_t2',1e9,'phi_step_t3',1e9, ...
    'theta_step_init',0,'theta_step_final',0,'theta_step_t',1e9,'theta_step_t2',1e9,'theta_step_t3',1e9, ...
    'psi_ref_init',0,'psi_ref_final',0,'psi_ref_t',1e9,'psi_ref_t2',1e9,'psi_ref_t3',1e9, ...
    'h_step_init',0,'h_step_final',0,'h_step_t',1e9,'h_step_t2',1e9,'h_step_t3',1e9);

cen = struct();
cen.vel = base;  cen.vel.VT_step_delta = 3.2;
cen.lat = base;  cen.lat.phi_step_final = deg2rad(5);
cen.lat.phi_step_t=50; cen.lat.phi_step_t2=100; cen.lat.phi_step_t3=150;
cen.long = base; cen.long.theta_step_final = deg2rad(5);
cen.long.theta_step_t=100; cen.long.theta_step_t2=150; cen.long.theta_step_t3=200;
cen.comb = cen.lat;
cen.comb.theta_step_final = deg2rad(5);
cen.comb.theta_step_t=100; cen.comb.theta_step_t2=150; cen.comb.theta_step_t3=200;

cfg0 = struct('act',act,'eng',eng,'clamp',theta_ref_clamp,'tau',tau_ref);
VT_Throttle=1; phi_psi=1; att_alt=1;
nomes = {'vel','lat','long','comb'};
for c = 1:numel(nomes)
    for m = 1:2
        cfg = cen.(nomes{c});
        fn = fieldnames(cfg);
        for k = 1:numel(fn), eval(sprintf('%s = cfg.%s;', fn{k}, fn{k})); end
        if m == 1
            modo = 'comoesta';
            act = cfg0.act;  eng = cfg0.eng;
            theta_ref_clamp = cfg0.clamp;  tau_ref = cfg0.tau;
        else
            modo = 'equiparado';                 % casado ao rig LQRy v2
            act.rate = 1e6;  act.bw = 24;  act.tau = 1/act.bw;
            eng.rate = 1e6;  eng.tau = 0.1;
            theta_ref_clamp = [-10 10];          % rad: clamp efetivamente off
            tau_ref = 0.01;                      % pre-filtro off
        end
        fprintf('PID2 %s / %s...\n', nomes{c}, modo);
        out_sim = sim('CL_NL_DH_SIL_casos', 'StopTime', '300');
        S = extrai_sinais_PID(out_sim); %#ok<NASGU>
        save(fullfile(pasta, sprintf('pid2_%s_%s.mat', nomes{c}, modo)), 'S');
    end
end
fprintf('sims_E_pid_v2 concluidas.\n');
