%% sims_D_pid — PID nos 3 cenarios, em DUAS configuracoes:
%   'comoesta'   : modelo como esta (servo real, clamp de theta_ref, pre-filtro)
%   'equiparado' : atuador ~ideal + clamp desligado + pre-filtro desligado
% Equiparacao 100% por workspace (nenhum modelo alterado nesta rodada) —
% conforme COMPARACAO_JUSTA.md do repo do LQRy.
% RODAR EM SESSAO PROPRIA (nao misturar paths com a arvore do LQRy).

run('C:\Users\kaue\Documents\PID_DH\controle-pid-drone-hibrido\DH_inicializacao.m');
raiz = 'C:\Users\kaue\Documents\PID_DH\HIL_PID\Matlab';
pasta = fullfile(raiz, 'Dados_mat_comparacao');
if ~exist(pasta,'dir'), mkdir(pasta); end
cd(raiz); PID_H_params;

% cenarios (identicos aos do artigo)
% NOTA: sem pulso de V_T. O harness SIL do LQRy nao aplica o pulso de +5 m/s
% (a referencia dele fica constante), entao o rig do PID tambem nao aplica —
% as referencias tem de ser identicas nos dois lados.
base = struct('VT_step_delta',0,'VT_step_t',5,'VT_pulse_delta',0, ...
    'VT_pulse_t',1e9,'VT_pulse_t2',1e9, ...
    'phi_step_init',0,'phi_step_final',0,'phi_step_t',1e9,'phi_step_t2',1e9,'phi_step_t3',1e9, ...
    'theta_step_init',0,'theta_step_final',0,'theta_step_t',1e9,'theta_step_t2',1e9,'theta_step_t3',1e9, ...
    'psi_ref_init',0,'psi_ref_final',0,'psi_ref_t',1e9,'psi_ref_t2',1e9,'psi_ref_t3',1e9, ...
    'h_step_init',0,'h_step_final',0,'h_step_t',1e9,'h_step_t2',1e9,'h_step_t3',1e9);

cen = struct();
cen.vel = base;                                   % S1: so velocidade
cen.vel.VT_step_delta = 3.2;                      % 12,0 -> 15,2 m/s em t=5 s
cen.latero = base;                                % S2: so latero
cen.latero.phi_step_final = deg2rad(5);
cen.latero.phi_step_t=50; cen.latero.phi_step_t2=100; cen.latero.phi_step_t3=150;
cen.long = base;                                  % S3: so longitudinal
cen.long.theta_step_final = deg2rad(5);
cen.long.theta_step_t=100; cen.long.theta_step_t2=150; cen.long.theta_step_t3=200;
cen.comb = cen.latero;                            % S4: combinado
cen.comb.theta_step_final = deg2rad(5);
cen.comb.theta_step_t=100; cen.comb.theta_step_t2=150; cen.comb.theta_step_t3=200;

% guarda a configuracao "como esta" do workspace
cfg0 = struct('act',act,'eng',eng,'clamp',theta_ref_clamp,'tau',tau_ref);

VT_Throttle=1; phi_psi=1; att_alt=1;      % VelHold + PhiHold + ThetaHold
nomes = {'vel','latero','long','comb'};

for c = 1:numel(nomes)
    for m = 1:2
        % ---- referencias do cenario ----
        cfg = cen.(nomes{c});
        fn = fieldnames(cfg);
        for k = 1:numel(fn), eval(sprintf('%s = cfg.%s;', fn{k}, fn{k})); end

        % ---- configuracao do rig ----
        if m == 1
            modo = 'comoesta';
            act = cfg0.act;  eng = cfg0.eng;
            theta_ref_clamp = cfg0.clamp;  tau_ref = cfg0.tau;
        else
            modo = 'equiparado';
            act.rate = 1e6;  act.bw = 100;  act.tau = 1/act.bw;   % servo ~ideal
            eng.rate = 1e6;  eng.tau = 0.01;                      % motor ~ideal
            theta_ref_clamp = [-10 10];      % rad: desativa o clamp de +-10 deg
            tau_ref = 0.01;                  % desliga o pre-filtro (so vale apos o fix)
        end

        fprintf('PID %s / %s...\n', nomes{c}, modo);
        out_sim = sim('CL_NL_DH_SIL_casos', 'StopTime', '300');
        S = extrai_sinais_PID(out_sim); %#ok<NASGU>
        save(fullfile(pasta, sprintf('pid_%s_%s.mat', nomes{c}, modo)), 'S');
    end
end
fprintf('sims D (PID) concluidas.\n');
