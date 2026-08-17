%% pid_recampanha_v2 — PID de registro (retunado+b0) por eixo, modos de missao
%  Rig: CL_NL_DH_SIL_rob (copia wind-enabled da arquitetura de missao)
%  Runs: {vel,lat,long} x {comoesta,equiparado} + comb equiparado
%  (comb comoesta = Dados_mat_ROBUSTEZ\PID_nom.mat, ja de registro)
run('C:\Users\kaue\Documents\PID_DH\controle-pid-drone-hibrido\DH_inicializacao.m');
raiz = 'C:\Users\kaue\Documents\PID_DH\HIL_PID\Matlab';
cd(raiz); PID_H_params;
pasta = fullfile(raiz,'Dados_mat_comparacao');
wind_ts = timeseries(zeros(2,3), [0;300]);
cfg0 = struct('act',act,'eng',eng,'clamp',theta_ref_clamp,'tau',tau_ref);

VT_Throttle = 1; phi_psi = 0; att_alt = 0;
VT_ref_base = VT_ref;
theta_step_init=0; theta_step_final=0; theta_step_t=1e9; theta_step_t2=1e9; theta_step_t3=1e9;
phi_step_init=0; phi_step_final=0; phi_step_t=1e9; phi_step_t2=1e9; phi_step_t3=1e9;
VT_pulse_delta=0; VT_pulse_t=15; VT_pulse_t2=50;

% cenarios: {nome, refVel, refPsi, refAlt}
cz = {'vel',15.2,0,0; 'lat',12,5,0; 'long',12,0,5; 'comb',15.2,5,5};
for c = 1:4
  for m = 1:2
    if c<4 || m==2   % comb comoesta ja existe (PID_nom)
      refVel = cz{c,2}; refPsi = cz{c,3}; refAlt = cz{c,4};
      VT_step_delta = refVel - VT_ref_base; VT_step_t = 5;
      if cz{c,3}>0, psi_ref_init=0; psi_ref_final=deg2rad(refPsi);
        psi_ref_t=50; psi_ref_t2=100; psi_ref_t3=150;
      else, psi_ref_init=0; psi_ref_final=0; psi_ref_t=1e9; psi_ref_t2=1e9; psi_ref_t3=1e9; end
      if cz{c,4}>0, h_step_init=0; h_step_final=refAlt;
        h_step_t=80; h_step_t2=160; h_step_t3=240;
      else, h_step_init=0; h_step_final=0; h_step_t=1e9; h_step_t2=1e9; h_step_t3=1e9; end
      if m==1, modo='comoesta';
        act=cfg0.act; eng=cfg0.eng; theta_ref_clamp=cfg0.clamp; tau_ref=cfg0.tau;
      else, modo='equiparado';   % casado ao rig LQRy v2
        act.rate=1e6; act.bw=24; act.tau=1/act.bw;
        eng.rate=1e6; eng.tau=0.1; theta_ref_clamp=[-10 10]; tau_ref=0.01;
      end
      fprintf('PID2 %s/%s...\n', cz{c,1}, modo); tic;
      out_sim = sim('CL_NL_DH_SIL_rob','StopTime','300');
      % extrator robusto (arquitetura de missao — sem theta/phi step sigs)
      R2D=180/pi; Ys=out_sim.Y.signals.values; Us=out_sim.U.signals.values;
      t=out_sim.Y.time; lg=out_sim.logsout;
      gr=@(nm) interp1(lg.getElement(nm).Values.Time, lg.getElement(nm).Values.Data, t,'linear','extrap');
      mk=@(vals) struct('time',t,'signals',struct('values',vals));
      S = struct('tout',t);
      S.VT_NL  = mk([gr('VT_ref_f'), Ys(:,1)]);
      S.H_NL   = mk([gr('h_ref_f'),  Ys(:,15)]);
      S.psi_NL = mk([Ys(:,10)*R2D, gr('psi_ref_f')*R2D]);
      S.theta_NL = mk([Ys(1,9)*R2D*ones(size(t)), Ys(:,9)*R2D]);
      S.phi_NL   = mk([zeros(size(t)), Ys(:,8)*R2D]);
      S.elev_NL  = mk(Us(:,2)*R2D); S.Throttle_NL = mk(Us(:,1));
      S.ail_NL   = mk(Us(:,3)*R2D); S.rud_NL      = mk(Us(:,4)*R2D); %#ok<STRNU>
      save(fullfile(pasta,sprintf('pid2_%s_%s.mat',cz{c,1},modo)),'S');
      fprintf('  ok %.0fs\n', toc);
    end
  end
end
fprintf('pid_recampanha_v2 concluida.\n');
