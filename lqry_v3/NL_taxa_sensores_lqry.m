% NL_taxa_sensores_lqry.m
% =============================================================
% "O LQRy precisa de sensores muito rapidos" (Mirko): teste ISOLADO da taxa
% de I/O no modelo NAO LINEAR (sem X-Plane, sem UDP, sem jitter, sem motor
% do XP9). Mesma missao (oval GUI, 15 m/s), mesmos ganhos (LQRy do Mirko com
% o psi Hold novo de 10/09), mesma planta (Ana, motor tau 0,3 s, curso 15 deg)
% — so muda a taxa em que o controlador recebe medida e entrega comando
% (nl_insere_zoh). Ts_io = 0 e o caso do SIL do Mirko (planta continua).
%
% Uso:  run('...\lqry_v3\NL_taxa_sensores_lqry.m')
% Saida: xplane/voos/NL_taxa_sensores_LQRYmirko_<data>.{txt,png} + um
%        NL_missao_*.mat por caso.
% =============================================================

here = fileparts(mfilename('fullpath'));
addpath(here);
xpV = fullfile(fileparts(here), 'xplane', 'voos');

if ~exist('CONF_taxas', 'var') || isempty(CONF_taxas)
    CONF_taxas = [0 0.01 0.02 0.05 0.1 0.2];      % s: 0 = continuo | 100 | 50 | 20 | 10 | 5 Hz
end
if ~exist('CONF_n_delay', 'var') || isempty(CONF_n_delay), CONF_n_delay = 0; end
if ~exist('CONF_eng_tau', 'var') || isempty(CONF_eng_tau), CONF_eng_tau = 0.3; end

R2D = 180/pi; T_MAX = 75;
res = struct('Ts_io', {}, 'rotulo', {}, 'arquivo', {}, 'voo', {}, 'm', {});

for kk = 1:numel(CONF_taxas)
    Ts_io = CONF_taxas(kk);
    if Ts_io > 0, rot = sprintf('%.0fHz', 1/Ts_io); else, rot = 'continuo'; end
    fprintf('\n################ caso %d/%d: I/O %s ################\n', kk, numel(CONF_taxas), rot);

    % --- config do lancador NL (mesma receita do voo X-Plane de hoje) ---
    NL3_ganhos_dir = fullfile(here, 'LQRy_Guiagem', 'ganhos_mirko');
    NL3_ganhos     = 'orig';
    NL3_VT         = 15;
    NL3_eng_tau    = CONF_eng_tau;
    NL3_sat_deg    = 15;
    NL3_Cm_thr     = 0;
    NL3_plot       = false;
    NL3_tag        = sprintf('LQRYmirko_psiNovo_oval_taxaIO_%s', rot);
    NL3_Ts_io      = Ts_io;
    NL3_n_delay    = CONF_n_delay;
    clear NL3_WPs NL3_TimeXP NL3_R_accept

    NL_missao_lqry3

    % --- metricas ate T_MAX (mesma janela da comparacao X-Plane) ---
    Y = voo.Y; U = voo.U; t = voo.t(:);
    jan = t <= T_MAX; Y = Y(:, jan); U = U(jan, :); t = t(jan);
    m = struct();
    m.fim      = voo.t(end);
    m.cap      = NL3_resultado.capturas;
    m.h_rms    = rms(Y(8,:) - 600);
    m.h_min    = min(Y(8,:)); m.h_max = max(Y(8,:));
    m.VT_rms   = rms(Y(1,:) - 15);
    m.VT_min   = min(Y(1,:)); m.VT_max = max(Y(1,:));
    m.phi_max  = max(abs(Y(5,:)))*R2D;
    m.th_min   = min(Y(6,:))*R2D; m.th_max = max(Y(6,:))*R2D;
    m.al_max   = max(Y(14,:))*R2D;
    m.de_min   = min(U(:,2))*R2D; m.de_max = max(U(:,2))*R2D;
    m.thr_min  = min(U(:,1)); m.thr_max = max(U(:,1));
    m.thr_fora = 100*mean(U(:,1) < 0 | U(:,1) > 1);
    m.sup_sat  = 100*mean(max(abs(U(:,2:4)), [], 2)*R2D > 15);
    k_dep = find(abs(voo.Y(5,:))*R2D > 60, 1); if isempty(k_dep), m.t_dep = NaN; else, m.t_dep = voo.t(k_dep); end
    k_st  = find(voo.Y(14,:)*R2D > 18.5, 1);   if isempty(k_st),  m.t_est = NaN; else, m.t_est = voo.t(k_st);  end
    res(end+1) = struct('Ts_io', Ts_io, 'rotulo', rot, 'arquivo', vooFile, 'voo', voo, 'm', m); %#ok<SAGROW>
end

%% tabela + figura (a partir dos .mat salvos)
CONF_arquivos = {res.arquivo};
NL_taxa_sensores_tabela
