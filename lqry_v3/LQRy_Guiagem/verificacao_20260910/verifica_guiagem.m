function R = verifica_guiagem(varargin)
% VERIFICA_GUIAGEM  Harness de verificacao (fora do controlador) do pacote LQRy_Guiagem do Mirko.
%   R = verifica_guiagem('mdl','CL_NL_DH_GUIA'|'CL_NL_DH_GUIA_Original', 'ganhos_dir',<pasta>, ...
%                        'psi_dir',<pasta so' do Ganho_hold_psi>, 'missao','oval'|'agressivo', ...
%                        'eng_tau',0.3, 'VT_missao',NaN, 'tag','...', 'salvar',true)
% Reproduz a configuracao do guiagem_NL.m (mesmas variaveis, mesmos valores), mas como FUNCAO:
% as variaveis vao para o modelo por Simulink.SimulationInput/setVariable, sem tocar o workspace
% base e sem 'clear all'. Nada do controlador e' alterado. Resultados em verificacao_*/resultados.
p = inputParser;
p.addParameter('mdl', 'CL_NL_DH_GUIA');
p.addParameter('ganhos_dir', '');
p.addParameter('psi_dir', '');
p.addParameter('missao', 'oval');
p.addParameter('WPs', []);
p.addParameter('R_accept', NaN);
p.addParameter('VT_missao', NaN);
p.addParameter('eng_tau', 0.3);
p.addParameter('T_max', 200);
p.addParameter('tag', '');
p.addParameter('salvar', true);
p.addParameter('fechar_figs', true);
p.parse(varargin{:}); o = p.Results;

vdir = fileparts(mfilename('fullpath'));          % .../LQRy_Guiagem/verificacao_YYYYMMDD
here = fileparts(vdir);                            % .../LQRy_Guiagem (pacote do Mirko, intocado)
addpath(here, fullfile(here, 'planta'));
if isempty(o.ganhos_dir), o.ganhos_dir = fullfile(here, 'ganhos_mirko'); end

%% ---- missao (presets identicos aos do gui_guiagem_NL / guiagem_NL) ----
if ~isempty(o.WPs)
    WPs = o.WPs; R_accept = 100;
else
    switch lower(o.missao)
        case 'oval',      WPs = [256 0 600 15; 416 160 600 15; 256 320 600 15; 0 320 600 15; -160 160 600 15; 0 0 600 15]; R_accept = 100;
        case 'agressivo', WPs = [260 0 620 18; 260 260 600 15; 0 260 620 18; 0 0 600 15]; R_accept = 110;
        otherwise, error('missao desconhecida: %s', o.missao);
    end
end
if ~isnan(o.R_accept), R_accept = o.R_accept; end
VT_missao = o.VT_missao;
if isnan(VT_missao)                                % como a GUI (vtPlanta): mediana das velocidades dos WPs -> 12/15/18
    vm = median(WPs(:, 4)); cand = [12 15 18]; [~, q] = min(abs(cand - vm) + 1e-6*cand); VT_missao = cand(q);
end
eng_tau = o.eng_tau; T_max = o.T_max;
sat_deg = 15; act_rate_deg = 150; act_tau = 0.05;

%% ---- dados: trim + ganhos (todas as variaveis dos .mat, como o load sem saida do script) ----
V = struct();
D = load(fullfile(here, 'ganhos_mirko', 'Dados_Trim.mat'));
for f = fieldnames(D)', V.(f{1}) = D.(f{1}); end
Plantas = D.Plantas;
for f = {'Ganho_hold_theta', 'Ganho_hold_H', 'Ganho_hold_VT', 'Ganho_hold_phi', 'Ganho_hold_psi'}
    src = fullfile(o.ganhos_dir, [f{1} '.mat']);
    if strcmp(f{1}, 'Ganho_hold_psi') && ~isempty(o.psi_dir), src = fullfile(o.psi_dir, 'Ganho_hold_psi.mat'); end
    G = load(src);
    for g = fieldnames(G)', V.(g{1}) = G.(g{1}); end
end
i = find([Plantas.Ve] == VT_missao & [Plantas.He] == 600, 1);
assert(~isempty(i), 'nao ha planta para %g m/s @ 600 m (use 12, 15 ou 18)', VT_missao);

%% ---- variaveis do modelo (copiadas 1:1 do guiagem_NL.m) ----
V.Ts = 1/100; V.surfaces = 24; V.Variacao_Iner = 0; V.coef_Ana = 1; V.coef_Sato = 0;
Xe_planta = Plantas(i).Xe; Xe_planta(12) = -600; V.Xe_planta = Xe_planta;
V.U_trim = double(Plantas(i).Ue(1:4)); V.de_trim = V.U_trim(2); V.h0 = 600;
V.act = struct('rate', deg2rad(act_rate_deg), 'tau', act_tau, 'bw', 1/act_tau);
V.eng = struct('rate', 1.0, 'tau', eng_tau); V.sat_surf_rad = deg2rad(sat_deg);
V.prot_on = 0; V.alpha_prot = deg2rad(16);
V.K_bank_guia = 0.1975; V.phi_max_guia = deg2rad(20);
V.h_ref0 = V.h0; V.VT_ref0 = VT_missao;
V.WPs = WPs; V.R_accept = R_accept; V.N_WPs = size(WPs, 1); V.WPfim_N = WPs(end, 1); V.WPfim_E = WPs(end, 2);
V.VT_Throttle = 1; V.phi_psi = 0; V.att_alt = 0;
V.i = i; V.VT_missao = VT_missao; V.eng_tau = eng_tau; V.sat_deg = sat_deg;
V.xi_alt0 = double(Plantas(i).Xe(8)) / double(V.GintLong_Alt{i});

%% ---- simula ----
mdl = o.mdl;
if ~bdIsLoaded(mdl), load_system(fullfile(here, [mdl '.slx'])); end
in = Simulink.SimulationInput(mdl);
in = in.setModelParameter('StopTime', num2str(T_max));
for f = fieldnames(V)', in = in.setVariable(f{1}, V.(f{1})); end
if isempty(o.psi_dir), spsi = ''; else, spsi = [' + psi de ' o.psi_dir]; end
fprintf('\n>>> verifica_guiagem [%s]: modelo %s | ganhos %s%s | missao %s (%d WPs, R_accept %g m) | planta %d (%s) | motor tau %g s\n', ...
    o.tag, mdl, o.ganhos_dir, spsi, o.missao, size(WPs, 1), R_accept, i, Plantas(i).nome, eng_tau);
R = struct('tag', o.tag, 'mdl', mdl, 'ganhos_dir', o.ganhos_dir, 'psi_dir', o.psi_dir, 'missao', o.missao, 'VT_missao', VT_missao, ...
           'i', i, 'eng_tau', eng_tau, 'WPs', WPs, 'R_accept', R_accept, 'erro', '');
t0 = tic;
try
    out = sim(in);
catch ME
    R.erro = ME.message; fprintf('!!! ERRO na simulacao: %s\n', ME.message); return;
end
R.t_cpu = toc(t0);

%% ---- resumo (identico ao guiagem_NL.m) ----
Y = squeeze(out.Y_xp); if size(Y, 1) ~= 14, Y = Y.'; end       % [VT p q r phi theta psi h beta t xN xE psi_abs alpha]
t = out.tout(:);
U = squeeze(out.U_xp); if size(U, 1) == 4 && size(U, 2) > 4, U = U.'; end
if size(U, 1) ~= numel(t), U = interp1(linspace(0, t(end), size(U, 1))', U, t, 'previous', 'extrap'); end
R2D = 180/pi;
R.t_end = t(end);
R.VT = [min(Y(1,:)) max(Y(1,:))]; R.h = [min(Y(8,:)) max(Y(8,:))]; R.phi_max = max(abs(Y(5,:)))*R2D;
R.theta = [min(Y(6,:)) max(Y(6,:))]*R2D; R.alpha = [min(Y(14,:)) max(Y(14,:))]*R2D;
R.thr = [min(U(:,1)) max(U(:,1))]; R.de = [min(U(:,2)) max(U(:,2))]*R2D; R.da = [min(U(:,3)) max(U(:,3))]*R2D; R.dr = [min(U(:,4)) max(U(:,4))]*R2D;
R.frac_sat_de = mean(abs(U(:,2)) >= deg2rad(14.99)); R.frac_sat_da = mean(abs(U(:,3)) >= deg2rad(14.99));
R.frac_sat_dr = mean(abs(U(:,4)) >= deg2rad(14.99)); R.frac_sat_thr = mean(U(:,1) <= 0.001 | U(:,1) >= 0.999);
R.frac_alem15 = mean(abs(U(:,2)) > deg2rad(15) | abs(U(:,3)) > deg2rad(15) | abs(U(:,4)) > deg2rad(15));
R.departure = (min(Y(8,:)) < 400) || (max(abs(Y(5,:))) > pi/2) || (max(abs(Y(6,:))) > pi/2);
N_WPs = size(WPs, 1); dmin = zeros(1, N_WPs);
for k = 1:N_WPs, dmin(k) = min(sqrt((Y(11,:) - WPs(k,1)).^2 + (Y(12,:) - WPs(k,2)).^2)); end
R.dmin = dmin; R.n_hit = sum(dmin <= R_accept); R.N_WPs = N_WPs;
fprintf('sim %.1f s (%.0f s CPU) | VT %.1f..%.1f | h %.1f..%.1f m | phi max %.1f | theta %.1f..%.1f | alpha %.1f..%.1f | de %+.1f..%+.1f | da %+.1f..%+.1f | dr %+.1f..%+.1f deg | manete %.2f..%.2f\n', ...
    R.t_end, R.t_cpu, R.VT, R.h, R.phi_max, R.theta, R.alpha, R.de, R.da, R.dr, R.thr);
fprintf('saturacao (fracao do tempo): de %.1f%% da %.1f%% dr %.1f%% manete %.1f%% | comandos alem de 15 deg: %.1f%% | departure: %d\n', ...
    100*R.frac_sat_de, 100*R.frac_sat_da, 100*R.frac_sat_dr, 100*R.frac_sat_thr, 100*R.frac_alem15, R.departure);
for k = 1:N_WPs
    if dmin(k) <= R_accept, s = 'CAPTURADO'; else, s = '-- fora --'; end
    fprintf('WP%d (N %+7.1f, E %+7.1f, h %g, V %g): dist min %6.1f m  %s\n', k, WPs(k,:), dmin(k), s);
end
fprintf('CAPTURAS: %d/%d\n', R.n_hit, N_WPs);

%% ---- figuras + .mat ----
if o.salvar
    close(findall(groot, 'Type', 'figure', 'Tag', 'guiagem_NL'));
    rd = fullfile(vdir, 'resultados'); if ~exist(rd, 'dir'), mkdir(rd); end
    base = fullfile(rd, sprintf('verif_%s_%s_%s_%s', o.tag, mdl, o.missao, datestr(now, 'yyyymmdd_HHMMSS')));
    titulo = sprintf('%s | %s | %s | planta %d (%g m/s) | motor %g s: %d/%d capturas', o.tag, mdl, o.missao, i, VT_missao, eng_tau, R.n_hit, N_WPs);
    fig = plot_guiagem_NL(t, Y, U, WPs, R_accept, strrep(titulo, '_', '\_'), '');
    figs = findall(groot, 'Type', 'figure', 'Tag', 'guiagem_NL'); fig2 = setdiff(figs, fig);
    exportgraphics(fig, [base '.png'], 'Resolution', 120);
    if ~isempty(fig2), exportgraphics(fig2(1), [base '_comandos.png'], 'Resolution', 120); end
    if o.fechar_figs, close(figs); end
    save([base '.mat'], 't', 'Y', 'U', 'WPs', 'R_accept', 'R');
    R.arq = base; fprintf('figuras e dados: %s.*\n', base);
end
end
