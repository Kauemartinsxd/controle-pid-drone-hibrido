% guiagem_NL.m — LQRy do DH (Mirko, CL_NL_DH_18_jun_2026, estrutura intacta) com
% GUIAGEM LOS POR WAYPOINTS no modelo nao linear da Ana. Pacote autocontido:
%   guiagem_NL.m        este script (configuracao + simulacao + resumo + figuras)
%   gui_guiagem_NL.m    mapa clicavel de waypoints que chama este script (>> gui_guiagem_NL)
%   plot_guiagem_NL.m   figuras (trajetoria, series temporais, comandos)
%   CL_NL_DH_GUIA.slx   o modelo: controlador do Mirko + chart Guidance_Star + planta NL + atuadores
%   planta/             sfunction_DH e funcoes do modelo NL (originais, sem alteracao)
%   ganhos_mirko/       Ganho_hold_*.mat originais + Dados_Trim.mat
%   ganhos_v3/          ganhos re-sintetizados (mesma estrutura) + relatorio de projeto
%
% Uso: abra esta pasta no MATLAB e rode  >> guiagem_NL
% Troque 'ganhos' abaixo entre 'mirko' e 'v3' para comparar. Caso 4 do artigo
% (Velocity Hold + psi Hold + H Hold) e' o modo padrao: a guiagem entrega psi_ref,
% h_ref e v_ref direto nos integradores dos holds.
close(findall(groot, 'Type', 'figure', 'Tag', 'guiagem_NL'));   % fecha so' as figuras de corridas anteriores (nao a GUI)

%% ===================== CONFIGURACAO =====================
% (cada variavel abaixo pode ser definida ANTES de rodar o script para sobrescrever o padrao)
if ~exist('ganhos', 'var'),    ganhos = 'mirko'; end   % 'mirko' (Ganho_hold_*.mat originais) | 'v3' (re-sintese, ganhos_v3/)
if ~exist('VT_missao', 'var'), VT_missao = 15;   end   % velocidade de engate e planta do gain scheduling: 12 | 15 | 18 m/s (600 m)
if ~exist('WPs', 'var')
    % waypoints [N E h V] no referencial do engate (o aviao parte da origem, proa 0, h 600 m, VT_missao)
    WPs = [ 256    0  600  15;      % circuito OVAL de 6 WPs (retas 256 m, pontas R 160 m, ~1417 m)
            416  160  600  15;
            256  320  600  15;
              0  320  600  15;
           -160  160  600  15;
              0    0  600  15 ];
    % WPs = [260 0 620 18; 260 260 600 15; 0 260 620 18; 0 0 600 15];   % circuito AGRESSIVO (90 deg, +-20 m, 15/18 m/s); use R_accept = 110
end
if ~exist('R_accept', 'var'), R_accept = 100; end      % raio de captura [m]
if ~exist('eng_tau', 'var'),  eng_tau = 0.3;  end      % lag do motor [s]; 3.5 reproduz o motor eletrico do X-Plane 9
T_max = 200;                                          % teto de simulacao [s] (para sozinha 5 s apos o ultimo WP)
VT_Throttle = 1; phi_psi = 0; att_alt = 0;             % Caso 4: Velocity Hold + psi Hold + H Hold
sat_deg = 15;  act_rate_deg = 150;  act_tau = 0.05;    % atuadores (a planta do artigo nao tem): curso, rate limit, servo

%% ===================== PATHS E DADOS =====================
here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here, 'planta'));
load(fullfile(here, 'ganhos_mirko', 'Dados_Trim.mat'));          % Plantas (1x9): A B C D Xe Ue por condicao
gd = fullfile(here, ['ganhos_' ganhos]);
for f = {'Ganho_hold_theta', 'Ganho_hold_H', 'Ganho_hold_VT', 'Ganho_hold_phi', 'Ganho_hold_psi'}
    load(fullfile(gd, [f{1} '.mat']));
end
i = find([Plantas.Ve] == VT_missao & [Plantas.He] == 600, 1);
assert(~isempty(i), 'nao ha planta para %g m/s @ 600 m (use 12, 15 ou 18)', VT_missao);

%% ===================== VARIAVEIS DO MODELO =====================
Ts = 1/100; surfaces = 24; Variacao_Iner = 0; coef_Ana = 1; coef_Sato = 0;
Xe_planta = Plantas(i).Xe; Xe_planta(12) = -600;                  % planta parte do trim da condicao i, a 600 m
U_trim = double(Plantas(i).Ue(1:4)); de_trim = U_trim(2); h0 = 600;
act.rate = deg2rad(act_rate_deg); act.tau = act_tau; act.bw = 1/act_tau;
eng.rate = 1.0; eng.tau = eng_tau; sat_surf_rad = deg2rad(sat_deg);
prot_on = 0; alpha_prot = deg2rad(16);                            % alpha-protection desligada (bloco inerte)
K_bank_guia = 0.1975; phi_max_guia = deg2rad(20);                 % so' usados no modo phi Hold (phi_psi = 1)
h_ref0 = h0; VT_ref0 = VT_missao;
N_WPs = size(WPs, 1); WPfim_N = WPs(end, 1); WPfim_E = WPs(end, 2);
xi_alt0 = double(Plantas(i).Xe(8)) / double(GintLong_Alt{i});     % CI do integrador de H: theta_ref(0) = theta de trim (engate sem transiente)

%% ===================== SIMULA =====================
mdl = 'CL_NL_DH_GUIA';
if bdIsLoaded(mdl), bdclose(mdl); end
load_system(fullfile(here, [mdl '.slx']));
fprintf('guiagem_NL: ganhos %s | planta %d (%s) | %d WPs, R_accept %g m | motor tau %g s, curso +-%g deg\n', ...
    ganhos, i, Plantas(i).nome, N_WPs, R_accept, eng_tau, sat_deg);
fprintf('engate (fixo): origem N 0, E 0, proa 0 (Norte), h 600 m, V_T %g m/s, em trim: theta %.2f deg, manete %.3f, de %+.2f deg\n', ...
    VT_missao, Plantas(i).Xe(8)*180/pi, U_trim(1), de_trim*180/pi);
t0 = tic; out = sim(mdl, 'StopTime', num2str(T_max)); t_cpu = toc(t0);

%% ===================== RESUMO =====================
Y = squeeze(out.Y_xp); if size(Y, 1) ~= 14, Y = Y.'; end          % canais: [VT p q r phi theta psi h beta t xN xE psi_abs alpha]
t = out.tout(:);
U = squeeze(out.U_xp); if size(U, 1) == 4 && size(U, 2) > 4, U = U.'; end
if size(U, 1) ~= numel(t), U = interp1(linspace(0, t(end), size(U, 1))', U, t, 'previous', 'extrap'); end   % log a 20 Hz -> base de t
R2D = 180/pi;
fprintf('\n===== resultado (%s, %.0f s de CPU) =====\n', ganhos, t_cpu);
fprintf('sim %.1f s | VT %.1f..%.1f | h %.1f..%.1f m | phi max %.1f deg | theta %.1f..%.1f | alpha %.1f..%.1f | de %+.1f..%+.1f deg | manete %.2f..%.2f\n', ...
    t(end), min(Y(1,:)), max(Y(1,:)), min(Y(8,:)), max(Y(8,:)), max(abs(Y(5,:)))*R2D, min(Y(6,:))*R2D, max(Y(6,:))*R2D, ...
    min(Y(14,:))*R2D, max(Y(14,:))*R2D, min(U(:,2))*R2D, max(U(:,2))*R2D, min(U(:,1)), max(U(:,1)));
n_hit = 0;
for k = 1:N_WPs
    dmin = min(sqrt((Y(11,:) - WPs(k,1)).^2 + (Y(12,:) - WPs(k,2)).^2));
    hit = dmin <= R_accept; n_hit = n_hit + hit;
    if hit, s = 'CAPTURADO'; else, s = '-- fora --'; end
    fprintf('WP%d (N %+7.1f, E %+7.1f, h %g, V %g): dist min %6.1f m  %s\n', k, WPs(k,:), dmin, s);
end
fprintf('CAPTURAS: %d/%d\n', n_hit, N_WPs);

%% ===================== FIGURAS =====================
rd = fullfile(here, 'resultados'); if ~exist(rd, 'dir'), mkdir(rd); end
arq = fullfile(rd, sprintf('guiagem_NL_%s_%s.png', ganhos, datestr(now, 'yyyymmdd_HHMMSS')));
plot_guiagem_NL(t, Y, U, WPs, R_accept, sprintf('LQRy ganhos %s, planta %d (%g m/s), Caso 4 + guiagem LOS: %d/%d capturas', ganhos, i, VT_missao, n_hit, N_WPs), arq);
save(strrep(arq, '.png', '.mat'), 't', 'Y', 'U', 'WPs', 'R_accept', 'ganhos', 'i', 'n_hit');
fprintf('figura e dados em %s\n', rd);
