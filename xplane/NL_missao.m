% NL_missao.m
% =============================================================
% MISSAO POR WAYPOINTS DO DH EM SIMULACAO PURA (SEM X-Plane):
% mesma guiagem LOS e mesmo controlador PID do XP_missao, mas a
% planta e' o modelo nao linear da Ana (sfunction_DH) — o
% modelo_NL_DH_GUIA.slx e' o XP_missao com a Planta_XP trocada.
%
% Roda em segundos, nao precisa do X-Plane aberto (plano B da demo)
% e salva o voo no MESMO formato do XP_missao (plot_XP_missao
% funciona sem mudanca). Engate = origem, proa 0, h 600, VT 12.
%
% Uso:
%   NL_missao                  % default: circuito oval 280x160 m (= XP_missao)
% Config opcional (defina antes):
%   NL_WPs      [Nx4]  [a_frente(m) a_direita(m) hMSL(m) vel(m/s)]
%                      (no SIL frame do engate == NE)
%   NL_R_accept [m]    raio de aceitacao (default 80)
%   NL_TimeXP   [s]    duracao (default: perimetro/12 x 1.5 + 15)
%   NL_tag      [char] sufixo do .mat salvo
% =============================================================

%% Config (preservada atraves do clear do DH_inicializacao)
if ~exist('NL_R_accept','var') || isempty(NL_R_accept), NL_R_accept = 60; end  % 60 p/ o oval de 6 WPs (80 ate 2026-08-31)
if ~exist('NL_WPs','var'),    NL_WPs = []; end
if ~exist('NL_TimeXP','var'), NL_TimeXP = []; end
if ~exist('NL_tag','var'),    NL_tag = 'SILmissao'; end
if isempty(NL_WPs)
    % espelho do default do XP_missao (oval 6 WPs: cabe nos ~130 s de motor)
    NL_WPs = [ 160    0  600  12;
               260  100  600  12;
               160  200  600  12;
                 0  200  600  12;
              -100  100  600  12;
                 0    0  600  12];
end
setpref('XP_DH','nlmissao', struct('R',NL_R_accept,'WP',NL_WPs,'T',NL_TimeXP,'tag',NL_tag));

%% 1) Workspace da dissertacao (ganhos, trim) — faz clear/bdclose
xpDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(xpDir);
run(fullfile(rootDir, 'DH_inicializacao.m'));
xpDir = fileparts(mfilename('fullpath'));   % re-cria pos-clear
rootDir = fileparts(xpDir);
addpath(xpDir);
addpath(fullfile(rootDir,'nao_linear'));    % sfunction_DH DA DISSERTACAO na frente
addpath(fullfile(rootDir,'utilitarios'));

cfgm = getpref('XP_DH','nlmissao');
rmpref('XP_DH','nlmissao');

%% 2) Guiagem: WPs em NE do engate (origem), interpolacao de Dh>20 m
WPs_user = cfgm.WP;
WPs = WPs_user(1,:);
for i = 2:size(WPs_user,1)
    dalt = abs(WPs_user(i,3) - WPs_user(i-1,3));
    if dalt > 20
        n_sub = ceil(dalt/20);
        for k = 1:n_sub-1
            WPs(end+1,:) = WPs_user(i-1,:) + (k/n_sub)*(WPs_user(i,:) - WPs_user(i-1,:)); %#ok<SAGROW>
        end
    end
    WPs(end+1,:) = WPs_user(i,:); %#ok<SAGROW>
end
R_accept = cfgm.R;
N_WPs = size(WPs,1);   % p/ o corte de fim de missao no modelo (Cond_fim_missao)
WPfim_N = WPs(end,1);  WPfim_E = WPs(end,2);   % ultimo WP (dist independente do dist_mon)
h_ref  = he;  VT_ref = Ve;
h_ref0 = h_ref; VT_ref0 = VT_ref;          % base dos deltas do Guidance_Star
% step de validacao de theta do modelo GUIA (INERTE aqui)
theta_test_t = 1e9; theta_test_init = 0; theta_test_final = 0;
seg = [0 0; WPs(:,1:2)];        % engate (origem) -> WP1 -> ... (= XP_missao)
per = sum(sqrt(sum(diff(seg).^2,2)));
TimeXP = cfgm.T;
if isempty(TimeXP), TimeXP = ceil(per/12*1.5 + 15); end
fprintf('NL_missao: %d WPs (%d apos interpolacao), perimetro %.0f m, %.0f s.\n', ...
    size(WPs_user,1), size(WPs,1), per, TimeXP);

%% 3) Simula (segundos — sem pacing, sem X-Plane)
mdl = 'modelo_NL_DH_GUIA';
load_system(fullfile(xpDir,[mdl '.slx']));
out = sim(mdl, 'StopTime', num2str(TimeXP));

%% 4) Salva no formato do XP_missao (plot_XP_missao compativel)
voosDir = fullfile(xpDir, 'voos');
if ~isfolder(voosDir), mkdir(voosDir); end
voo = struct();
voo.quando   = datestr(now, 'yyyy-mm-dd HH:MM:SS');
voo.Y        = squeeze(out.Y_xp);
if size(voo.Y,1) ~= 14, voo.Y = voo.Y.'; end
voo.U        = squeeze(out.U_xp);
if size(voo.U,2) ~= 4 && size(voo.U,1) == 4, voo.U = voo.U.'; end
voo.t        = out.tout(:);
voo.t_xplane = voo.Y(10,:).';               % canal t = tempo da sim
voo.wp_idx   = out.wp_idx_log(:);
voo.dist_wp  = out.dist_log(:);
voo.WPs      = WPs;
voo.WPs_user = WPs_user;
voo.R_accept = R_accept;
voo.psi_engate = 0;
voo.cfg = struct('planta','NL (modelo da Ana, sfunction_DH)', ...
    'TimeXP',TimeXP, 'h_ref',h_ref, 'VT_ref',VT_ref, ...
    'C_theta',C_theta, 'C_vel',C_vel, 'C_alt',C_alt, 'C_phi',C_phi, ...
    'theta_ref_clamp',theta_ref_clamp);
vooFile = fullfile(voosDir, ['NL_missao_' datestr(now,'yyyymmdd_HHMMSS') '_' cfgm.tag '.mat']);
save(vooFile, 'voo');
fprintf('Missao SIL salva em: %s\n', vooFile);

%% 5) Resumo + figuras (mesmo pos-processamento do XP_missao)
Y = voo.Y; t = voo.t;
nWP = size(WPs,1);
fprintf('\n===== NL_missao (SIL): resumo =====\n');
fprintf('VT: min %.1f | max %.1f | h: min %.1f | max %.1f\n', ...
    min(Y(1,:)), max(Y(1,:)), min(Y(8,:)), max(Y(8,:)));
fprintf('phi max |%.1f| deg | theta %.1f..%.1f deg\n', ...
    rad2deg(max(abs(Y(5,:)))), rad2deg(min(Y(6,:))), rad2deg(max(Y(6,:))));
for k = 1:nWP
    d = sqrt((WPs(k,1)-Y(11,:)).^2 + (WPs(k,2)-Y(12,:)).^2);
    dmin = min(d);
    hit = dmin <= R_accept;
    lab = '-- fora --'; if hit, lab = 'CAPTURADO'; end
    fprintf('WP%d (N %+7.1f, E %+7.1f): dist min %6.1f m  %s\n', k, WPs(k,1), WPs(k,2), dmin, lab);
end
fprintf('wp_idx final: %d de %d\n', voo.wp_idx(end), nWP);
try
    plot_XP_missao(voo, vooFile);
catch e
    fprintf('plot falhou: %s\n', e.message);
end
