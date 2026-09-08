% VR_plot_sato_real.m
% =============================================================
% Figura ESTILO SATO (Fig. 6.13 da dissertacao do Sato) para o
% doublet REAL de profundor: mesmas entradas do voo real de dez/24
% (seg 2, V0=18,8 m/s) aplicadas em:
%
%   - Medido: o proprio voo real (ArduPilot, modo MANUAL);
%   - Modelo da Ana (nao linear, dyn_rigidbody_DH) trimado na
%     VELOCIDADE REAL e excitado em modo delta (analogo exato do
%     "M1 Identificado" do Sato — modelo identificado de voo);
%   - X-Plane gemeo v1.2 (campanha VR_replay_GEMEO_MERGED, replay
%     delta em malha aberta).
%
% Mesmas convencoes visuais do Sato: valores ABSOLUTOS, eixos
% largos (VT 0-20, theta +-50, q +-200, de +-20), preto=medido,
% vermelho=modelo, azul tracejado=X-Plane.
%
% Saida: voos/VR_sato_doublet_real.png
% =============================================================

xpDir = fileparts(mfilename('fullpath'));
repo  = fileparts(xpDir);
addpath(fullfile(repo, 'utilitarios'));
voosDir = fullfile(xpDir, 'voos');

% Config opcional (defina antes de rodar): VR_sato_campanha (arquivo da
% campanha XP), VR_sato_rotulo (legenda do X-Plane), VR_sato_png (saida)
if ~exist('VR_sato_campanha','var') || isempty(VR_sato_campanha)
    VR_sato_campanha = 'VR_replay_GEMEO_MERGED.mat';
    VR_sato_rotulo   = 'X-Plane (gemeo v1.2)';
    VR_sato_png      = 'VR_sato_doublet_real.png';
end

JSEG = 2;                                  % doublet de profundor (long, t0=328 s)
S  = load(fullfile(voosDir, 'VR_segmentos.mat'));
G  = load(fullfile(voosDir, VR_sato_campanha));
s  = S.SEG(JSEG);
rg = G.R([G.R.iseg] == JSEG);

% ---- entradas reais (convencao X-Plane, delta do trim da janela) ----
sgnE   = S.sinais.elev;
de_real_deg  = sgnE * s.u(:,2) * 15;             % profundor real [deg] (curso .acf 15)
dde_real_deg = de_real_deg - sgnE*s.u_trim(2)*15;% delta p/ os modelos
dthr_real    = s.u(:,3) - s.u_trim(3);           % delta de manete [0,1]

% ---- modelo da Ana: trim na VELOCIDADE REAL + replay delta ----
V0 = s.V0;
[Xe, Ue] = local_trim(V0, 600, 0);
fprintf('Trim Ana a %.1f m/s: alpha %.2f deg | de %+.2f deg | thr %.3f\n', ...
    V0, rad2deg(atan(Xe(3)/Xe(1))), rad2deg(Ue(2)), Ue(1));
fde  = @(t) interp1(s.t, dde_real_deg, t, 'previous', 0);
fthr = @(t) interp1(s.t, dthr_real,   t, 'previous', 0);
Ureal = @(t) Ue + [fthr(t); deg2rad(fde(t)); 0; 0; 0; 0; 0];
[tnl, Xnl] = ode45(@(t,X) dyn_rigidbody_DH(t, X, Ureal(t)), (0:0.02:s.t(end))', Xe);
VTnl = sqrt(Xnl(:,1).^2 + Xnl(:,3).^2);
qnl  = rad2deg(Xnl(:,5));
thnl = rad2deg(Xnl(:,8));
de_ana_deg = rad2deg(Ue(2)) + fde(tnl);

% ---- X-Plane gemeo: absolutos direto da campanha ----
de_xp_deg = rg.u_sent(:,2) * 15;                 % comando enviado [deg]

% ---- figura ----
f = figure('Color','w','Position',[60 30 900 1000], 'Visible','off');
try, f.Theme = 'light'; catch, end
tiledlayout(4,1,'TileSpacing','compact','Padding','compact');
cM = [0 0 0]; cA = [0.85 0.1 0.1]; cX = [0 0.35 0.85];

nexttile; hold on; grid on;
plot(s.t,  s.V,  '-',  'Color', cM, 'LineWidth', 1.5);
plot(tnl,  VTnl, '-',  'Color', cA, 'LineWidth', 1.3);
plot(rg.t, rg.V, '--', 'Color', cX, 'LineWidth', 1.4);
ylabel('V_T [m/s]'); ylim([0 25]); xlim([0 s.t(end)]);
title(sprintf(['Doublet REAL de profundor (voo dez/24, V_0 = %.1f m/s) — ' ...
    'mesmas entradas nos tres mundos'], V0));
legend({'Medido (voo real)', 'Modelo da Ana (identificado)', VR_sato_rotulo}, ...
    'Location', 'southeast');

nexttile; hold on; grid on;
plot(s.t,  s.theta, '-',  'Color', cM, 'LineWidth', 1.5);
plot(tnl,  thnl,    '-',  'Color', cA, 'LineWidth', 1.3);
plot(rg.t, rg.theta,'--', 'Color', cX, 'LineWidth', 1.4);
ylabel('\theta [{\circ}]'); ylim([-50 50]); xlim([0 s.t(end)]);

nexttile; hold on; grid on;
plot(s.t,  rad2deg(s.q), '-',  'Color', cM, 'LineWidth', 1.5);
plot(tnl,  qnl,          '-',  'Color', cA, 'LineWidth', 1.3);
plot(rg.t, rad2deg(rg.q),'--', 'Color', cX, 'LineWidth', 1.4);
ylabel('q [{\circ}/s]'); ylim([-200 200]); xlim([0 s.t(end)]);

nexttile; hold on; grid on;
stairs(s.t, de_real_deg, '-',  'Color', cM, 'LineWidth', 1.5);
plot(tnl,   de_ana_deg,  '-',  'Color', cA, 'LineWidth', 1.1);
stairs(rg.u_sent(:,1), de_xp_deg, '--', 'Color', cX, 'LineWidth', 1.1);
ylabel('\delta_e [{\circ}]'); ylim([-20 20]); xlim([0 s.t(end)]); xlabel('t [s]');
legend({'\delta_e real (PWM\rightarrowgraus)', ...
    '\delta_e modelo (trim proprio + \Delta real)', ...
    '\delta_e X-Plane (trim proprio + \Delta real)'}, 'Location', 'southeast');

fn = fullfile(voosDir, VR_sato_png);
exportgraphics(f, fn, 'Resolution', 125); close(f);
fprintf('Figura salva: %s\n', fn);
clear VR_sato_campanha VR_sato_rotulo VR_sato_png

%% -----------------
function [Xe, Ue] = local_trim(Ve, he, gammae)
    montaX = @(y) [Ve*cos(y(1)); 0; Ve*sin(y(1)); 0; 0; 0; 0; ...
                   y(1)+gammae; 0; 0; 0; -he; 0; 0];
    montaU = @(y) [y(2); y(3); 0; 0; 0; 0; 0];
    resid  = @(y) subsref(dyn_rigidbody_DH(0, montaX(y), montaU(y)), ...
                          struct('type','()','subs',{{[1 3 5]}}));
    J = @(y) sum(resid(y).^2);
    y0 = [deg2rad(6), 0.5, deg2rad(3)];    % chute p/ V alto (alpha ~ 14.4*(12/V)^2)
    opt = optimset('MaxFunEvals',5e4,'MaxIter',5e4,'TolFun',1e-16,'TolX',1e-14);
    y  = fminsearch(J, y0, opt);
    y  = fminsearch(J, y, opt);
    fprintf('Trim fminsearch: residuo J = %.3e\n', J(y));
    Xe = montaX(y); Ue = montaU(y);
end
