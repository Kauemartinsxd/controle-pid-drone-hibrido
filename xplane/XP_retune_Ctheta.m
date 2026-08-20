% XP_retune_Ctheta.m
% =============================================================
% FASE 0 (PLANO_GUIAGEM) — retune do C_theta para a planta do
% X-Plane (.acf v3). Reproduz o pipeline completo a partir dos
% dados de XP_ident_theta.m (2026-08-19/20):
%
%   1) Ajuste 2a ordem q/de por fminsearch sobre (corrida - baseline),
%      janela 0-0.9 s (apos isso o fugoide — fortemente excitado pelo
%      degrau — reverte o sinal de q e um 2a ordem nao captura).
%      Resultado: G_q = (97.0 s + 137.4)/(s^2 + 21.27 s + 95.9)
%      (polos reais -6.5/-14.8, K_dc q/de = 1.43 1/s).
%   2) Planta de theta com o damper: G_th = [Gq/(1+Kq*Gq)]/s, Kq=0.10.
%   3) Projeto: o pidtune PI (wc 2-4, PM 75-85, qualquer focus) fica
%      sempre com OS 15-19% — o zero Ki/Kp perto do crossover e' quem
%      poe o overshoot (mesma assinatura do C_theta da dissertacao
%      nesta planta: OS 14.9%). O pidtune PIDF converge p/ Kd<0 com
%      N~3 (lag-lead disfarcado) — descartado por robustez frente a
%      incerteza do fit. Escolha: PI "quase-P" com zero bem abaixo
%      do crossover (Ki/Kp = 0.56 rad/s << wc 4.4):
%
%        C_theta_XP: Kp = 1.6, Ki = 0.9, Kd = 0   (PI, como o original)
%
%      Previsto na planta identificada: OS 8.0%, PM 98 deg, wc 4.4
%      rad/s, GM inf; rejeicao de degrau de 5 deg no profundor:
%      pico 2.1 deg / recupera <0.5 deg em 3.7 s (esse Ki absorve o
%      erro TrimInput vs trim real do .acf sem depender do I lento
%      do C_alt).
%   4) Validacao em voo (XP_missao com theta_test): degrau de
%      theta_ref +-3..5 deg com OS < 10% e sem ciclo de estol com o
%      clamp reaberto.
%
% Contexto (diagnostico 2026-08-20): o equilibrio real do .acf v3 a
% VT 12 e' theta ~5.5 deg / thr ~0.85 (voo asa5). Com Xe(8)=2 e teto
% do clamp +3, theta_ref max = 5 deg fica NO limite e o I lentissimo
% do C_alt (Ki 0.0043) leva >10 min p/ compensar o centro errado —
% origem do offset de h e da queda p/ o 2o regime. Por isso o
% XP_missao usa Xe(8)=5 deg e clamp [-10 +1.5] (theta_ref max 6.5,
% protecao de energia: theta > ~7 deg -> arrasto > empuxo).
% =============================================================

scratch = fileparts(mfilename('fullpath'));
identFile = fullfile(scratch, 'voos', 'XP_ident_theta_20260819_223117.mat');
S = load(identFile);

%% 1) Ajuste 2a ordem (janela do curto periodo)
D0 = S.runs(1).D;                       % baseline (degrau 0)
t_grid = (0:0.05:0.9)';
qb = interp1(D0(:,1), D0(:,2), t_grid, 'linear', 'extrap');
DS = {};
for ir = 2:numel(S.runs)
    D = S.runs(ir).D;
    qi = interp1(D(:,1), D(:,2), t_grid, 'linear', 'extrap');
    DS{end+1} = struct('dq', qi - qb, 'dde', deg2rad(S.runs(ir).step_deg)); %#ok<AGROW>
end
cost = @(p) local_cost(p, DS, t_grid);
opt = optimset('MaxFunEvals', 2e4, 'MaxIter', 2e4, 'TolFun', 1e-10, 'TolX', 1e-10);
p0 = [0.5, 5, 3, 25, 0.05];
[pb, J] = fminsearch(cost, p0, opt);
rng(7);
for k = 1:8
    [pt, Jt] = fminsearch(cost, p0 .* (0.3 + 1.4*rand(1,5)), opt);
    if Jt < J, pb = pt; J = Jt; end
end
Gq = tf([pb(1) pb(2)], [1 abs(pb(3)) abs(pb(4))]);
fprintf('G_q identificada: '); Gq %#ok<NOPTS>

%% 2-3) Planta de theta e controlador escolhido
Kq  = 0.10;                             % damper (inalterado)
Gth = feedback(Gq, Kq) * tf(1, [1 0]);

C_theta_XP.Kp = 1.6;
C_theta_XP.Ki = 0.9;
C_theta_XP.Kd = 0;
C_theta_XP.N  = 100;

C = pid(C_theta_XP.Kp, C_theta_XP.Ki);
T = feedback(C*Gth, 1);
si = stepinfo(T);
[gm, pm, wgm, wpm] = margin(C*Gth);
fprintf('C_theta_XP (Kp %.2f, Ki %.2f): OS %.1f%% | ts %.1f s | PM %.0f deg @ %.1f rad/s\n', ...
    C_theta_XP.Kp, C_theta_XP.Ki, si.Overshoot, si.SettlingTime, pm, wpm);

% comparacao com o C_theta da dissertacao na MESMA planta
C0 = pid(0.917, 0.857);
si0 = stepinfo(feedback(C0*Gth, 1));
fprintf('C_theta dissertacao na planta XP: OS %.1f%% | ts %.1f s  (motivo do retune)\n', ...
    si0.Overshoot, si0.SettlingTime);

function J = local_cost(p, DS, t)
    G = tf([p(1) p(2)], [1 abs(p(3)) abs(p(4))]);
    td = min(max(p(5), 0), 0.15);
    J = 0;
    for i = 1:numel(DS)
        u = DS{i}.dde * (t >= td);
        J = J + sum((lsim(G, u, t) - DS{i}.dq).^2);
    end
end
