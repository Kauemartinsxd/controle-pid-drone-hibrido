function T = lqry_v3_vt_fronteira(varargin)
% LQRY_V3_VT_FRONTEIRA  Quanto do pulso de V_T do artigo (+3 m/s por 10 s) cada conjunto
% de ganhos consegue seguir, contra a margem de fase da malha de manete com o motor do
% X-Plane (tau 3,5 s) e com um motor rapido (0,3 s). Linear, planta 5 (15 m/s), theta Hold
% + Alt Hold + Vel Hold FECHADOS. Varre pesos (lim.thr, Ti.VT) da re-sintese v3.
%
%   T = lqry_v3_vt_fronteira                      % original, v3 entregue e 6 candidatos
%   T = lqry_v3_vt_fronteira('cand', {45,2.5; 80,1.5})
p = inputParser;
p.addParameter('cand', {45,2.5; 60,2.0; 80,1.5; 100,1.2; 150,1.0; 250,0.8});
p.addParameter('i', 5);
p.parse(varargin{:}); o = p.Results;
here = fileparts(mfilename('fullpath')); addpath(here);
raizN = 'C:\Users\kaue\Documents\Dissertacao_Mestrado\lqry_mirko_atualizado\Nova pasta';
Sv = load(fullfile(raizN,'Ganho_hold_VT.mat')); St = load(fullfile(raizN,'Ganho_hold_theta.mat')); Sa = load(fullfile(raizN,'Ganho_hold_H.mat'));
R0 = lqry_v3_projeto('iset', o.i, 'salvar', false, 'verbose', false); P = R0.planta(o.i);
Al = P.long.Al; Bthr = P.long.Bl_thr; Bde = P.long.Bl_de;
T = struct([]);
fprintf('%-34s %8s %8s | %10s %8s | %8s %8s\n', 'conjunto', 'Kp', 'Ki', 'pulso10s', 't90[s]', 'PM 3.5s', 'PM 0.3s');
i = o.i;
r = avalia(Al, Bthr, Bde, double(St.GstateLong{i}), double(St.GintLong{i}), double(Sa.GstateLong_Alt{i}), double(Sa.GintLong_Alt{i}), double(Sv.GstateLong_speed{i}), double(Sv.GintLong_speed{i}));
T = linha(T, 'ORIGINAL (Mirko)', abs(double(Sv.GstateLong_speed{i}(1))), abs(double(Sv.GintLong_speed{i})), r);
K = P.K; r = avalia(Al, Bthr, Bde, -K.theta(1:5), -K.theta(6), -K.alt(1:5), -K.alt(6), -K.vel(1:4), -K.vel(5));
T = linha(T, 'v3 entregue (thr 25 %, Ti 8 s)', abs(K.vel(1)), abs(K.vel(5)), r);
for c = 1:size(o.cand, 1)
    Rc = lqry_v3_projeto('iset', i, 'salvar', false, 'verbose', false, 'lim', struct('thr', o.cand{c,1}), 'Ti', struct('VT', o.cand{c,2}));
    K = Rc.planta(i).K; r = avalia(Al, Bthr, Bde, -K.theta(1:5), -K.theta(6), -K.alt(1:5), -K.alt(6), -K.vel(1:4), -K.vel(5));
    T = linha(T, sprintf('v3 thr %g %%, Ti %g s', o.cand{c,1}, o.cand{c,2}), abs(K.vel(1)), abs(K.vel(5)), r);
end
fprintf('\npulso10s = fracao do degrau de +3 m/s atingida durante o pulso de 10 s do artigo (motor 0,1 s);\nt90 = degrau mantido (motor 0,1 s); PM = margem de fase da manete na entrada da planta, Alt Hold fechado\n');
end

function T = linha(T, nome, Kp, Ki, r)
fprintf('%-34s %8.1f %8.2f | %9.0f%% %8.1f | %8.1f %8.1f\n', nome, Kp, Ki, r.pk, r.t90, r.PM35, r.PM03);
T(end+1).nome = nome; T(end).Kp = Kp; T(end).Ki = Ki; T(end).pulso_pct = r.pk; T(end).t90 = r.t90; T(end).PM35 = r.PM35; T(end).PM03 = r.PM03;
end

function r = avalia(Al, Bthr, Bde, Gs, Gi, Ga, Gia, Gv, Giv)
[A, B] = malha(Al, Bthr, Bde, Gs, Gi, Ga, Gia, Gv, Giv, 0.1);
sys = ss(A, B, [1 zeros(1,10)], 0);
t = (0:0.05:120)'; y = step(sys*3, t); k = find(y >= 2.7, 1); if isempty(k), r.t90 = NaN; else, r.t90 = t(k); end
u = 3*double(t < 10); yp = lsim(sys, u, t); r.pk = 100*max(yp)/3;
tms = [3.5 0.3]; PM = zeros(1,2);
for k = 1:2
    [A, ~] = malha(Al, Bthr, Bde, Gs, Gi, Ga, Gia, Gv, Giv, tms(k));
    Aol = A; Aol(1:5,10) = 0; Bin = zeros(11,1); Bin(1:5) = Bthr/100; Cout = zeros(1,11); Cout(10) = 1;
    [~, PM(k)] = margin(-ss(Aol, Bin, Cout, 0));
end
r.PM35 = PM(1); r.PM03 = PM(2);
end

function [A, B] = malha(Al, Bthr, Bde, Gs, Gi, Ga, Gia, Gv, Giv, tm)
% estados: [dVT dalpha q dtheta dH | de | xi_th | xi_H | thr_lag | T_motor | xi_V]
surf = 24; tau_thr = 0.1;
n = 11; A = zeros(n); B = zeros(n,1);
A(1:5,1:5) = Al; A(1:5,6) = Bde; A(1:5,10) = Bthr/100;
A(6,[1:4 6]) = surf*Gs; A(6,6) = A(6,6) - surf; A(6,7) = surf*Gi;     % theta Hold
A(7,1:5) = -Ga; A(7,4) = A(7,4) + 1; A(7,8) = -Gia;                    % xi_th' = theta - theta_ref (Alt Hold)
A(8,5) = 1;                                                            % xi_H' = H
A(9,1:4) = Gv/tau_thr; A(9,11) = Giv/tau_thr; A(9,9) = -1/tau_thr;     % lag da manete <- Vel Hold
A(10,9) = 1/tm; A(10,10) = -1/tm;                                      % motor
A(11,1) = 1; B(11) = -1;                                               % xi_V' = VT - VT_ref
end
