function res = XP_fit_ident_wn(identFile, Tw)
% XP_FIT_IDENT_WN  Ajuste 2a ordem q/de na resposta ANTISSIMETRICA dos degraus
% de XP_ident_theta (mesma receita da varredura de raios de 2026-08-30:
% (run+a - run-a)/2, janela curta Tw [s, default 0.9], cap wn <= 25 rad/s).
% res = [amp wn zeta zero K rms] por amplitude. Referencias do gemeo:
% v1 (raios auto) 8.33/0.68 | v1.1 (pitch 1.47 ft) 6.25/0.61.
if nargin < 2 || isempty(Tw), Tw = 0.9; end
S = load(identFile); runs = S.runs; R2D = 180/pi; tg = (0:0.05:Tw)'; res = [];
amps = unique(abs([runs.step_deg])); amps = amps(amps > 0);
for amp = amps
    ip = find([runs.step_deg] == amp, 1); im = find([runs.step_deg] == -amp, 1);
    if isempty(ip) || isempty(im), continue; end
    q = @(r) interp1(runs(r).D(:,1) - runs(r).D(1,1), runs(r).D(:,2), tg, 'linear', 'extrap');
    dq = (q(ip) - q(im))/2 * R2D;                 % deg/s por degrau de +amp deg
    fun = @(p) sum((step_resp(p, tg) - dq).^2);
    best = []; bestJ = inf;
    for wn0 = [4 6 8 12], for z0 = [0.4 0.7], for a0 = [2 5]
        p0 = [max(abs(dq))/amp, a0, wn0, z0];
        [p, J] = fminsearch(fun, p0, optimset('Display', 'off', 'MaxFunEvals', 4000, 'MaxIter', 4000));
        if p(3) > 0 && p(3) <= 25 && p(4) > 0 && J < bestJ, bestJ = J; best = p; end
    end, end, end
    fprintf('degrau +-%g deg: wn %.2f rad/s, zeta %.2f (zero %.2f, K %.2f) | pico dq %.1f deg/s | RMS %.2f deg/s\n', ...
        amp, best(3), best(4), best(2), best(1), max(abs(dq)), sqrt(bestJ/numel(tg)));
    res = [res; amp, best(3), best(4), best(2), best(1), sqrt(bestJ/numel(tg))]; %#ok<AGROW>
end
end

function y = step_resp(p, t)
K = p(1); a = p(2); wn = p(3); z = p(4);
s = tf('s'); G = K*(s + a)/(s^2 + 2*z*wn*s + wn^2); y = step(G, t); y = y(:);
end
