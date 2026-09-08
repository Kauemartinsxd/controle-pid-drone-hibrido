function res = vr_fit_q_de(t, de_deg, q_degs, tw)
%VR_FIT_Q_DE Ajuste de 2a ordem q/delta_e por lsim a um doublet qualquer.
%   res = vr_fit_q_de(t, de_deg, q_degs, [t1 t2])
% Modelo G(s) = K (s + a) / (s^2 + 2 z wn s + wn^2) (mesma forma do
% XP_fit_ident_wn), entrada delta_e [deg] com ZOH, saida q [deg/s]; nivel
% pre-janela removido dos dois. Multi-start + cap wn <= 25 rad/s.
% res = [wn zeta a K rms fit%] (fit% = 100(1-||e||/||q-mean||)).
    ws = warning('off', 'Control:analysis:LsimUndersampled'); cleanup = onCleanup(@() warning(ws));
    m = t >= tw(1) & t <= tw(2) & ~isnan(q_degs) & ~isnan(de_deg);
    tt = t(m); tt = tt - tt(1); u = de_deg(m); y = q_degs(m);
    npre = max(3, round(0.3/median(diff(tt))));
    u = u - mean(u(1:npre)); y = y - mean(y(1:npre));
    tg = (0:0.02:tt(end))'; ug = interp1(tt, u, tg, 'previous', 0); yg = interp1(tt, y, tg, 'linear', 'extrap');
    fun = @(p) sum((resp(p, tg, ug) - yg).^2);
    best = []; bestJ = inf;
    for wn0 = [4 6 9 13], for z0 = [0.3 0.6 0.9], for a0 = [1 4]
        p0 = [max(abs(yg))/max(abs(ug)+eps)/wn0, a0, wn0, z0];
        [p, J] = fminsearch(fun, p0, optimset('Display','off','MaxFunEvals',3000,'MaxIter',3000));
        if p(3) > 0 && p(3) <= 25 && p(4) > 0 && p(4) < 3 && J < bestJ, bestJ = J; best = p; end
    end, end, end
    e = resp(best, tg, ug) - yg;
    res = [best(3), best(4), best(2), best(1), sqrt(bestJ/numel(tg)), 100*(1 - norm(e)/norm(yg - mean(yg)))];
end

function y = resp(p, t, u)
    K = p(1); a = p(2); wn = p(3); z = p(4);
    s = tf('s'); G = K*(s + a)/(s^2 + 2*z*wn*s + wn^2);
    y = lsim(G, u, t); y = y(:);
end
