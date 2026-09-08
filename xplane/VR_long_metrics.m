function M = VR_long_metrics(arq_xp, segs, rotulo)
%VR_LONG_METRICS Metricas longitudinais real x X-Plane por segmento de profundor.
%   M = VR_long_metrics('VR_replay_20260908_xxxxxx.mat', [2 16], 'D3')
% Por segmento: V0, trim do X-Plane (de0 em graus com o curso do .acf, thr0),
% pico de q XP/real (janela 1 s antes -> 4 s apos), dTheta local (ref. t=1,9 s,
% max em 2-3,2 s) real e XP, e ajuste 2a ordem q/de (vr_fit_q_de, 1,5-4,5 s)
% do REAL e do X-Plane com a MESMA entrada (Delta u real x 15 deg).
    xpDir = fileparts(mfilename('fullpath')); vd = fullfile(xpDir, 'voos');
    S = load(fullfile(vd, 'VR_segmentos.mat')); SEG = S.SEG; sg = S.sinais; T_PRE = S.T_PRE;
    X = load(fullfile(vd, arq_xp)); R = X.R; R2D = 180/pi;
    if nargin < 2 || isempty(segs), segs = sort([R.iseg]); end
    if nargin < 3, rotulo = arq_xp; end
    curso = 25; if isfield(X, 'lims'), curso = X.lims(1); end
    dth = @(t, th) max(th(t >= 2 & t <= 3.2)) - interp1(t, th, 1.9);
    fprintf('\n=== %s ===\n seg |  V0  | de0 XP [deg] | thr0 | empuxo N | pico q XP/real | dTheta real / XP [deg] | wn/zeta REAL | wn/zeta XP | fit%% XP\n', rotulo);
    M = [];
    for j = segs
      for k = find([R.iseg] == j)          % pode haver repeticoes do mesmo segmento
        s = SEG(j); r = R(k); if ~strcmp(s.eixo, 'elev'), continue; end
        thr_N = NaN; if isfield(r, 'thrust0'), thr_N = r.thrust0; end
        tg = s.t(:); idy = tg > T_PRE-1 & tg < T_PRE+4;
        yr = R2D*s.q(:); yx = interp1(r.t, R2D*r.q, tg);
        pk = max(abs(yx(idy))) / max(abs(yr(idy)));
        de_real = sg.elev*(s.u(:,2) - s.u_trim(2))*15;
        fr = vr_fit_q_de(tg, de_real, yr, [1.5 4.5]);
        fx = vr_fit_q_de(r.t, interp1(tg, de_real, r.t, 'previous', 0), R2D*r.q, [1.5 4.5]);
        row = [j, s.V0, r.de0*curso, r.thr0, pk, dth(tg, s.theta), dth(r.t, r.theta), fr(1), fr(2), fx(1), fx(2), fx(6)];
        M(end+1, :) = row; %#ok<AGROW>
        fprintf(' %3d | %4.1f | %+7.1f      | %.2f | %6.2f   | %5.2f          | %+5.1f / %+5.1f          | %5.2f / %4.2f | %5.2f / %4.2f | %3.0f\n', ...
            j, s.V0, row(3), r.thr0, thr_N, pk, row(6), row(7), fr(1), fr(2), fx(1), fx(2), fx(6));
      end
    end
    if size(M, 1) >= 2
        p = polyfit(M(:,2), M(:,3), 1);
        fprintf(' medias: pico q %.2f | dTheta XP/real %.2f | trim de0: %+.1f deg/(m/s) (real ~0)\n', mean(M(:,5)), mean(M(:,7))/mean(M(:,6)), p(1));
    end
end
