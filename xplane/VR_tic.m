function T = VR_tic(arq_xp, segs, win, rotulo, sg_override)
%VR_TIC Coeficiente de desigualdade de Theil, voo real x modelo NL (Ana) e
% voo real x X-Plane, por segmento e sinal, na janela win (default [1 6] s =
% 1 s antes ate 4 s apos o doublet, como o VR_plot_comp).
%
%   T = VR_tic('VR_replay_GEMEO_V13LAT_MERGED.mat', [], [1 6], 'v1.3-lat')
%   T = VR_tic(arq, segs, win, rotulo, sinais_antigos)   % NL com outros sinais
%
% TIC = rms(z - y) / (rms(z) + rms(y)), z = real, y = modelo. 0 = identico,
% 1 = sem relacao. Sinais: profundor -> dVT, dtheta, q; aileron -> p, dphi;
% leme -> r, dphi. "d" = nivel pre-doublet removido (media em t < T_PRE-0,5 s)
% — sem isso o TIC absoluto de VT sai ~0,03 pela media (~17 m/s) e o de
% theta carrega o offset de trim, nao a dinamica.
%
% Modelo NL: trim da Ana em (V0 real, 600 m, gamma 0) por fminsearch e replay
% delta das QUATRO entradas reais (manete, profundor, aileron, leme), ZOH a
% 25 Hz, como o X-Plane recebe. Aileron com sinal oposto (Cl_da < 0 no
% modelo da Ana). Sinais servo->convencao vindos de VR_segmentos.mat (leme +1
% desde 2026-09-08).
    xpDir = fileparts(mfilename('fullpath')); vd = fullfile(xpDir, 'voos');
    addpath(fullfile(fileparts(xpDir), 'utilitarios'));
    assert(contains(which('dyn_rigidbody_DH'), 'utilitarios'), 'VR_tic: dyn_rigidbody_DH errado no path (esperado utilitarios/)');
    S = load(fullfile(vd, 'VR_segmentos.mat')); SEG = S.SEG; sg = S.sinais; T_PRE = S.T_PRE;
    if nargin >= 5 && ~isempty(sg_override), sg = sg_override; end
    X = load(fullfile(vd, arq_xp)); R = X.R;
    if nargin < 2 || isempty(segs), segs = sort([R.iseg]); end
    if nargin < 3 || isempty(win), win = [1 6]; end
    if nargin < 4 || isempty(rotulo), rotulo = arq_xp; end
    R2D = 180/pi;
    ticf = @(z, y) sqrt(mean((z - y).^2)) / (sqrt(mean(z.^2)) + sqrt(mean(y.^2)));
    rows = []; ax = {};
    fprintf('\n=== TIC real x NL | real x X-Plane — %s — janela %g-%g s (sinais: ail %+d elev %+d rudd %+d) ===\n', rotulo, win, sg.ail, sg.elev, sg.rudd);
    fprintf(' seg | eixo |  sinal1: NL | XP  |  sinal2: NL | XP  |  sinal3: NL | XP\n');
    for j = segs
        k = find([R.iseg] == j); if isempty(k), continue; end
        s = SEG(j); r = R(k); t = s.t(:);
        m = t >= win(1) & t <= win(2); ipre = t < T_PRE - 0.5;
        % ---- modelo NL da Ana: trim em V0 + replay delta das 4 entradas ----
        dthr = s.u(:,3) - s.u_trim(3);
        dde = sg.elev*(s.u(:,2) - s.u_trim(2))*15;
        dda = sg.ail *(s.u(:,1) - s.u_trim(1))*15;
        ddr = sg.rudd*(s.u(:,4) - s.u_trim(4))*15;
        [Xe, Ue] = trim_nl(s.V0, 600, 0);
        Uf = @(tt) Ue + [interp1(t, dthr, tt, 'previous', 0); deg2rad(interp1(t, dde, tt, 'previous', 0)); ...
                         -deg2rad(interp1(t, dda, tt, 'previous', 0)); deg2rad(interp1(t, ddr, tt, 'previous', 0)); 0; 0; 0];
        [~, Xn] = ode45(@(tt, x) dyn_rigidbody_DH(tt, x, Uf(tt)), t, Xe);
        nl = struct('VT', sqrt(Xn(:,1).^2 + Xn(:,3).^2), 'p', R2D*Xn(:,4), 'q', R2D*Xn(:,5), 'r', R2D*Xn(:,6), ...
                    'phi', R2D*Xn(:,7), 'theta', R2D*Xn(:,8));
        % ---- X-Plane na grade real ----
        ip = @(y) interp1(r.t, y, t, 'linear', 'extrap');
        xp = struct('VT', ip(r.V), 'p', ip(R2D*r.p), 'q', ip(R2D*r.q), 'r', ip(R2D*r.r), 'phi', ip(r.phi), 'theta', ip(r.theta));
        re = struct('VT', s.V(:), 'p', R2D*s.p(:), 'q', R2D*s.q(:), 'r', R2D*s.r(:), 'phi', s.phi(:), 'theta', s.theta(:));
        dl = @(y) y - mean(y(ipre));                       % nivel pre-doublet removido
        switch s.eixo
            case 'elev', sig = {'dVT', 'dtheta', 'q'}; z = {dl(re.VT), dl(re.theta), re.q}; yN = {dl(nl.VT), dl(nl.theta), nl.q}; yX = {dl(xp.VT), dl(xp.theta), xp.q};
            case 'ail',  sig = {'p', 'dphi', ''};        z = {re.p, dl(re.phi)};          yN = {nl.p, dl(nl.phi)};          yX = {xp.p, dl(xp.phi)};
            case 'rudd', sig = {'r', 'dphi', ''};        z = {re.r, dl(re.phi)};          yN = {nl.r, dl(nl.phi)};          yX = {xp.r, dl(xp.phi)};
        end
        v = nan(1, 6);
        for c = 1:numel(z)
            v(2*c-1) = ticf(z{c}(m), yN{c}(m)); v(2*c) = ticf(z{c}(m), yX{c}(m));
        end
        rows(end+1, :) = [j, v]; ax{end+1} = s.eixo; %#ok<AGROW>
        fprintf(' %3d | %-4s | %6s %.2f | %.2f | %6s %.2f | %.2f | %6s %s\n', j, s.eixo, sig{1}, v(1), v(2), sig{2}, v(3), v(4), sig{3}, fmt3(v(5), v(6)));
    end
    T = struct('rotulo', rotulo, 'win', win, 'rows', rows, 'eixo', {ax});
    fprintf(' medianas (NL | XP):');
    for e = {'elev', 'ail', 'rudd'}
        mm = strcmp(ax, e{1}); if ~nnz(mm), continue; end
        md = median(rows(mm, 2:end), 1, 'omitnan');
        switch e{1}
            case 'elev', fprintf('  profundor(n=%d) dVT %.2f|%.2f dtheta %.2f|%.2f q %.2f|%.2f;', nnz(mm), md(1), md(2), md(3), md(4), md(5), md(6));
            case 'ail',  fprintf('  aileron(n=%d) p %.2f|%.2f dphi %.2f|%.2f;', nnz(mm), md(1), md(2), md(3), md(4));
            case 'rudd', fprintf('  leme(n=%d) r %.2f|%.2f dphi %.2f|%.2f;', nnz(mm), md(1), md(2), md(3), md(4));
        end
    end
    fprintf('\n');
end

function s = fmt3(a, b)
    if isnan(a), s = ''; else, s = sprintf('%.2f | %.2f', a, b); end
end

function [Xe, Ue] = trim_nl(Ve, he, gammae)
    % trim do modelo da Ana (mesma receita do VR_tic_deck / plot_degrau_3vias_sato)
    montaX = @(y) [Ve*cos(y(1)); 0; Ve*sin(y(1)); 0; 0; 0; 0; y(1)+gammae; 0; 0; 0; -he; 0; 0];
    montaU = @(y) [y(2); y(3); 0; 0; 0; 0; 0];
    resid  = @(y) subsref(dyn_rigidbody_DH(0, montaX(y), montaU(y)), struct('type', '()', 'subs', {{[1 3 5]}}));
    Jc = @(y) sum(resid(y).^2);
    y0 = [deg2rad(6), 0.5, deg2rad(3)];
    opt = optimset('MaxFunEvals', 5e4, 'MaxIter', 5e4, 'TolFun', 1e-16, 'TolX', 1e-14);
    y = fminsearch(Jc, y0, opt); y = fminsearch(Jc, y, opt);
    Xe = montaX(y); Ue = montaU(y);
end
