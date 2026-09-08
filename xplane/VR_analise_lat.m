function M = VR_analise_lat(arq_fix, arq_ref, rotulos, png_out)
%VR_ANALISE_LAT Metricas latero-direcionais real x X-Plane, campanha a campanha.
%
%   M = VR_analise_lat('VR_replay_GEMEO_FIX_MERGED.mat', 'VR_replay_GEMEO_JCAL_MERGED.mat', ...
%                      {'v1.2 sinal+escala corrigidos','v1.2 (09-01, leme invertido, 1,67x)'}, 'VR_lat_fix_vs_jcal.png')
%
% Por segmento lateral (aileron: p, dphi; leme: r, dphi, f_DR, zeta_DR):
%   fit%  = 100*(1 - ||e||/||y - mean(y)||) na janela dinamica (1 s antes -> 4 s apos)
%   pico  = max|taxa_XP| / max|taxa_real| na mesma janela
%   dphi  = max|dphi_XP| / max|dphi_real| (0 -> 4 s apos o doublet; nivel pre-doublet removido)
%   f_DR  = frequencia dominante de r apos o doublet (FFT, detrend)
%   zeta  = decremento logaritmico dos picos sucessivos de |r| apos o doublet
%   TIC   = Theil, taxa primaria, janela dinamica
% Figura (opcional): 2x2 — leme (seg 20) r e dphi; aileron (seg 12) p e dphi;
% preto = voo real, vermelho tracejado = referencia, azul = corrigida.
    xpDir = fileparts(mfilename('fullpath')); vd = fullfile(xpDir, 'voos');
    S = load(fullfile(vd, 'VR_segmentos.mat')); SEG = S.SEG; T_PRE = S.T_PRE;
    camp = {arq_fix}; if nargin >= 2 && ~isempty(arq_ref), camp{end+1} = arq_ref; end
    if nargin < 3 || isempty(rotulos), rotulos = camp; end
    R2D = 180/pi; M = struct();
    for ic = 1:numel(camp)
        C = load(fullfile(vd, camp{ic})); R = C.R;
        fprintf('\n=== %s (%s) ===\n', rotulos{ic}, camp{ic});
        fprintf(' seg | eixo | V0   | fit%% | pico | dphi | TIC  | fDR real | fDR XP | zeta real | zeta XP\n');
        T = [];
        for k = 1:numel(R)
            r = R(k); s = SEG(r.iseg);
            if strcmp(s.eixo, 'elev'), continue; end
            if strcmp(s.eixo, 'ail'), yr0 = s.p; yx0 = r.p; else, yr0 = s.r; yx0 = r.r; end
            tmax = min(s.t(end), r.t(end)); tg = s.t(s.t <= tmax);
            yr = R2D*yr0(s.t <= tmax); yx = interp1(r.t, R2D*yx0, tg);
            idy = tg > T_PRE-1 & tg < T_PRE+4 & ~isnan(yx);
            e = yx(idy) - yr(idy);
            fit = 100*(1 - norm(e)/norm(yr(idy) - mean(yr(idy))));
            pico = max(abs(yx(idy))) / max(abs(yr(idy)));
            tic_ = sqrt(mean(e.^2)) / (sqrt(mean(yr(idy).^2)) + sqrt(mean(yx(idy).^2)));
            ipre = tg < T_PRE - 0.5; idw = tg > T_PRE & tg < T_PRE + 4;
            pr = s.phi(s.t <= tmax); pr = pr - mean(pr(ipre));
            px = interp1(r.t, r.phi, tg); px = px - mean(px(ipre), 'omitnan');
            dphi = max(abs(px(idw))) / max(abs(pr(idw)));
            fr = NaN; fx = NaN; zr = NaN; zx = NaN;
            if strcmp(s.eixo, 'rudd')
                fr = fdom(tg, yr, T_PRE + 1.5); fx = fdom(tg, yx, T_PRE + 1.5);
                zr = zeta_dec(tg, yr, T_PRE + 1.0); zx = zeta_dec(tg, yx, T_PRE + 1.0);
            end
            T(end+1,:) = [r.iseg, strcmp(s.eixo,'rudd'), s.V0, fit, pico, dphi, tic_, fr, fx, zr, zx]; %#ok<AGROW>
            fprintf(' %3d | %-4s | %4.1f | %4.0f | %4.2f | %4.2f | %4.2f | %5.2f | %5.2f | %5.2f | %5.2f\n', ...
                r.iseg, s.eixo, s.V0, fit, pico, dphi, tic_, fr, fx, zr, zx);
        end
        for ax = 0:1
            m = T(:,2) == ax; if ~nnz(m), continue; end
            nm = 'aileron'; if ax, nm = 'leme'; end
            fprintf(' %-7s (n=%d): fit medio %+.0f%% | pico mediana %.2f [%.2f..%.2f] | dphi mediana %.2f | TIC mediana %.2f', ...
                nm, nnz(m), mean(T(m,4)), median(T(m,5)), min(T(m,5)), max(T(m,5)), median(T(m,6)), median(T(m,7)));
            if ax, fprintf(' | fDR real %.2f XP %.2f | zeta real %.2f XP %.2f', ...
                    median(T(m,8),'omitnan'), median(T(m,9),'omitnan'), median(T(m,10),'omitnan'), median(T(m,11),'omitnan')); end
            fprintf('\n');
        end
        M(ic).rotulo = rotulos{ic}; M(ic).T = T; M(ic).R = R;
    end
    if nargin >= 4 && ~isempty(png_out)
        fig = figure('Visible','off','Position',[0 0 1400 800]); try, fig.Theme = 'light'; catch, end
        cores = {[0 0.3 1], [0.85 0 0]}; est = {'-', '--'};
        painel = {20, 'rudd'; 12, 'ail'};
        for ip = 1:2
            j = painel{ip,1}; s = SEG(j); tg = s.t;
            if strcmp(painel{ip,2}, 'ail'), yr = R2D*s.p; nomeTaxa = 'p [deg/s]'; else, yr = R2D*s.r; nomeTaxa = 'r [deg/s]'; end
            ipre = tg < T_PRE - 0.5; pr = s.phi - mean(s.phi(ipre));
            subplot(2,2,2*ip-1); hold on; plot(tg, yr, 'k', 'LineWidth', 1.6);
            subplot(2,2,2*ip);   hold on; plot(tg, pr, 'k', 'LineWidth', 1.6);
            leg = {'voo real'};
            for ic = numel(M):-1:1
                k = find([M(ic).R.iseg] == j); if isempty(k), continue; end
                r = M(ic).R(k);
                if strcmp(painel{ip,2}, 'ail'), yx = R2D*r.p; else, yx = R2D*r.r; end
                px = r.phi - mean(r.phi(r.t < T_PRE - 0.5));
                subplot(2,2,2*ip-1); plot(r.t, yx, est{ic}, 'Color', cores{ic}, 'LineWidth', 1.3);
                subplot(2,2,2*ip);   plot(r.t, px, est{ic}, 'Color', cores{ic}, 'LineWidth', 1.3);
                leg{end+1} = M(ic).rotulo; %#ok<AGROW>
            end
            subplot(2,2,2*ip-1); grid on; xlim([1 8]); xlabel('t [s]'); ylabel(nomeTaxa);
            title(sprintf('seg %d (%s) — taxa', j, painel{ip,2})); legend(leg, 'Location', 'best');
            subplot(2,2,2*ip); grid on; xlim([1 8]); xlabel('t [s]'); ylabel('\Delta\phi [deg]');
            title(sprintf('seg %d (%s) — rolagem acoplada', j, painel{ip,2}));
        end
        sgtitle(sprintf('Replay do voo real (malha aberta): %s', strjoin(rotulos, '  x  ')));
        exportgraphics(fig, fullfile(vd, png_out), 'Resolution', 110); close(fig);
        fprintf('Figura: %s\n', fullfile(vd, png_out));
    end
end

function f = fdom(t, y, t_ini)
    m = t > t_ini & ~isnan(y); if nnz(m) < 32, f = nan; return; end
    yy = detrend(y(m)); dt = median(diff(t(m))); n = 2^nextpow2(numel(yy)*4);
    Y = abs(fft(yy, n)); fr = (0:n-1)/(n*dt); band = fr > 0.2 & fr < 5;
    [~, ip] = max(Y(band)); fb = fr(band); f = fb(ip);
end

function z = zeta_dec(t, y, t_ini)
    % decremento logaritmico nos picos sucessivos de |y| (detrend) apos t_ini
    m = t > t_ini & ~isnan(y); z = nan; if nnz(m) < 16, return; end
    yy = abs(detrend(y(m))); pk = [];
    for k = 2:numel(yy)-1
        if yy(k) > yy(k-1) && yy(k) >= yy(k+1) && yy(k) > 0.05*max(yy), pk(end+1) = yy(k); end %#ok<AGROW>
    end
    if numel(pk) < 3, return; end
    n = min(4, numel(pk)) - 1;                    % usa ate 4 picos (2 ciclos)
    d = log(pk(1)/pk(1+n)) / (n/2);               % decremento por ciclo (2 picos de |y| por ciclo)
    z = d / sqrt(4*pi^2 + d^2);
end
