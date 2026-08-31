function plot_G5(id, xpmat, sufixo, rotulo, rotulo_xp)
% plot_G5 — figura comparada SIL x X-Plane da campanha G5 (doublets)
% no MESMO estilo da G4 (5 paineis: h, VT, psi, phi, theta) + metricas
% por fase do doublet no console.
%
% Uso:
%   plot_G5('dblh')                       % usa o XP_voo_*.mat mais recente
%   plot_G5('dblh','XP_voo_20260831_101500.mat')   % voo especifico
%   plot_G5('dblh',[],'_calmo',' — atmosfera calma')  % variante: sufixo
%       no nome do PNG (nao sobrescreve o original) + rotulo no titulo
%
% Requer voos/SIL_PID_G5_<id>.mat (gerado por G5_sil).
% Salva voos/G5_<id>_SILxXP<sufixo>.png.
if nargin < 3 || isempty(sufixo), sufixo = ''; end
if nargin < 4 || isempty(rotulo), rotulo = ''; end
if nargin < 5 || isempty(rotulo_xp), rotulo_xp = 'X-Plane (gêmeo v1.1)'; end

here = fileparts(mfilename('fullpath'));
vd   = fullfile(here, 'voos');

S = load(fullfile(vd, ['SIL_PID_G5_' id '.mat']));
sp = S.silp;  m = sp.manobra;

if nargin < 2 || isempty(xpmat)
    d = dir(fullfile(vd, 'XP_voo_*.mat'));
    if isempty(d), error('plot_G5: nenhum XP_voo_*.mat em voos/.'); end
    [~,i] = max([d.datenum]);  xpmat = d(i).name;
end
X = load(fullfile(vd, xpmat));  v = X.voo;
fprintf('plot_G5(%s): SIL_PID_G5_%s.mat  x  %s\n', id, id, xpmat);

Y  = v.Y;
xp = struct('t',v.t(:), 'VT',Y(1,:)', 'phi',rad2deg(Y(5,:))', ...
    'theta',rad2deg(Y(6,:))', 'psi',rad2deg(Y(7,:))', 'h',Y(8,:)');
sl = struct('t',sp.t(:), 'VT',sp.VT(:), 'phi',rad2deg(sp.phi(:)), ...
    'theta',rad2deg(sp.theta(:)), 'psi',rad2deg(sp.psi(:)), 'h',sp.h(:));

% ---- referencias reconstruidas da definicao da manobra ----
% doublet: ref = base + A*(t>=t1) - 2A*(t>=t2) + A*(t>=t3)
dbl = @(t, base, A, tt) base + A*(t>=tt(1)) - 2*A*(t>=tt(2)) + A*(t>=tt(3));
h_base = 600;  psi_base = 0;  VT_base = 12;
href_of   = @(t) dbl(t, h_base,   m.h_A,   m.h_t);
psiref_of = @(t) dbl(t, psi_base, m.psi_A, m.psi_t);
VTref_of  = @(t) VT_base + m.VT_delta*(t>=m.VT_t);

% ---- metricas por fase (canais ativos) ----
fprintf('\n=== G5 %s — %s: SIL x X-Plane ===\n', id, m.titulo);
fprintf('%-26s %9s %9s\n', 'metrica', 'SIL', 'X-Plane');
canais = {};
if m.h_A   ~= 0, canais(end+1,:) = {'h',   'm',   h_base,   m.h_A,   m.h_t};   end %#ok<AGROW>
if m.psi_A ~= 0, canais(end+1,:) = {'psi', 'deg', psi_base, m.psi_A, m.psi_t}; end %#ok<AGROW>
for c = 1:size(canais,1)
    [ch, un, base, A, tt] = canais{c,:};
    fases = { ...
        sprintf('%s +%g', ch, A), tt(1), min(tt(2), m.T), base,   base+A; ...
        sprintf('%s -%g', ch, A), tt(2), min(tt(3), m.T), base+A, base-A; ...
        sprintf('%s ret', ch),    tt(3), m.T,             base-A, base};
    for f = 1:3
        if fases{f,2} >= m.T, continue; end
        [osS,tsS,esS] = met(sl.t, sl.(ch), fases{f,2:5});
        [osX,tsX,esX] = met(xp.t, xp.(ch), fases{f,2:5});
        fprintf('%-10s OS [%%]        %9.1f %9.1f\n', fases{f,1}, osS, osX);
        fprintf('%-10s ts5%% [s]      %9.1f %9.1f\n', fases{f,1}, tsS, tsX);
        fprintf('%-10s ess [%s]     %9.2f %9.2f\n', fases{f,1}, un, esS, esX);
    end
end
fprintf('RMS h   vs ref [m]:        %9.2f %9.2f\n', ...
    rms(sl.h  - href_of(sl.t)),  rms(xp.h  - href_of(xp.t)));
fprintf('RMS psi vs ref [deg]:      %9.2f %9.2f\n', ...
    rms(sl.psi - psiref_of(sl.t)), rms(xp.psi - psiref_of(xp.t)));
fprintf('RMS VT  vs ref [m/s]:      %9.2f %9.2f\n', ...
    rms(sl.VT - VTref_of(sl.t)), rms(xp.VT - VTref_of(xp.t)));

% ---- figura (estilo G4: fundo branco, 5 paineis) ----
f = figure('Color','w','Position',[50 50 900 900]);
tiledlayout(5,1,'TileSpacing','compact','Padding','compact');
cS = [0.00 0.45 0.74]; cX = [0.85 0.33 0.10]; cR = [0.4 0.4 0.4];
pans = { ...
  sl.h,     xp.h,     href_of(sl.t),   'h [m]'; ...
  sl.VT,    xp.VT,    VTref_of(sl.t),  'V_T [m/s]'; ...
  sl.psi,   xp.psi,   psiref_of(sl.t), '\psi [deg]'; ...
  sl.phi,   xp.phi,   [],              '\phi [deg]'; ...
  sl.theta, xp.theta, [],              '\theta [deg]'};
for k = 1:5
    ax = nexttile; hold(ax,'on'); grid(ax,'on');
    set(ax,'Color','w','XColor','k','YColor','k','GridColor',[0.78 0.78 0.78],'Toolbar',[]);
    if ~isempty(pans{k,3})
        plot(ax, sl.t, pans{k,3}, '--', 'Color', cR, 'LineWidth', 1.0);
    end
    plot(ax, sl.t, pans{k,1}, '-', 'Color', cS, 'LineWidth', 1.3);
    plot(ax, xp.t, pans{k,2}, '-', 'Color', cX, 'LineWidth', 1.1);
    ylabel(ax, pans{k,4}, 'Color','k'); xlim(ax, [0 m.T]);
    if k==1
        title(ax, ['G5 — PID cascata, SIL vs ' rotulo_xp ': ' m.titulo rotulo], 'Color','k');
        lg = legend(ax, {'referência','SIL (modelo da Ana)',rotulo_xp}, 'Location','southeast');
        set(lg,'Color','w','TextColor','k','EdgeColor',[0.5 0.5 0.5]);
    end
    if k==5, xlabel(ax,'t [s]','Color','k'); end
end
png = fullfile(vd, ['G5_' id '_SILxXP' sufixo '.png']);
exportgraphics(f, png, 'Resolution', 130);
fprintf('figura salva: %s\n', png);
end

function [os,ts,ess] = met(t, y, t0, t1, y0, yf)
% metricas de um degrau y0->yf na janela [t0,t1]: OS%, ts5% e ess (media
% dos ultimos 5 s). MESMA convencao do G4 (g4_pid_analise): ts = ultimo
% instante fora da faixa de 5% — se nao assenta na janela, ts satura no
% comprimento dela (e o ess conta a historia).
k = t>=t0 & t<=t1;  tk = t(k);  yk = y(k);
amp = yf - y0;
os  = (max(sign(amp)*(yk - yf))/abs(amp))*100;
tol = 0.05*abs(amp);
ki  = find(abs(yk - yf) > tol, 1, 'last');
if isempty(ki), ts = 0; else, ts = tk(min(ki+1,numel(tk))) - t0; end
ess = mean(yk(tk > t1-5)) - yf;
end
