function M = lqry_v3_figuras_casos(varargin)
% LQRY_V3_FIGURAS_CASOS  Figuras (estilo Fig. 38 do artigo, plot_caso_estilo_mirko) e
% metricas (Tab. 12: OS, t_r 10-90 %, t_s 2 %, e_ss) dos 8 casos da Tabela 11, com os
% ganhos ORIGINAIS (reproducao_SIL\caso*_nominal.mat) sobrepostos aos ganhos v3
% (lqry_v3\sil\casos_artigo\caso*_v3.mat). Tambem a Fig. 38 (Caso 4 x inercia) do v3.
%
%   M = lqry_v3_figuras_casos                 % todos os casos disponiveis
%   M = lqry_v3_figuras_casos('casos', [2 4])
%
% Janelas das metricas: 1o degrau de cada doublet, detectado no proprio sinal de
% referencia logado (o artigo usa Airspeed 10-20 s, Altitude 40-60 s, psi/phi 90-110 s,
% theta 40-60 s). Saidas em lqry_v3/sil/casos_artigo/: Caso_0k_orig_vs_v3.png,
% Fig38_v3_caso4_inercia.png, metricas_casos_orig_vs_v3.{txt,csv,tex}.
p = inputParser;
p.addParameter('casos', 1:8);
p.addParameter('tmax', 150);
p.parse(varargin{:}); o = p.Results;
here = fileparts(mfilename('fullpath'));
mirko = 'C:\Users\kaue\Documents\Dissertacao_Mestrado\lqry_mirko_atualizado\reproducao_SIL\mirko_run';
origdir = 'C:\Users\kaue\Documents\Dissertacao_Mestrado\lqry_mirko_atualizado\reproducao_SIL';
outdir = fullfile(here, 'sil', 'casos_artigo');
addpath(mirko);                                   % plot_caso_estilo_mirko, metricas_degrau

nomeProp = {'Throttle fixo', 'Velocity Hold'}; nomeLat = {'\psi Hold', '\phi Hold'}; nomeLong = {'H Hold', '\theta Hold'};
L = {}; rows = {};
function push(fmt, varargin), L{end+1} = sprintf(fmt, varargin{:}); end %#ok<*AGROW>
push('METRICAS DOS CASOS DO ARTIGO (Tabela 11) — ganhos ORIGINAIS x v3, SIL do Mirko, Cond. 5 (15 m/s), refs 5, atuador ideal');
push('1o degrau de cada doublet; OS relativo a amplitude; t_r 10-90 %%; t_s faixa 2 %%; e_ss = media dos ultimos 2 s da janela - ref');
push('(alc = fracao do degrau atingida ao fim da janela; tr = NaN quando 90 %% nao e atingido dentro da janela)');
push('%-6s %-34s %-9s | %7s %7s %7s %8s %6s | %7s %7s %7s %8s %6s', 'Caso', 'Modos', 'Variavel', 'OS_o', 'tr_o', 'ts_o', 'ess_o', 'alc_o', 'OS_v3', 'tr_v3', 'ts_v3', 'ess_v3', 'alc_v3');
M = struct([]);
for caso = o.casos
    fo = fullfile(origdir, sprintf('caso%d_nominal.mat', caso));
    fv = fullfile(outdir, sprintf('caso%d_v3.mat', caso));
    if ~exist(fo, 'file') || ~exist(fv, 'file'), fprintf('  [caso %d] faltam dados (%d/%d)\n', caso, exist(fo,'file')>0, exist(fv,'file')>0); continue; end
    So = load(fo); So = So.S; Sv = load(fv); Sv = Sv.S;
    cfg = So.cfg; if isfield(Sv, 'erro') && ~isempty(Sv.erro), fprintf('  [caso %d] v3 com erro: %s\n', caso, Sv.erro); continue; end
    modos = sprintf('%s + %s + %s', nomeProp{cfg.VT_Throttle+1}, nomeLat{cfg.phi_psi+1}, nomeLong{cfg.att_alt+1});
    modos_txt = strrep(modos, '\', '');
    % --- figura sobreposta ---
    fig = plot_caso_estilo_mirko({So, Sv}, {'Ganhos originais (Mirko)', 'Ganhos v3'}, caso, cfg, '  — original x v3', o.tmax);
    png = fullfile(outdir, sprintf('Caso_%02d_orig_vs_v3.png', caso));
    exportgraphics(fig, png, 'Resolution', 170, 'BackgroundColor', 'white'); close(fig);
    % --- metricas por canal ativo ---
    canais = {};
    if cfg.VT_Throttle == 1, canais(end+1,:) = {'Airspeed', 'VT_NL', 2, 1}; end
    if cfg.att_alt == 0 && cfg.refAlt ~= 0, canais(end+1,:) = {'Altitude', 'H_NL', 2, 1}; else, if cfg.att_alt == 1, canais(end+1,:) = {'theta', 'theta_NL', 2, 1}; end, end
    if cfg.phi_psi == 0, canais(end+1,:) = {'psi', 'psi_NL', 1, 2}; else, canais(end+1,:) = {'phi', 'phi_NL', 2, 1}; end
    for j = 1:size(canais, 1)
        so = So.(canais{j,2}); sv = Sv.(canais{j,2});
        [t0, t1] = janela_ref(so.time, so.values(:, canais{j,4}));
        if isnan(t0), continue; end
        mo = metricas_degrau(so.time, so.values(:, canais{j,3}), so.values(:, canais{j,4}), t0, t1);
        mv = metricas_degrau(sv.time, sv.values(:, canais{j,3}), sv.values(:, canais{j,4}), t0, t1);
        ao = alcance(so.time, so.values(:, canais{j,3}), so.values(:, canais{j,4}), t0, t1);
        av = alcance(sv.time, sv.values(:, canais{j,3}), sv.values(:, canais{j,4}), t0, t1);
        push('%-6d %-34s %-9s | %7.2f %7.2f %7.2f %+8.3f %5.0f%% | %7.2f %7.2f %7.2f %+8.3f %5.0f%%', caso, modos_txt, canais{j,1}, mo.os, mo.tr, mo.ts, mo.ess, ao, mv.os, mv.tr, mv.ts, mv.ess, av);
        rows(end+1,:) = {caso, modos_txt, canais{j,1}, t0, t1, mo.os, mo.tr, mo.ts, mo.ess, mv.os, mv.tr, mv.ts, mv.ess};
        M(end+1).caso = caso; M(end).canal = canais{j,1}; M(end).janela = [t0 t1]; M(end).orig = mo; M(end).v3 = mv;
    end
    % --- extremos (o que o artigo nao mostra: quanto o comando pede) ---
    push('%-6s %-34s %-9s | de %+.1f..%+.1f deg, thr %.2f..%.2f, theta %.1f..%.1f, phi %.1f..%.1f | de %+.1f..%+.1f, thr %.2f..%.2f, theta %.1f..%.1f, phi %.1f..%.1f', ...
        '', '', 'extremos', min(So.elev_NL.values(:,1)), max(So.elev_NL.values(:,1)), min(So.Throttle_NL.values(:,1)), max(So.Throttle_NL.values(:,1)), ...
        min(So.theta_NL.values(:,2)), max(So.theta_NL.values(:,2)), min(So.phi_NL.values(:,2)), max(So.phi_NL.values(:,2)), ...
        min(Sv.elev_NL.values(:,1)), max(Sv.elev_NL.values(:,1)), min(Sv.Throttle_NL.values(:,1)), max(Sv.Throttle_NL.values(:,1)), ...
        min(Sv.theta_NL.values(:,2)), max(Sv.theta_NL.values(:,2)), min(Sv.phi_NL.values(:,2)), max(Sv.phi_NL.values(:,2)));
    fprintf('  caso %d -> %s\n', caso, png);
end
% --- Fig. 38 do v3: Caso 4 x inercia (nominal, +10% "do codigo" x1.90, -10%) ---
fi = {fullfile(outdir, 'caso4_v3.mat'), fullfile(outdir, 'caso4_iner2_v3.mat'), fullfile(outdir, 'caso4_iner1_v3.mat')};
if all(cellfun(@(f) exist(f, 'file') > 0, fi))
    Lr = cellfun(@(f) getfield(load(f), 'S'), fi, 'UniformOutput', false);
    fig = plot_caso_estilo_mirko(Lr, {'Caso nominal', 'Inércia +10% (cód. \times1,90)', 'Inércia -10%'}, 4, Lr{1}.cfg, '  — ganhos v3 (Fig. 38 refeita)', o.tmax);
    exportgraphics(fig, fullfile(outdir, 'Fig38_v3_caso4_inercia.png'), 'Resolution', 200, 'BackgroundColor', 'white'); close(fig);
    push('');
    push('FIG. 38 / TAB. 12 REFEITAS COM OS GANHOS v3 (Caso 4 x inercia):');
    push('%-20s %-9s %8s %8s %9s %9s', 'Inercia', 'Variavel', 'OS[%]', 'tr[s]', 'ts2%[s]', 'ess');
    nomes = {'Nominal', '+10% cod. (x1.90)', '-10% (x0.90)'};
    for k = 1:3
        for jan = {'Airspeed','VT_NL',2,1; 'Altitude','H_NL',2,1; 'psi','psi_NL',1,2}'
            s = Lr{k}.(jan{2}); [t0, t1] = janela_ref(s.time, s.values(:, jan{4}));
            m = metricas_degrau(s.time, s.values(:, jan{3}), s.values(:, jan{4}), t0, t1);
            push('%-20s %-9s %8.2f %8.2f %9.2f %+9.3f', nomes{k}, jan{1}, m.os, m.tr, m.ts, m.ess);
        end
    end
    fi3 = fullfile(outdir, 'caso4_iner3_v3.mat');
    if exist(fi3, 'file')
        S3 = load(fi3); S3 = S3.S;
        for jan = {'Airspeed','VT_NL',2,1; 'Altitude','H_NL',2,1; 'psi','psi_NL',1,2}'
            s = S3.(jan{2}); [t0, t1] = janela_ref(s.time, s.values(:, jan{4}));
            m = metricas_degrau(s.time, s.values(:, jan{3}), s.values(:, jan{4}), t0, t1);
            push('%-20s %-9s %8.2f %8.2f %9.2f %+9.3f', '+10% real (x1.10)', jan{1}, m.os, m.tr, m.ts, m.ess);
        end
    end
end
fprintf('%s\n', L{:});
fid = fopen(fullfile(outdir, 'metricas_casos_orig_vs_v3.txt'), 'w'); fprintf(fid, '%s\n', L{:}); fclose(fid);
fid = fopen(fullfile(outdir, 'metricas_casos_orig_vs_v3.csv'), 'w');
fprintf(fid, 'caso,modos,canal,t0,t1,OS_orig,tr_orig,ts_orig,ess_orig,OS_v3,tr_v3,ts_v3,ess_v3\n');
for r = 1:size(rows, 1), fprintf(fid, '%d,"%s",%s,%g,%g,%.4g,%.4g,%.4g,%.4g,%.4g,%.4g,%.4g,%.4g\n', rows{r,:}); end
fclose(fid);
tex = {'\begin{table}[htbp]', '\centering', ...
    '\caption{Casos da Tabela 11 do artigo no modelo n\~ao linear: ganhos originais e re-s\''intese (sobressinal, tempo de subida 10--90\,\%, tempo de acomoda\c{c}\~ao 2\,\% e erro de regime no primeiro degrau de cada doublet).}', ...
    '\label{tab:casos_artigo_orig_v3}', '\begin{tabular}{clrrrrrrrr}', '\hline', ...
    ' & & \multicolumn{4}{c}{Original} & \multicolumn{4}{c}{Proposto} \\', ...
    'Caso & Vari\''avel & OS [\%] & $t_r$ [s] & $t_s$ [s] & $e_{ss}$ & OS [\%] & $t_r$ [s] & $t_s$ [s] & $e_{ss}$ \\', '\hline'};
for r = 1:size(rows, 1)
    tex{end+1} = sprintf('%d & %s & %.1f & %.2f & %s & %.3f & %.1f & %.2f & %s & %.3f \\\\', rows{r,1}, rows{r,3}, rows{r,6}, rows{r,7}, tsstr(rows{r,8}), rows{r,9}, rows{r,10}, rows{r,11}, tsstr(rows{r,12}), rows{r,13});
end
tex = [tex, {'\hline', '\end{tabular}', '\end{table}'}];
fid = fopen(fullfile(outdir, 'metricas_casos_orig_vs_v3.tex'), 'w'); fprintf(fid, '%s\n', tex{:}); fclose(fid);
fprintf('\nsalvos em %s\n', outdir);
end

function [t0, t1] = janela_ref(t, r)
% primeiro degrau da referencia logada e o degrau seguinte (fim da janela)
k = find(abs(diff(r)) > 1e-6, 1);
if isempty(k), t0 = NaN; t1 = NaN; return; end
t0 = t(k+1);
k2 = find(abs(diff(r(k+1:end))) > 1e-6, 1);
if isempty(k2), t1 = t(end); else, t1 = t(k + k2 + 1); end
end

function s = tsstr(v)
if isnan(v), s = '--'; else, s = sprintf('%.2f', v); end
end

function a = alcance(t, y, r, t0, t1)
% fracao do degrau atingida ao fim da janela [%]: (y(t1-) - y(t0)) / (ref - y(t0))
seg = t >= t0 & t < t1; ys = y(seg); rs = r(seg);
if isempty(ys), a = NaN; return; end
dy = rs(end) - ys(1); if abs(dy) < 1e-9, a = NaN; return; end
a = 100*(ys(end) - ys(1))/dy;
end
