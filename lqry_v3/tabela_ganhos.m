function T = tabela_ganhos(varargin)
% TABELA_GANHOS  Comparacao dos ganhos do LQRy: projeto ORIGINAL do Mirko
% (lqry_mirko_atualizado\Nova pasta) x re-sintese v3 (lqry_v3\ganhos), nas
% mesmas unidades dos Ganho_hold_*.mat e tambem em "ganho efetivo por grau"
% (a forma fisicamente legivel: quantos graus de superficie por grau de erro).
%
%   T = tabela_ganhos            % imprime, salva .txt/.csv/.tex em lqry_v3/ganhos/
%   T = tabela_ganhos('iset', [2 5 8], 'salvar', false)
%
% Estrutura (identica nos dois conjuntos — o v3 mexe SO nos numeros):
%   theta Hold : de[deg]      = GstateLong*[dVT dalpha q dtheta de_deg] + GintLong*int(theta-theta_ref)
%   Alt   Hold : theta_ref[rad]= GstateLong_Alt*[dVT dalpha q dtheta dH]  + GintLong_Alt*int(H-H_ref)
%   Vel   Hold : thr[%]       = GstateLong_speed*[dVT dalpha q dtheta]    + GintLong_speed*int(VT-VT_ref)
%   phi   Hold : [da;dr][rad] = GstateLat*[beta p r phi]                  + Gintlat*[int(phi-phi_ref); int(beta)]
%   psi   Hold : phi_ref[rad] = GstateLat_psi*[beta p r phi psi]          + Gintlat_psi*int(psi-psi_ref)
%
% Kaue / Claude, 2026-09-03.
p = inputParser;
p.addParameter('raizN', 'C:\Users\kaue\Documents\Dissertacao_Mestrado\lqry_mirko_atualizado\Nova pasta');
p.addParameter('v3dir', fullfile(fileparts(mfilename('fullpath')), 'ganhos'));
p.addParameter('iset', [2 5 8]);        % 12, 15 e 18 m/s @ 600 m
p.addParameter('salvar', true);
p.parse(varargin{:}); o = p.Results;
D2R = pi/180; R2D = 180/pi;

O = carrega(o.raizN); V = carrega(o.v3dir);
S = load(fullfile(o.raizN, 'Dados_Trim.mat')); Plantas = S.Plantas;

L = {};   % linhas do relatorio
add = @(varargin) 0;  %#ok<NASGU>
function push(fmt, varargin), L{end+1} = sprintf(fmt, varargin{:}); end %#ok<*AGROW>

push('TABELA DE GANHOS — LQRy do Mirko: projeto ORIGINAL x re-sintese v3   (%s)', char(datetime('now')));
push('Estrutura do controlador IDENTICA nos dois conjuntos; mudam apenas os valores.');
push('');

%% ---------- 1) ganhos brutos, nas unidades dos .mat ----------
push('== 1) GANHOS BRUTOS (unidades dos Ganho_hold_*.mat) ==');
for i = o.iset
    push('');
    push('-- Planta %d (%s: %g m/s @ %g m) --', i, Plantas(i).nome, Plantas(i).Ve, Plantas(i).He);
    par = { 'theta Hold  Gstate [dVT dalpha q dtheta de]', O.GstateLong{i},      V.GstateLong{i}
            'theta Hold  Gint',                            O.GintLong{i},        V.GintLong{i}
            'Alt Hold    Gstate [dVT dalpha q dtheta dH]', O.GstateLong_Alt{i},  V.GstateLong_Alt{i}
            'Alt Hold    Gint',                            O.GintLong_Alt{i},    V.GintLong_Alt{i}
            'Vel Hold    Gstate [dVT dalpha q dtheta]',    O.GstateLong_speed{i},V.GstateLong_speed{i}
            'Vel Hold    Gint',                            O.GintLong_speed{i},  V.GintLong_speed{i}
            'phi Hold    Gstate [beta p r phi] (2x4)',     O.GstateLat{i},       V.GstateLat{i}
            'phi Hold    Gint [int_phi int_beta] (2x2)',   O.Gintlat{i},         V.Gintlat{i}
            'psi Hold    Gstate [beta p r phi psi]',       O.GstateLat_psi{i},   V.GstateLat_psi{i}
            'psi Hold    Gint',                            O.Gintlat_psi{i},     V.Gintlat_psi{i} };
    for k = 1:size(par,1)
        push('%-44s ORIG %s', par{k,1}, mat2str(double(par{k,2}), 4));
        push('%-44s  V3  %s', '',       mat2str(double(par{k,3}), 4));
    end
end

%% ---------- 2) ganhos efetivos (por grau / por metro) ----------
push('');
push('== 2) GANHOS EFETIVOS (por grau, por metro, por m/s) — a leitura fisica ==');
push('%-34s %-10s %10s %10s %8s', 'Ganho efetivo', 'unidade', 'ORIGINAL', 'v3', 'orig/v3');
T = struct([]);
for i = o.iset
    e = efetivos(O, i, D2R, R2D); v = efetivos(V, i, D2R, R2D);
    push('');
    push('-- %g m/s (planta %d) --', Plantas(i).Ve, i);
    fn = fieldnames(e);
    for k = 1:numel(fn)
        a = e.(fn{k}); b = v.(fn{k});
        push('%-34s %-10s %10.4g %10.4g %8.1f', a.nome, a.un, a.val, b.val, abs(a.val/b.val));
    end
    T(end+1).i = i; T(end).Ve = Plantas(i).Ve; T(end).orig = e; T(end).v3 = v;
end

%% ---------- 3) o que satura o atuador ----------
push('');
push('== 3) ERRO QUE SATURA O ATUADOR (curso real do DH: superficies +-15 deg, manete [0,1]) ==');
push('%-46s %12s %12s', 'Condicao', 'ORIGINAL', 'v3');
for i = o.iset
    e = efetivos(O, i, D2R, R2D); v = efetivos(V, i, D2R, R2D);
    push('');
    push('-- %g m/s --', Plantas(i).Ve);
    push('%-46s %12.2f %12.2f', 'erro de theta p/ 15 deg de profundor [deg]', 15/abs(e.th_de.val),  15/abs(v.th_de.val));
    push('%-46s %12.2f %12.2f', 'erro de H p/ 10 deg de theta_ref [m]',       10/abs(e.H_thref.val),10/abs(v.H_thref.val));
    push('%-46s %12.2f %12.2f', 'erro de VT p/ 100 % de manete [m/s]',         100/abs(e.VT_thr.val),100/abs(v.VT_thr.val));
    push('%-46s %12.2f %12.2f', 'erro de psi p/ 30 deg de phi_ref [deg]',      30/abs(e.psi_phiref.val), 30/abs(v.psi_phiref.val));
end

%% ---------- 4) tabela LaTeX (dissertacao) ----------
tex = {};
tex{end+1} = '\begin{table}[htbp]';
tex{end+1} = '\centering';
tex{end+1} = '\caption{Ganhos efetivos do LQRy: projeto original e re-sintese proposta (mesma estrutura de controle).}';
tex{end+1} = '\label{tab:ganhos_lqry_orig_v3}';
tex{end+1} = '\begin{tabular}{llrrr}';
tex{end+1} = '\hline';
tex{end+1} = 'Malha & Unidade & Original & Proposto & Razao \\';
tex{end+1} = '\hline';
i0 = o.iset(min(2, numel(o.iset)));            % 15 m/s como referencia da tabela
e = efetivos(O, i0, D2R, R2D); v = efetivos(V, i0, D2R, R2D);
fn = fieldnames(e);
for k = 1:numel(fn)
    a = e.(fn{k}); b = v.(fn{k});
    tex{end+1} = sprintf('%s & %s & %.3g & %.3g & %.1f \\\\', a.tex, a.untex, a.val, b.val, abs(a.val/b.val));
end
tex{end+1} = '\hline';
tex{end+1} = '\end{tabular}';
tex{end+1} = sprintf('\\\\[2pt]\\footnotesize Condicao de voo: %g m/s, %g m (planta %d de %d do escalonamento).', ...
    Plantas(i0).Ve, Plantas(i0).He, i0, numel(Plantas));
tex{end+1} = '\end{table}';

%% ---------- saida ----------
fprintf('%s\n', L{:});
if o.salvar
    txt = fullfile(o.v3dir, 'tabela_ganhos_orig_vs_v3.txt');
    fid = fopen(txt, 'w'); fprintf(fid, '%s\n', L{:}); fclose(fid);
    fid = fopen(fullfile(o.v3dir, 'tabela_ganhos_orig_vs_v3.tex'), 'w'); fprintf(fid, '%s\n', tex{:}); fclose(fid);
    % CSV: uma linha por (planta, ganho)
    fid = fopen(fullfile(o.v3dir, 'tabela_ganhos_orig_vs_v3.csv'), 'w');
    fprintf(fid, 'planta,Ve_m_s,ganho,unidade,original,v3,razao_orig_v3\n');
    for i = o.iset
        e = efetivos(O, i, D2R, R2D); v = efetivos(V, i, D2R, R2D); fn = fieldnames(e);
        for k = 1:numel(fn)
            a = e.(fn{k}); b = v.(fn{k});
            fprintf(fid, '%d,%g,"%s","%s",%.6g,%.6g,%.4g\n', i, Plantas(i).Ve, a.nome, a.un, a.val, b.val, abs(a.val/b.val));
        end
    end
    fclose(fid);
    fprintf('\nsalvos em %s:\n  tabela_ganhos_orig_vs_v3.txt / .csv / .tex\n', o.v3dir);
end
end

% ======================================================================
function G = carrega(d)
G = struct();
for f = {'Ganho_hold_theta','Ganho_hold_H','Ganho_hold_VT','Ganho_hold_phi','Ganho_hold_psi'}
    s = load(fullfile(d, [f{1} '.mat'])); fn = fieldnames(s);
    for k = 1:numel(fn), G.(fn{k}) = s.(fn{k}); end
end
end

function e = efetivos(G, i, D2R, R2D)
gt = double(G.GstateLong{i});       git = double(G.GintLong{i});
ga = double(G.GstateLong_Alt{i});   gia = double(G.GintLong_Alt{i});
gv = double(G.GstateLong_speed{i}); giv = double(G.GintLong_speed{i});
gl = double(G.GstateLat{i});        gil = double(G.Gintlat{i});
gp = double(G.GstateLat_psi{i});    gip = double(G.Gintlat_psi{i});
mk = @(nome, un, val, tx, untx) struct('nome', nome, 'un', un, 'val', val, 'tex', tx, 'untex', untx);
e = struct();
e.th_de     = mk('theta -> profundor',        'deg/deg',     gt(4)*D2R,  '$\theta \rightarrow \delta_e$',        'deg/deg');
e.q_de      = mk('q -> profundor',            'deg/(deg/s)', gt(3)*D2R,  '$q \rightarrow \delta_e$',             'deg/(deg/s)');
e.al_de     = mk('alpha -> profundor',        'deg/deg',     gt(2)*D2R,  '$\alpha \rightarrow \delta_e$',        'deg/deg');
e.ith_de    = mk('int(theta) -> profundor',   'deg/(deg s)', git*D2R,    '$\int\theta \rightarrow \delta_e$',    'deg/(deg$\cdot$s)');
e.H_thref   = mk('H -> theta_ref',            'deg/m',       ga(5)*R2D,  '$H \rightarrow \theta_{ref}$',         'deg/m');
e.th_thref  = mk('theta -> theta_ref',        'rad/rad',     ga(4),      '$\theta \rightarrow \theta_{ref}$',    'rad/rad');
e.al_thref  = mk('alpha -> theta_ref',        'rad/rad',     ga(2),      '$\alpha \rightarrow \theta_{ref}$',    'rad/rad');
e.iH_thref  = mk('int(H) -> theta_ref',       'deg/(m s)',   gia*R2D,    '$\int H \rightarrow \theta_{ref}$',    'deg/(m$\cdot$s)');
e.VT_thr    = mk('VT -> manete',              '%/(m/s)',     gv(1),      '$V_T \rightarrow \delta_t$',           '\%/(m/s)');
e.iVT_thr   = mk('int(VT) -> manete',         '%/m',         giv,        '$\int V_T \rightarrow \delta_t$',      '\%/m');
e.phi_da    = mk('phi -> aileron',            'rad/rad',     gl(1,4),    '$\phi \rightarrow \delta_a$',          'rad/rad');
e.p_da      = mk('p -> aileron',              'rad/(rad/s)', gl(1,2),    '$p \rightarrow \delta_a$',             'rad/(rad/s)');
e.r_dr      = mk('r -> leme',                 'rad/(rad/s)', gl(2,3),    '$r \rightarrow \delta_r$',             'rad/(rad/s)');
e.beta_dr   = mk('beta -> leme',              'rad/rad',     gl(2,1),    '$\beta \rightarrow \delta_r$',         'rad/rad');
e.iphi_da   = mk('int(phi) -> aileron',       'rad/(rad s)', gil(1,1),   '$\int\phi \rightarrow \delta_a$',      'rad/(rad$\cdot$s)');
e.psi_phiref= mk('psi -> phi_ref',            'rad/rad',     gp(5),      '$\psi \rightarrow \phi_{ref}$',        'rad/rad');
e.beta_phiref=mk('beta -> phi_ref',           'rad/rad',     gp(1),      '$\beta \rightarrow \phi_{ref}$',       'rad/rad');
e.ipsi_phiref=mk('int(psi) -> phi_ref',       'rad/(rad s)', gip,        '$\int\psi \rightarrow \phi_{ref}$',    'rad/(rad$\cdot$s)');
end
