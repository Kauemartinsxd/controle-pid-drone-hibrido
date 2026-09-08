function OUT = lqry_v3_casos_artigo(varargin)
% LQRY_V3_CASOS_ARTIGO  Roda os casos do artigo do Mirko (Tabela 11: 8 combinacoes de
% Velocity Hold / phi-psi Hold / theta-H Hold; Fig. 38 / Tab. 12: Caso 4 com variacao de
% inercia) no SIL dele (CL_NL_DH_18_jun_2026, planta NL da Ana, atuador ideal, Cond. 5 =
% 15 m/s @ 600 m, refs de 5, 150 s) com os ganhos v3 — para comparar com as corridas dos
% ganhos ORIGINAIS ja reproduzidas em reproducao_SIL\caso*_nominal.mat / caso4_iner*.mat.
%
%   OUT = lqry_v3_casos_artigo                      % 8 casos + 3 variantes de inercia
%   OUT = lqry_v3_casos_artigo('casos', 1:2)        % so' os casos 1 e 2
%   OUT = lqry_v3_casos_artigo('casos', [], 'inercia', [1 2 3])
%
% Harness do v3 (estrutura intacta): so' a condicao inicial do integrador de H no
% engate (lqry_v3_prepara_modelo 'ic_bumpless'); doublets e pulso de V_T do artigo
% mantidos. Mesma logica de refs do Control_LQRy_DH.m dele (refAlt = 0 nos casos 6 e 8).
% Saida: lqry_v3/sil/casos_artigo/caso<k>_v3.mat e caso4_iner<v>_v3.mat (formato S do
% lqry_sim_variant: theta_NL, q_NL, elev_NL, VT_NL, Throttle_NL, H_NL, phi_NL, p_NL,
% ail_NL, psi_NL, r_NL, rud_NL, beta_NL, cfg, nan).
p = inputParser;
p.addParameter('casos', 1:8);
p.addParameter('inercia', []);          % variantes do Caso 4: 1 = x0.90 | 2 = x1.90 ("+10%" do codigo dele) | 3 = x1.10
p.addParameter('T', 150);
p.addParameter('i', 5);
p.addParameter('refs', 5);
p.addParameter('pular_existentes', true);
p.parse(varargin{:}); o = p.Results;

here = fileparts(mfilename('fullpath'));
mirko = 'C:\Users\kaue\Documents\Dissertacao_Mestrado\lqry_mirko_atualizado\reproducao_SIL\mirko_run';
outdir = fullfile(here, 'sil', 'casos_artigo'); if ~exist(outdir, 'dir'), mkdir(outdir); end
mdl = 'CL_NL_DH_18_jun_2026';
Tabela_casos = [1 1 1; 1 1 0; 1 0 1; 1 0 0; 0 1 1; 0 1 0; 0 0 1; 0 0 0];   % [VT_Throttle phi_psi att_alt]

% --- setup do Mirko (path isolado, Caso 4, i=5) + ganhos v3 no base ---
evalin('base', sprintf('run(''%s'')', fullfile(mirko, 'lqry_setup.m')));
addpath(here);
for f = {'Ganho_hold_theta','Ganho_hold_H','Ganho_hold_VT','Ganho_hold_phi','Ganho_hold_psi'}
    s = load(fullfile(here, 'ganhos', [f{1} '.mat'])); fn = fieldnames(s);
    for k = 1:numel(fn), assignin('base', fn{k}, s.(fn{k})); end
end
P = evalin('base', 'Plantas');
assignin('base', 'i', o.i); assignin('base', 'Ve', P(o.i).Ve); assignin('base', 'he', P(o.i).He);
assignin('base', 'Xe', P(o.i).Xe); assignin('base', 'Ue', P(o.i).Ue);
assignin('base', 'coef_Ana', 1); assignin('base', 'coef_Sato', 0);
assignin('base', 'refPhi', o.refs); assignin('base', 'refPsi', o.refs); assignin('base', 'reftheta', o.refs);
global HYB; HYB = [];

lista = {};
for c = o.casos(:)'
    lista(end+1,:) = {sprintf('caso%d_v3', c), c, 0}; %#ok<AGROW>
end
for v = o.inercia(:)'
    lista(end+1,:) = {sprintf('caso4_iner%d_v3', v), 4, v}; %#ok<AGROW>
end

OUT = struct();
for k = 1:size(lista, 1)
    tag = lista{k,1}; caso = lista{k,2}; iner = lista{k,3};
    arq = fullfile(outdir, [tag '.mat']);
    if o.pular_existentes && exist(arq, 'file'), fprintf('  [%s] ja existe, pulando\n', tag); d = load(arq); OUT.(tag) = d.S; continue; end
    assignin('base', 'VT_Throttle', Tabela_casos(caso,1)); assignin('base', 'phi_psi', Tabela_casos(caso,2)); assignin('base', 'att_alt', Tabela_casos(caso,3));
    if caso == 1 || caso == 3, assignin('base', 'refVel', 12.2); else, assignin('base', 'refVel', 15.2); end   % inerte no modelo v2
    if caso == 6 || caso == 8, assignin('base', 'refAlt', 0);    else, assignin('base', 'refAlt', o.refs); end
    assignin('base', 'Variacao_Iner', iner);
    bdclose(mdl); load_system(mdl);
    lqry_v3_prepara_modelo(mdl, struct('ic_bumpless', 1, 'fim_auto', 0, 'vt_pulse_off', 0, 'i', o.i));
    fprintf('[%s] caso %d (VT_Throttle=%d phi_psi=%d att_alt=%d refAlt=%g iner=%d) ... ', tag, caso, ...
        Tabela_casos(caso,1), Tabela_casos(caso,2), Tabela_casos(caso,3), evalin('base','refAlt'), iner);
    t0 = tic;
    try
        out = sim(mdl, 'SrcWorkspace', 'base', 'StopTime', num2str(o.T));
        S = struct();
        for nm = {'theta_NL','q_NL','elev_NL','VT_NL','Throttle_NL','H_NL','phi_NL','p_NL','ail_NL','psi_NL','r_NL','rud_NL','beta_NL'}
            v = out.get(nm{1}); S.(nm{1}) = struct('time', v.time, 'values', v.signals.values);
        end
        S.tout = out.tout; S.erro = '';
    catch me
        S = struct('erro', me.message, 'tout', []);
    end
    bdclose(mdl);
    S.t_cpu = toc(t0); S.tag = tag; S.ganhos = 'v3';
    S.cfg = struct('i', o.i, 'Variacao_Iner', iner, 'VT_Throttle', Tabela_casos(caso,1), 'phi_psi', Tabela_casos(caso,2), ...
        'att_alt', Tabela_casos(caso,3), 'refPhi', o.refs, 'refPsi', o.refs, 'refAlt', evalin('base','refAlt'), ...
        'reftheta', o.refs, 'coef_Ana', 1, 'coef_Sato', 0, 'caso', caso);
    if isempty(S.erro)
        S.nan = any(isnan(S.theta_NL.values(:)));
        fprintf('%.0f s | NaN=%d | theta[%.1f,%.1f] VT[%.1f,%.1f] H[%.1f,%.1f] phi[%.1f,%.1f] psi[%.1f,%.1f] de[%.1f,%.1f] thr[%.2f,%.2f]\n', ...
            S.t_cpu, S.nan, min(S.theta_NL.values(:,2)), max(S.theta_NL.values(:,2)), min(S.VT_NL.values(:,2)), max(S.VT_NL.values(:,2)), ...
            min(S.H_NL.values(:,2)), max(S.H_NL.values(:,2)), min(S.phi_NL.values(:,2)), max(S.phi_NL.values(:,2)), ...
            min(S.psi_NL.values(:,1)), max(S.psi_NL.values(:,1)), min(S.elev_NL.values(:,1)), max(S.elev_NL.values(:,1)), ...
            min(S.Throttle_NL.values(:,1)), max(S.Throttle_NL.values(:,1)));
    else
        S.nan = true; fprintf('ERRO: %s\n', S.erro);
    end
    save(arq, 'S'); OUT.(tag) = S;
end
% volta ao Caso 4 e desfaz o setup
assignin('base', 'VT_Throttle', 1); assignin('base', 'phi_psi', 0); assignin('base', 'att_alt', 0);
assignin('base', 'refAlt', o.refs); assignin('base', 'Variacao_Iner', 0);
evalin('base', sprintf('run(''%s'')', fullfile(mirko, 'lqry_teardown.m')));
end
