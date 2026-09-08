function OUT = lqry_v3_sil_check(varargin)
% LQRY_V3_SIL_CHECK  Roda o SIL do Mirko (CL_NL_DH_18_jun_2026, planta NL da Ana
% ou do Sato) com os ganhos ORIGINAIS ou os v3, em plantas "ideal" (como no
% artigo) ou "real" (motor tau 3,5 s + curso +-15 deg + efeito de potencia
% Cm_thr 0,25 medidos no X-Plane). Responde a pergunta do Kaue: por que o
% LQRy original aguentou o modelo do Sato e nao aguenta o X-Plane?
%
%   OUT = lqry_v3_sil_check                       % matriz completa (8 corridas)
%   OUT = lqry_v3_sil_check('casos', {'orig_ana_ideal','v3_ana_real'}, 'T', 100)
%
% Cada corrida: Caso 4 (VT Hold + psi Hold + Alt Hold), i = 5 (15 m/s), refs do
% artigo (VT +3 em 10-20 s, H +-5 m em 40/60/80, psi +-5 deg em 90/110/130).
% Resultado em OUT.(caso): sinais, NaN?, extremos, % de saturacao de manete.
% Salva lqry_v3/sil/<caso>.mat.
p = inputParser;
p.addParameter('casos', {'orig_ana_ideal','orig_sato_ideal','orig_ana_real','orig_sato_real', ...
                         'v3_ana_ideal','v3_sato_ideal','v3_ana_real','v3_sato_real'});
p.addParameter('T', 150);
p.addParameter('i', 5);
p.addParameter('refs', 5);
p.addParameter('tau_motor', 3.5);
p.addParameter('de_lim_deg', 15);
p.addParameter('Cm_thr', 0.25);
p.addParameter('salvar', true);
p.parse(varargin{:}); o = p.Results;

here = fileparts(mfilename('fullpath'));
v3dir = fullfile(here, 'ganhos');
mirko = 'C:\Users\kaue\Documents\Dissertacao_Mestrado\lqry_mirko_atualizado\reproducao_SIL\mirko_run';
raizN = 'C:\Users\kaue\Documents\Dissertacao_Mestrado\lqry_mirko_atualizado\Nova pasta';
outdir = fullfile(here, 'sil'); if ~exist(outdir, 'dir'), mkdir(outdir); end
mdl = 'CL_NL_DH_18_jun_2026';

% --- setup do Mirko (path isolado, ganhos originais, Caso 4, i=5) ---
evalin('base', sprintf('run(''%s'')', fullfile(mirko, 'lqry_setup.m')));
addpath(here);                       % o lqry_setup tira do path tudo que e' do repo PID (inclusive lqry_v3)
assignin('base', 'i', o.i);
P = evalin('base', 'Plantas');
for v = {'Ve','he','Xe','Ue'}
    switch v{1}
        case 'Ve', assignin('base', 'Ve', P(o.i).Ve);
        case 'he', assignin('base', 'he', P(o.i).He);
        case 'Xe', assignin('base', 'Xe', P(o.i).Xe);
        case 'Ue', assignin('base', 'Ue', P(o.i).Ue);
    end
end
assignin('base', 'refAlt', o.refs); assignin('base', 'refPsi', o.refs); assignin('base', 'refPhi', o.refs); assignin('base', 'reftheta', o.refs);
assignin('base', 'VT_Throttle', 1); assignin('base', 'phi_psi', 0); assignin('base', 'att_alt', 0); assignin('base', 'Variacao_Iner', 0);
global HYB
OUT = struct();
for c = 1:numel(o.casos)
    caso = o.casos{c};
    tk = strsplit(caso, '_'); ganhos = tk{1}; planta = tk{2}; atu = tk{3};
    % ganhos
    if strcmp(ganhos, 'v3'), gd = v3dir; else, gd = raizN; end
    for f = {'Ganho_hold_theta','Ganho_hold_H','Ganho_hold_VT','Ganho_hold_phi','Ganho_hold_psi'}
        s = load(fullfile(gd, [f{1} '.mat'])); fn = fieldnames(s);
        for k = 1:numel(fn), assignin('base', fn{k}, s.(fn{k})); end
    end
    % planta aerodinamica
    if strcmp(planta, 'sato'), assignin('base', 'coef_Ana', 0); assignin('base', 'coef_Sato', 1);
    else,                      assignin('base', 'coef_Ana', 1); assignin('base', 'coef_Sato', 0); end
    % atuadores
    HYB = [];
    if strcmp(atu, 'real')
        HYB = struct('k_Iroll', 1, 'k_Iyaw', 1, 'k_Clda', 1, 'k_Cnb', 1, 'k_Clb', 1, 'k_Clp', 1, 'k_Cnr', 1, ...
                     'de_lim', o.de_lim_deg*pi/180, 'Cm_thr', o.Cm_thr);
    end
    bdclose(mdl); load_system(mdl);
    if strcmp(atu, 'real')
        set_param([mdl '/throttle 1'], 'Denominator', sprintf('[%g 1]', o.tau_motor));   % motor lento (em memoria)
    end
    if strcmp(ganhos, 'v3')
        % ESTRUTURA DO MIRKO INTACTA: so' a condicao inicial do integrador de H (engate
        % bumpless, harness). O pulso de V_T e os doublets do artigo sao mantidos.
        lqry_v3_prepara_modelo(mdl, struct('ic_bumpless', 1, 'fim_auto', 0, 'vt_pulse_off', 0, 'i', o.i));
    end
    fprintf('[%s] ganhos=%s planta=%s atuadores=%s ... ', caso, ganhos, planta, atu);
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
    S.t_cpu = toc(t0); S.caso = caso; S.cfg = o;
    bdclose(mdl);
    if isempty(S.erro)
        th = S.theta_NL.values(:,2); H = S.H_NL.values(:,2); VT = S.VT_NL.values(:,2); ph = S.phi_NL.values(:,2);
        thr = S.Throttle_NL.values(:,1); el = S.elev_NL.values(:,1);
        S.nan = any(isnan(th)); S.t_nan = NaN; if S.nan, S.t_nan = S.tout(find(isnan(th), 1)); end
        S.sat_thr = 100*mean(thr <= 0.005 | thr >= 0.995);
        S.sat_de  = 100*mean(abs(el) >= 14.9);
        S.res = struct('theta', [min(th) max(th)], 'H', [min(H) max(H)], 'VT', [min(VT) max(VT)], 'phi', [min(ph) max(ph)], ...
                       'elev', [min(el) max(el)], 'thr', [min(thr) max(thr)]);
        fprintf('%.0f s | NaN=%d (t=%.1f) | theta %.1f..%.1f | H %.0f..%.0f | VT %.1f..%.1f | phi %.1f..%.1f | de %.1f..%.1f | thr %.2f..%.2f | sat thr %.0f%% de %.0f%%\n', ...
            S.t_cpu, S.nan, S.t_nan, S.res.theta, S.res.H, S.res.VT, S.res.phi, S.res.elev, S.res.thr, S.sat_thr, S.sat_de);
    else
        S.nan = true; fprintf('ERRO: %s\n', S.erro);
    end
    OUT.(caso) = S;
    if o.salvar, save(fullfile(outdir, [caso '.mat']), 'S'); end
end
HYB = [];
evalin('base', sprintf('run(''%s'')', fullfile(mirko, 'lqry_teardown.m')));
end
