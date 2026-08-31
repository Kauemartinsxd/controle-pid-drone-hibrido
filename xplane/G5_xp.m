% G5_xp.m — lado X-Plane da campanha G5: voa UMA manobra no gemeo v1.1.
% =============================================================
% Uso (uma manobra por reload do aviao):
%   1) X-Plane: File -> Open Aircraft -> DH-Lon-REV-03  (religa o motor)
%   2) G5_id = 'dblh';    G5_xp      % ou 'dblpsi' | 'missao'
%   3) plot_G5(G5_id)                % figura comparada + metricas
%
% E' um script (nao funcao) porque o XP_voo opera no workspace base.
% =============================================================
if ~exist('G5_id','var') || isempty(G5_id)
    error('G5_xp: defina G5_id = ''dblh'' | ''dblpsi'' | ''missao'' antes.');
end
G5_M = G5_manobras_def();
G5_m = G5_M(strcmp({G5_M.id}, G5_id));
if isempty(G5_m), error('G5_xp: manobra "%s" desconhecida.', G5_id); end
setpref('XP_DH','g5id', G5_id);   % sobrevive ao clear do XP_voo

XP_TimeXP        = G5_m.T;
XP_h_step_final  = G5_m.h_A;
XP_h_step_t      = G5_m.h_t(1);
XP_h_step_t2     = G5_m.h_t(2);
XP_h_step_t3     = G5_m.h_t(3);
XP_psi_step_deg  = G5_m.psi_A;
XP_psi_step_t    = G5_m.psi_t(1);
XP_psi_step_t2   = G5_m.psi_t(2);
XP_psi_step_t3   = G5_m.psi_t(3);
XP_VT_step_delta = G5_m.VT_delta;
XP_VT_step_t     = G5_m.VT_t;

XP_voo   % pre-flight + teleporte-engate + voo + save em voos/XP_voo_*.mat

G5_id = getpref('XP_DH','g5id'); rmpref('XP_DH','g5id');
fprintf('\nG5_xp (%s): voo em %s\nAgora rode:  plot_G5(''%s'')\n', G5_id, vooFile, G5_id);
