function G5_sil(ids)
% G5_sil — lado SIL da campanha G5: roda a(s) manobra(s) no modelo NL
% (planta = equacoes da Ana, controlador PID da dissertacao) e salva
% voos/SIL_PID_G5_<id>.mat com a struct silp (t, VT, phi, theta, psi, h
% em unidades SI/rad convertidas p/ o plot: angulos em rad como no Y).
%
% Uso:
%   G5_sil                 % roda as 3 manobras (dblh, dblpsi, missao)
%   G5_sil('dblpsi')       % so uma
%
% Espelho X-Plane: G5_xp. Figura comparada: plot_G5(id).
if nargin < 1 || isempty(ids), ids = {'dblh','dblpsi','missao'}; end
if ischar(ids), ids = {ids}; end
for k = 1:numel(ids)
    run_uma(ids{k});
end
end

function run_uma(id)
% O clear do DH_inicializacao limpa o workspace desta funcao — o id
% sobrevive via setpref (mesmo truque do XP_voo).
setpref('XP_DH','g5id', id);
here = fileparts(mfilename('fullpath'));            % .../xplane
run(fullfile(fileparts(here), 'DH_inicializacao.m'));
id   = getpref('XP_DH','g5id'); rmpref('XP_DH','g5id');
here = fileparts(mfilename('fullpath'));

M = G5_manobras_def();
m = M(strcmp({M.id}, id));
if isempty(m), error('G5_sil: manobra "%s" desconhecida.', id); end

% excitacoes (sobre os defaults inertes do DH_inicializacao)
h_step_final  = m.h_A;            h_step_t   = m.h_t(1);   %#ok<NASGU>
h_step_t2     = m.h_t(2);         h_step_t3  = m.h_t(3);   %#ok<NASGU>
psi_ref_final = deg2rad(m.psi_A); psi_ref_t  = m.psi_t(1); %#ok<NASGU>
psi_ref_t2    = m.psi_t(2);       psi_ref_t3 = m.psi_t(3); %#ok<NASGU>
VT_step_delta = m.VT_delta;       VT_step_t  = m.VT_t;     %#ok<NASGU>

fprintf('G5_sil: simulando "%s" (%s, %g s) no modelo NL...\n', id, m.titulo, m.T);
% SrcWorkspace current: o DH_inicializacao rodou NESTA funcao — sem isso
% o sim resolveria os parametros no workspace base (vazio/errado).
out = sim('modelo_NL_DH_CL', 'StopTime', num2str(m.T), 'SrcWorkspace', 'current');

Ys = out.Y.signals.values;   % [VT alpha beta gamma p q r phi theta psi ... h(15)]
silp = struct('t', out.tout(:), 'VT', Ys(:,1), 'phi', Ys(:,8), ...
    'theta', Ys(:,9), 'psi', Ys(:,10), 'h', Ys(:,15), 'manobra', m);
matFile = fullfile(here, 'voos', ['SIL_PID_G5_' id '.mat']);
save(matFile, 'silp');
fprintf('G5_sil: salvo %s\n', matFile);
end
