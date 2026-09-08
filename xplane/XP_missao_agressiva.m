% XP_missao_agressiva.m
% =============================================================
% Circuito AGRESSIVO por waypoints (2026-09-01): mesma guiagem LOS e mesma
% cascata PID do XP_missao, mas um quadrado com curvas de 90 graus, degraus
% de altitude de +-20 m e velocidade alternando entre waypoints — os tres
% eixos excitados ao mesmo tempo, para expor as diferencas entre o X-Plane
% e o modelo NL que o circuito oval (curvas suaves, h e V constantes) quase
% nao excita.
%
% v2 (15/18 m/s, versao da apresentacao): quadrado de 260 m, 1 volta
% (1040 m, ~75 s no X-Plane), V 18 nas pernas de subida e 15 nas de
% descida, R_accept 110 (2R = 220 < 260; raio de curva a 18 m/s ~150 m).
% Engate a 12 m/s (trim do controlador e do modelo NL): a 1a perna acelera.
% v1 (12/15 m/s, 20:14): legs 160 m / R 70 — X-Plane h 576..614, theta 30.
%
% Uso:  XP_missao_agressiva          (X-Plane aberto com o DH; reload
%                                     automatico se XP_auto_reload=true)
% Pos-voo: o XP_missao roda o modelo NL na mesma missao (autoNL) e gera a
% comparacao *_compNL.png em xplane/voos.
% =============================================================
XP_WPs_frame = [ 260    0  620  18;     % reta: sobe 20 m e acelera p/ 18
                 260  260  600  15;     % curva 90 dir: desce 20 m, 15 m/s
                   0  260  620  18;     % curva 90 dir: sobe, 18 m/s
                   0    0  600  15];    % curva 90 dir: desce e termina (origem)
XP_WPs_NE   = [];
XP_R_accept = 110;
XP_TimeXP   = 110;                      % teto de seguranca (fim = 5 s apos o ultimo WP)
if ~exist('XP_VT0','var') || isempty(XP_VT0), XP_VT0 = 12; end
if ~exist('XP_tag','var') || isempty(XP_tag), XP_tag = 'AGR15'; end
run(fullfile(fileparts(mfilename('fullpath')), 'XP_missao.m'));
