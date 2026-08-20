% XP_condicao_inicial.m
% =============================================================
% Teleporta o DH no X-Plane para uma condicao inicial de VOO:
% altitude AGL desejada, voo nivelado, VETOR DE VELOCIDADE ja
% definido (sem isso a aeronave estolaria e cairia apos o
% teleporte). Adaptado de ins_initial_state_xplane.m (Julio).
%
% Sequencia (padrao do Julio):
%   1) pausa a fisica do X-Plane
%   2) sendPOSI  — lat/lon atuais, altitude alvo, atitude nivelada
%   3) sendDREF  — local_vx/vy/vz (frame OpenGL: x=Leste, y=Cima,
%                  z=Sul) coerentes com VT0 na proa escolhida
%   4) sendCTRL  — superficies neutras + throttle inicial
%   5) despausa
%
% FLUXO DE USO (encadeie sem pausa entre os comandos):
%   XP_condicao_inicial
%   XP_inicializacao          % captura o trim = controles do passo 4
%   out = sim('modelo_XP_DH_CL');
%
% Config: defina antes de rodar (ou aceite os defaults abaixo)
%   XP_h0_agl [m]   altitude AGL inicial          (default 100)
%   XP_VT0    [m/s] velocidade inicial            (default 12 = Ve)
%   XP_psi0   [deg] proa inicial                  (default NaN = manter atual)
%   XP_thr0   [-]   throttle inicial [0,1]        (default NaN = Ue(1) do
%                   modelo matematico se existir no workspace, senao 0.5)
% =============================================================

%% Defaults
if ~exist('XP_h0_agl','var') || isempty(XP_h0_agl), XP_h0_agl = 100;  end
if ~exist('XP_VT0','var')    || isempty(XP_VT0),    XP_VT0    = 12;   end
if ~exist('XP_psi0','var'),                         XP_psi0   = NaN;  end
if ~exist('XP_thr0','var'),                         XP_thr0   = NaN;  end
if isnan(XP_thr0)
    if exist('Ue','var') && numel(Ue) >= 1
        XP_thr0 = Ue(1);       % trim de throttle do modelo matematico
    else
        XP_thr0 = 0.5;
    end
end

%% Paths / conexao
xpDir = fileparts(mfilename('fullpath'));
addpath(xpDir);
addpath(fullfile(fileparts(fileparts(xpDir)), 'trabalho_julio', ...
    'PIPER-1-6-roll_back', 'PIPER-1-6-roll_back', 'xplane', ...
    'XPlaneConnect-master', 'MATLAB'));
import XPlaneConnect.*
global GlobalSocket
if isempty(GlobalSocket)
    GlobalSocket = openUDP('127.0.0.1', 49009);
end

%% Estado atual (lat/lon/solo e proa)
drefs = {'sim/flightmodel/position/latitude', ...
         'sim/flightmodel/position/longitude', ...
         'sim/flightmodel/position/elevation', ...   % MSL [m]
         'sim/flightmodel/position/y_agl', ...       % AGL [m]
         'sim/flightmodel/position/psi'};            % proa [deg]
st = double(getDREFs(drefs, GlobalSocket));
ground_msl = st(3) - st(4);
target_msl = ground_msl + XP_h0_agl;
if isnan(XP_psi0), XP_psi0 = st(5); end

%% Teleporte com fisica pausada
pauseSim(1, GlobalSocket);
pause(0.2);

% Posicao/atitude: [lat, lon, MSL, pitch, roll, heading, gear(-998 = nao mexe)]
sendPOSI([st(1), st(2), target_msl, 0, 0, XP_psi0, -998], 0, GlobalSocket);
pause(0.2);

% Vetor de velocidade no frame local OpenGL (x=Leste, y=Cima, z=Sul)
hdg = deg2rad(XP_psi0);
sendDREF('sim/flightmodel/position/local_vx',  XP_VT0*sin(hdg), GlobalSocket);
sendDREF('sim/flightmodel/position/local_vy',  0,               GlobalSocket);
sendDREF('sim/flightmodel/position/local_vz', -XP_VT0*cos(hdg), GlobalSocket);
pause(0.2);

% Controles: neutro + throttle inicial (vira o TrimInput capturado
% pela XP_inicializacao logo em seguida)
sendCTRL([0, 0, 0, XP_thr0, -998, -998], 0, GlobalSocket);
pause(0.2);

% NAO despausa: o X-Plane fica CONGELADO na condicao inicial. Quem
% despausa e' o 1o sample do modelo Simulink (read_xp -> xp_read_dh(1)),
% que tambem re-zera a proa relativa. Tempo em malha aberta ~= 0.
fprintf(['XP_condicao_inicial: DH ARMADO (pausado) a %.0f m AGL, ' ...
    'VT = %.1f m/s, proa = %.0f deg, throttle = %.2f.\n' ...
    'Rode a sim para engatar — o modelo despausa o X-Plane sozinho.\n'], ...
    XP_h0_agl, XP_VT0, XP_psi0, XP_thr0);
clear XP_h0_agl XP_VT0 XP_psi0 XP_thr0
