% Autor: SATO, F. C. Y. CURSINO
% ITA - PG/EEC-D
% MODELO MATEMÁTICO COMPLETO NÃO-LINEAR DH
% PROGRAMA ADAPTADO DE NOTAS DE AULA AB-266 - PROF. ALMEIDA, F. A.
% MODIFICADO EM: 21/04/2026


function [sys,x0,str,ts,simStateCompliance]=sfunction_DH(t,x,u,flag,Xe,coef_Sato,coef_Ana,Variacao_Iner)

switch flag

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0         
    [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(Xe);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1
    sys = dyn_rigidbody_DH(t,x,u,coef_Sato,coef_Ana,Variacao_Iner);

  %%%%%%%%%%%%%%%%%%%%%%%%
  % Update and Terminate %
  %%%%%%%%%%%%%%%%%%%%%%%%
  case {2,9}
    sys = []; % do nothing

  %%%%%%%%%%
  % Output %
  %%%%%%%%%%
  case 3
    sys = obs_rigidbody_DH(t,x,u,coef_Sato,coef_Ana,Variacao_Iner); 

  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(Xe)

sizes = simsizes;
sizes.NumContStates  = 14;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 18;
sizes.NumInputs      = 7;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
str = [];
x0  = Xe;
ts  = [0 0];   % sample time: [period, offset]

% speicfy that the simState for this s-function is same as the default
simStateCompliance = 'DefaultSimState';



