% Autor: SATO, F. C. Y. CURSINO
% ITA - PG/EEC-D
% MODELO MATEMÁTICO COMPLETO NÃO-LINEAR DH (DRONE HÍBRIDO)
% PROGRAMA ADAPTADO DE NOTAS DE AULA AB-266 - PROF. ALMEIDA, F. A.
% MODIFICADO EM: 21/03/2026

% Modiicado por: Huascar Mirko Montecinos Cortez em 23/04/2026

function [Ti,Fext,Mext,m,axayaz] = modelo_DH(qbar,VT,alpha,beta,X,U,rho,coef_Sato,coef_Ana,Variacao_Iner)
%%Input vector

%Variable declaration
%%
p     = X(4);
q     = X(5);
r     = X(6);
phi   = X(7);
theta = X(8);

dt    = U(1);%manete trotle
de    = U(2);%elevator
da    = U(3);%aileron
dr    = U(4);%leme


%Lwb Matriz
%%
Lwb = [cos(alpha)*cos(beta)   sin(beta)   sin(alpha)*cos(beta)
      -cos(alpha)*sin(beta)   cos(beta)  -sin(alpha)*sin(beta)
      -sin(alpha)             0           cos(alpha)];

Lbw = Lwb';
%
%Aircraft's parameters
%%
VT0 = 12; %Reference speed [m/s]used to AVL's simulation
S   = 0.27; %area da asa 
m   = 2.2; %massa da aeronave kg
c   = 0.226 ; %corda média aerodinamica
b   = 1.2; %envergadura 
g   = 9.80665;
pi = 3.14159265;
e =0.8;
AR = (b^2)/S;

%%
% ============================================================
% Matriz de inercia
% ============================================================
Ti_nominal = [ 0.14410    0       -0.00167;
               0          0.11550  0;
              -0.00167    0        0.25716 ];
% ============================================================
% Variacao_Iner = 0  -> Nominal
% Variacao_Iner = 1  -> -10 %
% Variacao_Iner = 2  -> +10 %
% ============================================================
if Variacao_Iner == 0
    % Caso nominal
    Ti = Ti_nominal;
elseif Variacao_Iner == 1
    % Reducción del 10 %
    Ti = 0.90 * Ti_nominal;
elseif Variacao_Iner == 2
    % Aumento del 10 %
    Ti = 1.90 * Ti_nominal;
end
% 
Iy = Ti(2,2);
  
%%coeficientes aerodinâmicos
coef_DH; %UTILIZAR O ARQUIVO COEF QUE REPRESENTA O MODELO AERODINÂMICO COMPLETO DA AERONAVE DH

%Forces
 %%
 CL     = Cl0 + Clalpha*alpha + Clq*(q*c/(2*VT0));
 CD     = Cd0 + (1/(pi*e*AR))*CL^2;
 CY     = Cyb*beta + Cydr*dr + Cyp*p*b/(2*VT0) + Cyr*r*b/(2*VT0);
 F      = 14*dt*(rho/1.225)^0.8;
 axayaz = (Lbw*qbar*S*[-CD CY -CL]' + [F 0 0]')/m;

 Fext   = m*axayaz + m*g*[-sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)]'; %Fext   = m*axayaz + m*g*[-sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)]';
 
 %Moments
 %%
 Cl   = Clb*beta + Clp*p*b/(2*VT0) + Clr*r*b/(2*VT0) + Clda*da + Cldr*dr;
 Cm   = Cm0 + Cmalpha*alpha + Cmde*de + Cmq*q*c/(2*VT0);
 Cn   = Cnb*beta + Cnp*p*b/(2*VT0) + Cnr*r*b/(2*VT0) + Cnda*da + Cndr*dr;
 Mext = qbar*S*[b*Cl c*Cm b*Cn]';
 qp = qbar*S*Cm/Iy;
 
end
