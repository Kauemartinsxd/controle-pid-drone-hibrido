% Autor: SATO, F. C. Y. CURSINO
% ITA - PG/EEC-D
% MODELO MATEMÁTICO COMPLETO NÃO-LINEAR DH
% PROGRAMA ADAPTADO DE NOTAS DE AULA AB-266 - PROF. ALMEIDA, F. A.
% MODIFICADO EM: 21/03/2026 


function Xp = dyn_rigidbody_DH(t,X,U,coef_Sato,coef_Ana,Variacao_Iner)

%DECLARAÇÃO DE VARIAVEIS: ESTADOS, CONTROLES E PERTURBAÇÕES


u     = X(1);
v     = X(2);
w     = X(3);
p     = X(4);
q     = X(5);
r     = X(6);
phi   = X(7);
theta = X(8);
psi   = X(9);
xN    = X(10);
xE    = X(11);
xD    = X(12);
mi    = X(13);
lambda= X(14);

%Lbn Matrix

Lbn = [cos(theta)*cos(psi)                            cos(theta)*sin(psi)                            -sin(theta)
       sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta)
       cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];

Lnb = Lbn';

%PERTURBAÇÕES DO VENTO

U(5:7) = Lbn*U(5:7);
wx = U(5);
wy = U(6);
wz = U(7);

% DADOS ATMOSFÉRICOS, VELOCIDADE REAL E ÂNGULOS DE INCIDÊNCIA

VT    = sqrt((u-wx)^2 + (v-wy)^2 + (w-wz)^2 ); %VELOCIDADE AERODINÂMICA REAL
alpha = atan((w-wz)/(u-wx));% ÂNGULO DE ATAQUE
beta  = asin((v-wy)/VT);% ÂNGULO DE DERRAPAGEM (SIDE SLIP ANGLE)

T    = 288.15*(1-6.5e-3*(-xD)/288.15);% TEMPERATURA EM FUNÇÃO DA ALTITUDE
rho  = 1013.25e2*(1-6.5e-3*(-xD)/288.15)^(5.2561)/(287.3*T);% DENSIDADE DO AR EM FUNÇÃO DA ATITUDE

qbar = 0.5*rho*VT^2; % PRESSÃO DINÂMICA

% CÁLCULO DE FORÇAS E MOMENTOS

[Ti,Fext,Mext,m] = modelo_DH(qbar,VT,alpha,beta,X,U,rho,coef_Sato,coef_Ana,Variacao_Iner); 
Fx  = Fext(1);
Fy  = Fext(2);
Fz  = Fext(3);
L   = Mext(1);
M   = Mext(2);
N   = Mext(3);
Ixx = Ti(1,1);
Iyy = Ti(2,2);
Izz = Ti(3,3);
Ixz = -Ti(1,3);
Rt  = 6378137; %METROS


%EQUAÇÕES DE FORÇA

up = Fx/m - q*w + r*v;
vp = Fy/m - r*u + p*w;
wp = Fz/m - p*v + q*u;

%EQUAÇÕES DO MOVIMENTO


  ppqprp = inv(Ti)*[(L + (Iyy-Izz)*q*r + Ixz*p*q)
                      (M + (Izz-Ixx)*p*r + Ixz*(r^2-p^2))
                      (N + (Ixx-Iyy)*p*q - Ixz*q*r)];
              
%EQUAÇÕES DE CINEMÁTICA

phip   = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
thetap = q*cos(phi) - r*sin(phi);
psip   = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);

%EQUAÇÕES DE NAVEGAÇÃO

xNpxEpxDp = Lnb*[u v w]';
mip       = xNpxEpxDp(1)/Rt;
lambdap   = xNpxEpxDp(2)/(Rt*cos(mi));

%
Xp = [up
      vp
      wp
      ppqprp
      phip
      thetap
      psip
      xNpxEpxDp
      mip
      lambdap];

end
