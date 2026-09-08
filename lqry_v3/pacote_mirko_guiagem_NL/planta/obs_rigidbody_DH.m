% Autor: SATO, F. C. Y. CURSINO
% ITA - PG/EEC-D
% MODELO MATEMÁTICO COMPLETO DH
% PROGRAMA ADAPTADO DE NOTAS DE AULA AB-266 - PROF. ALMEIDA, F. A.
% MODIFICADO EM: 21/03/2026

function Y = obs_rigidbody_DH(t,X,U,coef_Sato,coef_Ana,Variacao_Iner)

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

wD = U(7);
U(5:7) = Lbn*U(5:7);
wx = U(5);
wy = U(6);
wz = U(7);

%PARÂMETROS DA AERONAVE
%%
VT0 = 12; %VELOCIDADE DE REFERÊNCIA [m/s]UTILIZADA NAS SIMULAÇÕES EM AVL PARA OBTENÇÃO DAS DERIVADAS DE ESTABILIDADE (VETOR THETA)
S   = 0.27; %ÁREA DA ASA [m^2]
m   = 2.2;%MASSA TOTAL DA AERONAVE (CONSIDERAR A CARGA ÚTIL) [kg]
c   = 0.226 ; %CORDA MÉDIA AERODINÂMICA [m]
b   = 1.2; %ENVERGADURA DA ASA [m]
g   = 9.80665;%ACELERAÇÃO DA GRAVIDADE [m/s^2]
pi = 3.14159265;%NÚMERO Pi
e =0.8;
AR = (b^2)/S;%ALONGAMENTO DA ASA


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

% COMANDOS PARA SUPERFÍCIES DE CONTROLE DA AERONAVE

dt    = U(1);%manete trotle
de    = U(2);%elevator
da    = U(3);%aileron
dr    = U(4);%leme

% DADOS ATMOSFÉRICOS, VELOCIDADE REAL E ÂNGULOS DE INCIDÊNCIA

VT    = sqrt((u-wx)^2 + (v-wy)^2 + (w-wz)^2 ); %Aerodynamic velocity
alpha = atan((w-wz)/(u-wx));
beta  = asin((v-wy)/VT);

T    = 288.15*(1-6.5e-3*(-xD)/288.15);
rho  = 1013.25e2*(1-6.5e-3*(-xD)/288.15)^(5.2561)/(287.3*T);


%%DEFINIÇÃO DO VETOR DE DERIVADAS DE ESTABILIDADE OBTIDO NO AVL(THETA - CONFORME
%%PROGRAMA DE IDENTIFICAÇAO OEM)

coef_DH;%UTILIZAR O ARQUIVO COEF QUE REPRESENTA O MODELO AERODINÂMICO COMPLETO DA AERONAVE DH

%%

qbar = 0.5*rho*VT^2;% PRESSÃO DINÂMICA

Cm   = Cm0 + Cmalpha*alpha + Cmde*de + Cmq*q*c/(2*VT0);% COEFICIENTE DE MOMENTO DE ARFAGEM
qp = qbar*S*Cm/Iy;%ACELERAÇÃO ANGULAR DE ARFAGEM [rad/s]


%Gamma - RELAÇÃO ENTRE O ÂNGULO DE ATAQUE E O ÂNGULO DE ARFAGEM

xNpxEpxDp = Lnb*[u v w]';
xDp       = xNpxEpxDp(3);
gamma     = asin((-xDp-wD)/VT);

% CÁLCULO DE FORÇAS E MOMENTOS

[Ti,Fext,Mext,m,axayaz] = modelo_DH(qbar,VT,alpha,beta,X,U,rho,coef_Sato,coef_Ana,Variacao_Iner);


% SAÍDAS OBSERVADAS

Y =  [VT
    alpha
    beta
    gamma
    p
    q
    r
    phi
    theta
    psi
    axayaz
    xN
    -xD
    mi
    lambda
    qp];


end

