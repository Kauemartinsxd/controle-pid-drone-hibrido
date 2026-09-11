function [A,B,C,D] = lin_DH(Xe,Ue,coef_Sato,coef_Ana,Variacao_Iner)

n = size(Xe,1);
m = size(Ue,1);

A = zeros(n,n);
B = zeros(n,m);

Y0 = obs_rigidbody_DH(0,Xe,Ue,coef_Sato,coef_Ana,Variacao_Iner);
p = size(Y0,1);

C = zeros(p,n);
D = zeros(p,m);

% Pasos mínimos físicos para los estados
dx_min = [ ...
    1e-3;      % u [m/s]
    1e-3;      % v [m/s]
    1e-3;      % w [m/s]
    1e-5;      % p [rad/s]
    1e-5;      % q [rad/s]
    1e-5;      % r [rad/s]
    1e-5;      % phi [rad]
    1e-5;      % theta [rad]
    1e-5;      % psi [rad]
    1e-2;      % xN [m]
    1e-2;      % xE [m]
    1e-2;      % xD [m]
    1e-7;      % mi
    1e-7];     % lambda

% Pasos mínimos físicos para entradas
du_min = [ ...
    1e-4;      % throttle
    1e-5;      % elevator [rad]
    1e-5;      % aileron [rad]
    1e-5;      % rudder [rad]
    1e-3;      % wind_x [m/s]
    1e-3;      % wind_y [m/s]
    1e-3];     % wind_z [m/s]

rel_step = 1e-4;

%% Matriz A
for j = 1:n
    dxj = zeros(n,1);
    dxj(j) = max(abs(Xe(j))*rel_step, dx_min(j));

    Xpup = dyn_rigidbody_DH(0, Xe + dxj, Ue, coef_Sato, coef_Ana,Variacao_Iner);
    Xpdw = dyn_rigidbody_DH(0, Xe - dxj, Ue, coef_Sato, coef_Ana,Variacao_Iner);

    A(:,j) = (Xpup - Xpdw)/(2*dxj(j));
end

%% Matriz B
for j = 1:m
    duj = zeros(m,1);
    duj(j) = max(abs(Ue(j))*rel_step, du_min(j));

    Xpup = dyn_rigidbody_DH(0, Xe, Ue + duj, coef_Sato, coef_Ana,Variacao_Iner);
    Xpdw = dyn_rigidbody_DH(0, Xe, Ue - duj, coef_Sato, coef_Ana,Variacao_Iner);

    B(:,j) = (Xpup - Xpdw)/(2*duj(j));
end

%% Matriz C
for j = 1:n
    dxj = zeros(n,1);
    dxj(j) = max(abs(Xe(j))*rel_step, dx_min(j));

    Yup = obs_rigidbody_DH(0, Xe + dxj, Ue, coef_Sato, coef_Ana,Variacao_Iner);
    Ydw = obs_rigidbody_DH(0, Xe - dxj, Ue, coef_Sato, coef_Ana,Variacao_Iner);

    C(:,j) = (Yup - Ydw)/(2*dxj(j));
end

%% Matriz D
for j = 1:m
    duj = zeros(m,1);
    duj(j) = max(abs(Ue(j))*rel_step, du_min(j));

    Yup = obs_rigidbody_DH(0, Xe, Ue + duj, coef_Sato, coef_Ana,Variacao_Iner);
    Ydw = obs_rigidbody_DH(0, Xe, Ue - duj, coef_Sato, coef_Ana,Variacao_Iner);

    D(:,j) = (Yup - Ydw)/(2*duj(j));
end

end