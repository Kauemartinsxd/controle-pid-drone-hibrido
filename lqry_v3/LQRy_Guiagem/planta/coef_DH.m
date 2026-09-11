% Modiicado por: Huascar Mirko Montecinos Cortez em 23/04/2026

%% Stability-axis derivatives...
%% Sato
%%Longitudinal _ Identification M1

if coef_Sato == 1

    Cl0 = 0.361632;
    Cd0 = 0.0813815;
    Clalpha = 6.98729;
    Cmalpha = -1.34314;
    Clq = 15.4571;
    Cm0 = 0.108861;
    Cmq = -40.2966;
    Cmde = 3.46691;

    %latero-direcional_Mfinal

    Cyb = 0.609958;
    Clb = -0.1036037;
    Cnb = 0.127030;
    Cyp = 5.00530;
    Clp = -0.785502;
    Cnp = -0.326726;
    Cyr = 11.731150;
    Clr = 0.433016;
    Cnr = -0.781478;
    Clda = -0.278348;
    Cnda = 0.127621;
    Cyda = 0.227152;
    Cydr = -0.642703;
    Cndr = 0.097487;
    Cldr = 0.854259;

end

%% Ana
% Modelo identificado - Drone Hibrido
% Criado por Ana Carolina de Lima Angelo
% Data: 24/04/2026

% Longitudinal

if coef_Ana == 1

    Cl0 = 1.93615e-01;
    Cd0 = 1.00383e-01;
    Clalpha = 2.87836e+00;
    Cmalpha = -8.81949e-01;
    Clq = 3.11023e+01;
    Cm0 = 8.30091e-02;
    Cmq = -9.92909e+00;
    Cmde = 1.04662e+00;

    % Latero-direcional

    Cyb = -3.98966e-01;
    Clb = -2.77332e-01;
    Cnb = 1.53474e-01;
    Cyp = -6.93329e-01;
    Clp = -1.32871e+00;
    Cnp = -1.08353e-01;
    Cyr = 3.66157e-02;
    Clr = 1.00433e+00;
    Cnr = -4.61062e-01;
    Clda = -7.10799e-01;
    Cnda = 3.95561e-02;
    Cyda = 0;
    Cydr = 4.65883e-02;
    Cndr = 1.82983e-01;
    Cldr = -6.90524e-02;

end
