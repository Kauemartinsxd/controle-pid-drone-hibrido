% MODELO MATEMÁTICO COMPLETO NÃO-LINEAR DH
% PROGRAMA ADAPTADO DE NOTAS DE AULA AB-266 - PROF. ALMEIDA, F. A.
% MODIFICADO EM: 21/04/2020               
% SATO F C Y C, ITA 2026


function [Xe,Ue,xe] = trimagem_DH(Ve,he,gammae,coef_Sato,coef_Ana,Variacao_Iner)%%

x0 = [0*pi/180 0 0];      %x0 = [alphae dte dee] % alphae - ângulo de ataque [rad]; dte - manete para voo em equilíbrio; dee - deflexão do profundor para voo em equilíbrio
f  = @(x)strflt2(x,Ve,he,gammae,coef_Sato,coef_Ana,Variacao_Iner);
xe = fsolve(f,x0);

function f = strflt2(y,Ve,he,gammae,coef_Sato,coef_Ana,Variacao_Iner)
                
ae  = y(1);
dte = y(2);
dee = y(3);

Xe = [Ve*cos(ae)   % u [m/s]    
      0            % v [m/s]
      Ve*sin(ae)   % w [m/s]
      0            % p [rad/s]
      0            % q
      0            % r
      0            % phi [rad]
      ae + gammae  % theta
      0            % psi
      0            % xN
      0            % xE
      -he          % -xD
      0            % mi
      0];          % lambda

% cálculo dos comandos (deflexões) e perturbações no equilíbrio
Ue = [dte %dte
      dee %dee
      0     %dae
      0     %dre
      0     %wxe
      0     %wye
      0];   %wze

  Xpontoe = dyn_rigidbody_DH(0,Xe,Ue,coef_Sato,coef_Ana,Variacao_Iner);
  
  f = [Xpontoe(1)
      Xpontoe(3)
      Xpontoe(5)];
  
end
    

end
