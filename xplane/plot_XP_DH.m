% plot_XP_DH.m
% =============================================================
% Figuras de uma corrida do DH no X-Plane (apos out = sim('modelo_XP_DH_CL')).
% Usa: out.Y_xp (10x1xT), out.U_xp (Tx4), out.t_xplane_log (T).
% =============================================================

Y  = squeeze(out.Y_xp);            % 10 x T: [VT p q r phi theta psi h beta t_xp]
U  = out.U_xp;                     % T x 4: [thr de da dr]
tx = out.t_xplane_log(:);          % tempo do X-Plane
t  = out.tout(:);                  % tempo de simulacao

figure('Name', 'DH no X-Plane — estados', 'Color', 'w');

subplot(4,1,1);
plot(t, Y(1,:), 'b', 'LineWidth', 1.2); hold on;
yline(VT_ref, 'r--', 'VT_{ref}');
grid on; ylabel('V_T [m/s]');
title('DH no X-Plane — cascata PID');

subplot(4,1,2);
plot(t, Y(8,:), 'b', 'LineWidth', 1.2); hold on;
yline(h_ref, 'r--', 'h_{ref}');
grid on; ylabel('h AGL [m]');

subplot(4,1,3);
plot(t, rad2deg(Y(7,:)), 'b', 'LineWidth', 1.2); hold on;
plot(t, rad2deg(Y(5,:)), 'Color', [0 0.6 0]);
plot(t, rad2deg(Y(6,:)), 'Color', [0.85 0.4 0]);
grid on; ylabel('[deg]'); legend('\psi rel', '\phi', '\theta', 'Location', 'best');

subplot(4,1,4);
plot(t, rad2deg(Y(2,:)), t, rad2deg(Y(3,:)), t, rad2deg(Y(4,:)));
grid on; ylabel('[deg/s]'); xlabel('t [s]');
legend('p', 'q', 'r', 'Location', 'best');

figure('Name', 'DH no X-Plane — comandos', 'Color', 'w');

subplot(2,1,1);
plot(t, U(:,1), 'b', 'LineWidth', 1.2);
grid on; ylabel('\delta_T [0-1]'); ylim([-0.05 1.05]);
title('Comandos enviados ao X-Plane');

subplot(2,1,2);
plot(t, rad2deg(U(:,2)), t, rad2deg(U(:,3)), t, rad2deg(U(:,4)));
hold on; yline(25, 'k:'); yline(-25, 'k:');
grid on; ylabel('[deg]'); xlabel('t [s]');
legend('\delta_e', '\delta_a', '\delta_r', 'Location', 'best');

% Qualidade da sincronizacao: t_xplane deve avancar ~1:1 com t
dtx = tx(end) - tx(1);
fprintf('Sincronizacao: sim %.1f s | X-Plane avancou %.1f s | razao %.3f\n', ...
    t(end), dtx, dtx / t(end));
