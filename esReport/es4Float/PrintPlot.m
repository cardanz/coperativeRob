function [ ] = PrintPlot( plt )

% some predefined plots
% you can add your own

figure(1);
subplot(2,1,1);
hplot = plot(plt.t, plt.q);
set(hplot, 'LineWidth', 1);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot);
set(hplot, 'LineWidth', 1);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');


figure(2);
% subplot(3,1,1);
% hplot = plot(plt.t, plt.p);
% set(hplot, 'LineWidth', 1);
% legend('x','y','z','roll','pitch','yaw');
% subplot(2,1,2);
hplot = plot(plt.t, plt.p_dot);
set(hplot, 'LineWidth', 1);
title('vehicle velocity')
xlabel('t (s)')
ylabel('L.vel (m/s),A.vel (rad/s) ')
legend('xdot', 'ydot','zdot','omega_x','omega_y','omega_z');


figure(3);
hplot = plot(plt.t, plt.a(1:7,:));
set(hplot, 'LineWidth', 2);
legend('Ajl_11','Ajl_22','Ajl_33','Ajl_44','Ajl_55','Ajl_66','Ajl_77');

figure(4);
hplot = plot(plt.t, plt.a(9:12,:));
set(hplot, 'LineWidth', 2);
legend('Aha', 'AaltLand','AtargetDistU','AtargetDistL');
    
    
figure(5);
subplot(3,1,1);
hplot = plot(plt.t, plt.p((4: end),:));
set(hplot, 'LineWidth', 1);
legend('roll','pitch','yaw');
subplot(2,1,2);
hplot = plot(plt.t, plt.p_dot((4: end),:));
set(hplot, 'LineWidth', 1);
legend('omega_x','omega_y','omega_z');

figure(6);
hplot = plot(plt.t, plt.altitude(:,:));
set(hplot, 'LineWidth', 1);
legend('altitude');

figure(7);
hplot = plot(plt.t, plt.p((1: 3),:));
set(hplot, 'LineWidth', 1);
legend('x','y','z');
title('vehicle position')
xlabel('t (s)')
ylabel('Position (m)')
ylim([-50 50])

figure(8);
hplot = plot(plt.t, plt.a(10:11,:));
set(hplot, 'LineWidth', 2);  
legend('safety altitude','landing');
title('Afunction vehicle altitude')
xlabel('t (s)')

figure(9);
hplot = plot(plt.t, plt.altitude);
set(hplot, 'LineWidth', 1);
legend('vehicle altitude');  
title('altitude')
xlabel('t (s)')
    

end

