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

figure(4);
hplot = plot(plt.t, plt.a(8:9,:));
set(hplot, 'LineWidth', 2);
legend('Amu', 'Aha');
    
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

figure(8);
hplot = plot(plt.t, plt.a(2:8,:));
set(hplot, 'LineWidth', 1);  
title('act. function joint lower limit');
legend('jl1','jl2','jl3','jl4','jl5','jl6','jl7');
xlabel('t (s)')
figure(15);
hplot = plot(plt.t, plt.a(9:15,:));
set(hplot, 'LineWidth', 1);  
title('act. function joint lower limit')
legend('ju1','ju2','ju3','ju4','ju5','ju6','ju7');
xlabel('t (s)')

figure(9);
hplot = plot(plt.t, plt.altitude);
set(hplot, 'LineWidth', 1);
legend('vehicle altitude');  
title('altitude')
xlabel('t (s)')
   
figure(10);
hplot = plot(plt.t, plt.toolPos(:,:));
set(hplot, 'LineWidth', 1);
legend('x','y','z');
title('tool position')
xlabel('t (s)')
ylabel('Position (m)')
figure(20);
hplot = plot(plt.t, plt.toolPos(1,:));
set(hplot, 'LineWidth', 1);
title('comparison vehicle and tool x')
xlabel('t (s)')
ylabel('Position (m)')
hold on 
hplot = plot(plt.t, plt.p(3,:));
set(hplot, 'LineWidth', 1);
legend('x tool','x vehicle');
figure(21);
hplot = plot(plt.t, plt.toolPos(2,:));
set(hplot, 'LineWidth', 1);
title('comparison vehicle and tool y')
xlabel('t (s)')
ylabel('Position (m)')
hold on 
hplot = plot(plt.t, plt.p(2,:));
set(hplot, 'LineWidth', 1);
legend('y tool','y vehicle');
figure(22);
hplot = plot(plt.t, plt.toolPos(3,:));
set(hplot, 'LineWidth', 1);
title('comparison vehicle and tool z')
xlabel('t (s)')
ylabel('Position (m)')
hold on 
hplot = plot(plt.t, plt.p(3,:));
set(hplot, 'LineWidth', 1);
legend('z tool','z vehicle');
figure(23)
plot(plt.t, plt.xdot_t, 'LineWidth', 1);
title('tool velocities');
legend('w_x','w_y','w_z','v_x', 'v_y','v_z');
xlabel('t (s)')
ylabel('Velocities')

jlmin  = [-2.9;-1.6;-2.9;-2.95;-2.9;-1.65;-2.8];
jlmax  = [2.9;1.65;2.9;0.01;2.9;1.25;2.8];
printMin = repmat(jlmin,[1,length(plt.t)]);
printMax = repmat(jlmax,[1,length(plt.t)]);

%1
figure(12);
subplot(4,1,1);
hplot = plot(plt.t,plt.q(1,:));
set(hplot, 'LineWidth', 1);
title('joint position 1')
xlabel('t (s)')
ylabel('angle (rad)')
hold on
hplot = plot(plt.t,printMin(1,:));
set(hplot, 'LineWidth',1, 'LineStyle','--');
hold on
hplot = plot(plt.t,printMax(1,:));
set(hplot, 'LineWidth',1, 'LineStyle','--','Color','r');
legend('q1','qMin1','qMax1'); 
%2
subplot(4,1,2);
hplot = plot(plt.t,plt.q(2,:));
set(hplot, 'LineWidth', 1);
title('joint position 2')
xlabel('t (s)')
ylabel('angle (rad)')
hold on
hplot = plot(plt.t,printMin(2,:));
set(hplot, 'LineWidth',1, 'LineStyle','--');
hold on
hplot = plot(plt.t,printMax(2,:));
set(hplot, 'LineWidth',1, 'LineStyle','--','Color','r');
legend('q2','qMin2','qMax2'); 
%3
subplot(4,1,3);
hplot = plot(plt.t,plt.q(3,:));
set(hplot, 'LineWidth', 1);
title('joint position 3')
xlabel('t (s)')
ylabel('angle (rad)')
hold on
hplot = plot(plt.t,printMin(3,:));
set(hplot, 'LineWidth',1, 'LineStyle','--');
hold on
hplot = plot(plt.t,printMax(3,:));
set(hplot, 'LineWidth',1, 'LineStyle','--','Color','r');
legend('q3','qMin3','qMax3'); 
%4
subplot(4,1,4);
hplot = plot(plt.t,plt.q(4,:));
set(hplot, 'LineWidth', 1);
title('joint position 4')
xlabel('t (s)')
ylabel('angle (rad)')
hold on
hplot = plot(plt.t,printMin(4,:));
set(hplot, 'LineWidth',1, 'LineStyle','--');
hold on
hplot = plot(plt.t,printMax(4,:));
set(hplot, 'LineWidth',1, 'LineStyle','--','Color','r');
legend('q4','qMin4','qMax4'); 
%5
figure(11);
subplot(3,1,1);
hplot = plot(plt.t,plt.q(5,:));
set(hplot, 'LineWidth', 1);
title('joint position 5')
xlabel('t (s)')
ylabel('angle (rad)')
hold on
hplot = plot(plt.t,printMin(5,:));
set(hplot, 'LineWidth',1, 'LineStyle','--');
hold on
hplot = plot(plt.t,printMax(5,:));
set(hplot, 'LineWidth',1, 'LineStyle','--','Color','r');
legend('q5','qMin5','qMax5'); 
%6
subplot(3,1,2);
hplot = plot(plt.t,plt.q(6,:));
set(hplot, 'LineWidth', 1);
title('joint position 6')
xlabel('t (s)')
ylabel('angle (rad)')
hold on
hplot = plot(plt.t,printMin(6,:));
set(hplot, 'LineWidth',1, 'LineStyle','--');
hold on
hplot = plot(plt.t,printMax(6,:));
set(hplot, 'LineWidth',1, 'LineStyle','--','Color','r');
legend('q6','qMin6','qMax6'); 
%7
subplot(3,1,3);
hplot = plot(plt.t,plt.q(7,:));
set(hplot, 'LineWidth', 1);
title('joint position 7')
xlabel('t (s)')
ylabel('angle (rad)')
hold on
hplot = plot(plt.t,printMin(7,:));
set(hplot, 'LineWidth',1, 'LineStyle','--');
hold on
hplot = plot(plt.t,printMax(7,:));
set(hplot, 'LineWidth',1, 'LineStyle','--','Color','r');
legend('q7','qMin7','qMax7'); 

    

    

end

