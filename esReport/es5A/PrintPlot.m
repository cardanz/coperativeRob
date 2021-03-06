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
subplot(3,1,1);
hplot = plot(plt.t, plt.p);
set(hplot, 'LineWidth', 1);
legend('x','y','z','roll','pitch','yaw');
subplot(2,1,2);
hplot = plot(plt.t, plt.p_dot);
set(hplot, 'LineWidth', 1);
legend('xdot', 'ydot','zdot','omega_x','omega_y','omega_z');


figure(3);
hplot = plot(plt.t, plt.a(1:7,:));
set(hplot, 'LineWidth', 2);
legend('Ajl_11','Ajl_22','Ajl_33','Ajl_44','Ajl_55','Ajl_66','Ajl_77');

figure(4);
hplot = plot(plt.t, plt.a(9,:));
set(hplot, 'LineWidth', 2);
legend('Aha');
xlabel('t (s)');

jlmin  = [-2.9;-1.6;-2.9;-2.95;-2.9;-1.65;-2.8];
jlmax  = [2.9;1.65;2.9;0.01;2.9;1.25;2.8];
printMin = repmat(jlmin,[1,length(plt.t)]);
printMax = repmat(jlmax,[1,length(plt.t)]);

prefSetting = [-0.0031 1.2586 0.0128 -1.2460]';
desiredShape = repmat(prefSetting,[1,length(plt.t)]);
%1
figure(5);
subplot(4,1,1);
hplot = plot(plt.t,plt.q(1,:));
set(hplot, 'LineWidth', 2);
title('joint position 1')
xlabel('t (s)')
ylabel('angle (rad)')
hold on
hplot = plot(plt.t,printMin(1,:));
set(hplot, 'LineWidth',1, 'LineStyle','--');
hold on
hplot = plot(plt.t,printMax(1,:));
set(hplot, 'LineWidth',1, 'LineStyle','--','Color','r');
hold on
hplot = plot(plt.t,desiredShape(1,:));
set(hplot, 'LineWidth',1, 'LineStyle',':','Color','r');
legend('q1','qMin1','qMax1','desired'); 
%2
subplot(4,1,2);
hplot = plot(plt.t,plt.q(2,:));
set(hplot, 'LineWidth', 2);
title('joint position 2')
xlabel('t (s)')
ylabel('angle (rad)')
hold on
hplot = plot(plt.t,printMin(2,:));
set(hplot, 'LineWidth',1, 'LineStyle','--');
hold on
hplot = plot(plt.t,printMax(2,:));
set(hplot, 'LineWidth',1, 'LineStyle','--','Color','r');
hold on
hplot = plot(plt.t,desiredShape(2,:));
set(hplot, 'LineWidth',1, 'LineStyle',':','Color','r');
legend('q2','qMin2','qMax2','desired'); 
%3
subplot(4,1,3);
hplot = plot(plt.t,plt.q(3,:));
set(hplot, 'LineWidth', 2);
title('joint position 3')
xlabel('t (s)')
ylabel('angle (rad)')
hold on
hplot = plot(plt.t,printMin(3,:));
set(hplot, 'LineWidth',1, 'LineStyle','--');
hold on
hplot = plot(plt.t,printMax(3,:));
set(hplot, 'LineWidth',1, 'LineStyle','--','Color','r');
hold on
hplot = plot(plt.t,desiredShape(3,:));
set(hplot, 'LineWidth',1, 'LineStyle',':','Color','r');
legend('q3','qMin3','qMax3','desired'); 

%4
subplot(4,1,4);
hplot = plot(plt.t,plt.q(4,:));
set(hplot, 'LineWidth', 2);
title('joint position 4')
xlabel('t (s)')
ylabel('angle (rad)')
hold on
hplot = plot(plt.t,printMin(4,:));
set(hplot, 'LineWidth',1, 'LineStyle','--');
hold on
hplot = plot(plt.t,printMax(4,:));
set(hplot, 'LineWidth',1, 'LineStyle','--','Color','r');
hold on
hplot = plot(plt.t,desiredShape(4,:));
set(hplot, 'LineWidth',1, 'LineStyle',':','Color','r');
legend('q4','qMin4','qMax4','desired');
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

figure(12);
hplot = plot(plt.t,plt.opt(:,:));
set(hplot, 'LineWidth', 2);
legend('AprefS1','AprefS2','AprefS3','AprefS4');

% figure (13)
% plot(plt.t, plt.man, 'LineWidth', 1);
% title('man');
% xlabel('t (s)')

end

