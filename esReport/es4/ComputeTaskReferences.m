function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = 0.2 * [ang; lin];
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2);

%i wish to drive the angle to zero
uvms.xdot.ha = 0.2 * (0 - norm(uvms.v_rho));

%error between goal and vehicle position and orientation projected on <w>
[w_vang, w_vlin] = CartError(uvms.wTgvehicle , uvms.wTv);
uvms.xdot.vehiclePos(1:3,:) = Saturate(0.7 * w_vlin, 0.7);
uvms.xdot.vehicleAtt(1:3,:) = Saturate(0.2 * w_vang, 0.2);

%reference for  safe altittude task 1 m
uvms.xdot.vehicleAlt = 0.7 * (1.5 - uvms.w_distance);
%reference for  safe altittude task 5 m
%uvms.xdot.vehicleAlt = 0.7 * (5.5 - uvms.w_distance);
%reference for  safe altittude task 10 m
%uvms.xdot.vehicleAlt = 0.7 * (10.5 - uvms.w_distance);

%reference for landing action 
uvms.xdot.vehicleAltLanding = 0.5 * (0 - uvms.w_distance);

%reference for horizontal alignment
uvms.xdot.vehiclehorAlignement = Saturate(0.3 * (0 - uvms.phi), 0.3);

%reference tool distance
uvms.xdot.targetDistance = Saturate(0.5 * (1.5 - norm(uvms.targetDistance)), 0.5);
%stop vehicle
uvms.xdot.vehicleStop = zeros(6,1);

%print var
uvms.wAng = w_vang;
uvms.wLin = w_vlin;