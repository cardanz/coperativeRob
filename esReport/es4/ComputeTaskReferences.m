function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = 0.2 * [ang; lin];
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2);

%i wish to drive the angle to zero
uvms.xdot.ha = 0.1 * (0 - norm(uvms.v_rho));

%error between goal and vehicle position and orientation projected on <w>
[w_vang, w_vlin] = CartError(uvms.wTgvehicle , uvms.wTv);
uvms.xdot.vehiclePos(1:3,:) = Saturate(0.7 * w_vlin, 0.7);
uvms.xdot.vehicleAtt(1:3,:) = Saturate(0.2 * w_vang, 0.2);

%value t = 1m, 5m, 10m;range 0.5m
threshold = 1;
range = 0.5;
%reference for altittude task 1 m
uvms.xdot.vehicleAlt = Saturate(0.7 * ((threshold + range) - uvms.w_distance),0.7);

%reference for landing action 
uvms.xdot.vehicleAltLanding = 0.5 * (0 - uvms.w_distance);

%reference for horizontal alignment
uvms.xdot.vehiclehorAlignement = Saturate(0.5 * (0 - uvms.theta), 0.5);

%reference tool distance
uvms.xdot.targetDistance = Saturate(0.5 * (1.5 - norm(uvms.targetDistance)), 0.5);

%reference stop vehicle
uvms.xdot.vehicleStop = zeros(6,1);


%print var
uvms.wAng = w_vang;
uvms.wLin = w_vlin;