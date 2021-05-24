function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = 0.2 * [ang; lin];
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2);

%error between goal and vehicle position and orientation projected on <w>
[w_vang, w_vlin] = CartError(uvms.wTgvehicle , uvms.wTv);
uvms.xdot.vehiclePos(1:3,:) = Saturate(0.2 * w_vlin, 0.2);
uvms.xdot.vehicleAtt(1:3,:) = Saturate(0.2 * w_vang, 0.2);

%reference for altittude task
uvms.xdot.vehicleAlt = 0.2 * (1.5 - uvms.w_distance);

%print var
uvms.wAng = w_vang;
uvms.wLin = w_vlin;