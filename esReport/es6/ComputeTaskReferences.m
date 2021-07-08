function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = 0.5 * [ang; lin];
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.5);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.5);

%i wish to drive the angle to zero
uvms.xdot.ha = 0.2 * (0 - norm(uvms.v_rho));

%error between goal and vehicle position and orientation projected on <w>
 [w_vang, w_vlin] = CartError(uvms.wTgvehicle , uvms.wTv);
 uvms.xdot.vehiclePos(1:3,:) = Saturate(0.7 * w_vlin, 0.7);
 uvms.xdot.vehicleAtt(1:3,:) = Saturate(0.5 * w_vang, 0.5);

%stop vehicle
uvms.xdot.vehicleStop = zeros(6,1);

%joint limits lower
uvms.xdot.jointLimitsL = Saturate(0.5 * ((uvms.jlmin + uvms.rangeJoint) -uvms.q), 0.5);
%joint limits upper
uvms.xdot.jointLimitsU = Saturate(0.5 * ((uvms.jlmax - uvms.rangeJoint) -uvms.q), 0.5);

%prefered shapes
prefSetting = [-0.0031 1.2586 0.0128 -1.2460]';
uvms.xdot.preferedShape = Saturate(0.5 * (prefSetting- uvms.q(1:4)), 0.5);
%vehicle control 
uvms.xdot.vehicleControl = uvms.p_dot;

