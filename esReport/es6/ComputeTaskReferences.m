function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for tool-frame position control task
gain = 1;
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = gain * [ang; lin];
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), gain);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), gain);

%i wish to drive the angle to zero
uvms.xdot.ha = gain * (0 - norm(uvms.v_rho));

%error between goal and vehicle position and orientation projected on <w>
 [w_vang, w_vlin] = CartError(uvms.wTgvehicle , uvms.wTv);
 uvms.xdot.vehiclePos(1:3,:) = Saturate(0.7 * w_vlin, 0.7);
 uvms.xdot.vehicleAtt(1:3,:) = Saturate(0.5 * w_vang, 0.5);

%stop vehicle
uvms.xdot.vehicleStop = zeros(6,1);

%joint limits lower
uvms.xdot.jointLimitsL = Saturate(gain * ((uvms.jlmin + uvms.rangeJoint) -uvms.q), gain);
%joint limits upper
uvms.xdot.jointLimitsU = Saturate(gain * ((uvms.jlmax - uvms.rangeJoint) -uvms.q), gain);

%prefered shapes
prefSetting = [-0.0031 1.2586 0.0128 -1.2460]';
uvms.xdot.preferedShape = Saturate(gain * (prefSetting- uvms.q(1:4)), gain);
%vehicle control 
uvms.xdot.vehicleControl = uvms.p_dot;

