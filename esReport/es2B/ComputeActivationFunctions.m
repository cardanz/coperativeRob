function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

% arm tool position control
% always active
uvms.A.t = eye(6);

%activation function limited att
uvms.A.ha = IncreasingBellShapedFunction(0.1, 0.2, 0, 1, norm(uvms.v_rho));

%activation function vehicle position 
uvms.A.vehiclePos = eye(3);
%activation function vehicle attitude
uvms.A.vehicleAtt = eye(3);

% exampl activation function vehicle altitude(for sure about 1m, and i don't care
%over 1.5 m DecreasingBellShapedFunction(1, 1.5, 0, 1, uvms.sensorDistance);
% 1 m
uvms.A.vehicleAlt = DecreasingBellShapedFunction(1, 1.5, 0, 1, uvms.w_distance);
% 5 m
%uvms.A.vehicleAlt = DecreasingBellShapedFunction(5, 5.5, 0, 1, uvms.w_distance);
% 10 m
%uvms.A.vehicleAlt = DecreasingBellShapedFunction(10, 10.5, 0, 1, uvms.w_distance);

%landing activation function 
uvms.A.vehicleAltLanding = 1;


