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