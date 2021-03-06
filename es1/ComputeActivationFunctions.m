function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

% arm tool position control
% always active
uvms.A.t = eye(6);

%activation function (equality) 
uvms.A.vpos = eye(3);
uvms.A.vatt = eye(3);

%activation functione angle
uvms.A.ha = IncreasingBellShapedFunction(0.1, 0.2, 0, 1, norm(uvms.v_rho));

%activation function altitude
uvms.A.valt = DencreasingBellShapedFunction(0.1, 2, 0, 1,uvms.w_dist);