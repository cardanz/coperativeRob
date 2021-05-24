function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

% arm tool position control
% always active
uvms.A.t = eye(6);

%activation function vehicle position 
uvms.A.vehiclePos = eye(3);
%activation function vehicle attitude
uvms.A.vehicleAtt = eye(3);
%activation function vehicle altitude(for sure about 1m, and i don't care
%over 1.5 m
uvms.A.vehicleAlt = DecreasingBellShapedFunction(1, 1.5, 0, 1, uvms.sensorDistance);