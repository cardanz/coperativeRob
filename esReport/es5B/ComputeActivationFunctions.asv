function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here
switch mission.phase
        case 1  
            %during phase 1 safe navigation
            uvms.Aa.vehiclePos = eye(3);
            uvms.Aa.vehicleAtt = eye(3);
            uvms.Aa.ha = 1;
            %add vehicle min altitude
            
        case 2
            %
            uvms.Aa.horAlignement = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time);
            uvms.Aa.vehicleAtt = DecreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time) * eye(3);
            uvms.Aa.vehiclePos =  eye(3);
            uvms.Aa.vehicleAtt = eye(3);
end

% arm tool position control
% always active
uvms.A.t = eye(6) * uvms.Aa.t;

%activation function limited att
uvms.A.ha = IncreasingBellShapedFunction(0.1, 0.2, 0, 1, norm(uvms.v_rho)) * uvms.Aa.ha;

%activation function vehicle position 
uvms.A.vehiclePos = eye(3) * uvms.Aa.vehiclePos;
%activation function vehicle attitude
uvms.A.vehicleAtt = eye(3) * uvms.Aa.vehicleAtt;



