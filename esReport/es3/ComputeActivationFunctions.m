function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here
switch mission.phase
        case 1  
            %during phase 1 safe navigation
            uvms.Aa.vehiclePos = eye(3);
            uvms.Aa.vehicleAtt = eye(3);
            uvms.Aa.ha = 1;
            uvms.Aa.vehicleAlt = 1;
            uvms.Aa.vehicleAltLanding = 0; 
            uvms.Aa.horAlignement = zeros(3,3);
                
        case 2
            % point to the rock and land
            uvms.Aa.horAlignement = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time);
            uvms.Aa.vehicleAtt = DecreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time) * eye(3);
            uvms.Aa.vehicleAlt = DecreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time);
            uvms.Aa.vehiclePos = DecreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time) * eye(3);
            uvms.Aa.vehicleAltLanding = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time); 
            uvms.Aa.ha = 1;                       
end

% arm tool position control
% always active
uvms.A.t = eye(6);

%activation function limited att
uvms.A.ha = IncreasingBellShapedFunction(0.1, 0.2, 0, 1, norm(uvms.v_rho)) * uvms.Aa.ha;

%activation function vehicle position 
uvms.A.vehiclePos = eye(3) * uvms.Aa.vehiclePos;
%activation function vehicle attitude
uvms.A.vehicleAtt = eye(3) * uvms.Aa.vehicleAtt;

% exampl activation function vehicle altitude(for sure about 1m, and i don't care
%over 1.5 m DecreasingBellShapedFunction(1, 1.5, 0, 1, uvms.sensorDistance);
threshold = 1;
range = 0.5;
uvms.A.vehicleAlt = DecreasingBellShapedFunction(threshold, (threshold + range), 0, 1, uvms.w_distance) * uvms.Aa.vehicleAlt;

%landing activation function 
uvms.A.vehicleAltLanding = 1 * uvms.Aa.vehicleAltLanding;
%
uvms.A.horAlignement = 1;


