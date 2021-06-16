function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here
switch mission.phase
        case 1  
            %during phase 1 safe navigation
            %uvms.Aa.vehiclePos = eye(3);
            %uvms.Aa.vehicleAtt = eye(3);
            uvms.Aa.ha = 1;
            %add vehicle min altitude
            %uvms.Aa.vehicleStop = 0;
            uvms.Aa.preferedShape = 1;
            uvms.Aa.tool = 1;
            uvms.Aa.vehicleControl = 1;
            
        case 2
            uvms.Aa.ha = 1;
            uvms.Aa.vehiclePos =  DecreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time) * eye(3);
            uvms.Aa.vehicleAtt = DecreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time) * eye(3);
            uvms.Aa.vehicleStop = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time);
            uvms.Aa.preferedShape = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time);
            uvms.Aa.tool = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time);
end

% arm tool position control
uvms.A.t = eye(6) * uvms.Aa.tool;

%activation function limited att
uvms.A.ha = IncreasingBellShapedFunction(0.1, 0.2, 0, 1, norm(uvms.v_rho)) * uvms.Aa.ha;

%activation function vehicle position 
uvms.A.vehiclePos = eye(3) * uvms.Aa.vehiclePos;
%activation function vehicle attitude
uvms.A.vehicleAtt = eye(3) * uvms.Aa.vehicleAtt;

%activation limit movement
uvms.A.vehicleStop = eye(6) * uvms.Aa.vehicleStop;

%shape
uvms.A.preferedShape = eye(4) * uvms.Aa.preferedShape;

%vehicle control 
uvms.A.vehicleControl = eye(6) * uvms.Aa.vehicleControl;


