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
            uvms.Aa.horAlignement = 0;
            uvms.Aa.t = 0;
            uvms.Aa.targetDistance = 0;
            uvms.Aa.vehicleStop = 0;
            uvms.Aa.jointLimits = 0;
                
        case 2
            % point to the rock 
            uvms.Aa.vehiclePos = eye(3);
            uvms.Aa.vehicleAtt = DecreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time) * eye(3); 
            uvms.Aa.ha = 1; 
            uvms.Aa.vehicleAlt = 1;
            uvms.Aa.vehicleAltLanding = 0; 
            uvms.Aa.horAlignement = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time);
            uvms.Aa.t = 0;
            uvms.Aa.targetDistance = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time);
            uvms.Aa.vehicleStop = 0;
            uvms.Aa.jointLimits = 0;
        case 3
            %land pointing
            uvms.Aa.vehiclePos = DecreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time) * eye(3);
            uvms.Aa.vehicleAtt = zeros(3,3);
            uvms.Aa.ha = 1; 
            uvms.Aa.vehicleAltLanding = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time);
            uvms.Aa.vehicleAlt = DecreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time);
            uvms.Aa.horAlignement = 1;
            uvms.Aa.t = 0;
            uvms.Aa.targetDistance = DecreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time);
            uvms.Aa.vehicleStop = 0;
            uvms.Aa.jointLimits = 0;
        case 4
            uvms.Aa.vehiclePos = 0 * eye(3);
            uvms.Aa.vehicleAtt = zeros(3,3);
            uvms.Aa.ha = 1; 
            uvms.Aa.vehicleAltLanding = 1;
            uvms.Aa.vehicleAlt = 0;
            uvms.Aa.horAlignement = 1;
            uvms.Aa.t = 1;
            uvms.Aa.targetDistance = 0;
            uvms.Aa.vehicleStop = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time); 
            uvms.Aa.jointLimits = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time);
            
            
end

% arm tool position control
% always active
uvms.A.t = eye(6) * uvms.Aa.t;

%activation function limited att
uvms.A.ha = IncreasingBellShapedFunction(0.05, 0.1, 0, 1, norm(uvms.v_rho)) * uvms.Aa.ha;

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

%horz all. activation function
uvms.A.horAlignement = 1 * uvms.Aa.horAlignement;

%target distance activation function
uvms.A.targetDistance = IncreasingBellShapedFunction(1.5, 2, 0, 1, norm(uvms.targetDistance)) * uvms.Aa.targetDistance; 

%stop vehicle movement activation function
uvms.A.vehicleStop = eye(6) * uvms.Aa.vehicleStop;

%joint limits activation function
uvms.A.jointLimitsL = diag([DecreasingBellShapedFunction(uvms.jlmin(1), uvms.jlmin(1) + uvms.rangeJoint, 0, 1, uvms.q(1)),...
                            DecreasingBellShapedFunction(uvms.jlmin(2), uvms.jlmin(2) + uvms.rangeJoint, 0, 1, uvms.q(2)),...
                            DecreasingBellShapedFunction(uvms.jlmin(3), uvms.jlmin(3) + uvms.rangeJoint, 0, 1, uvms.q(3)),...
                            DecreasingBellShapedFunction(uvms.jlmin(4), uvms.jlmin(4) + uvms.rangeJoint, 0, 1, uvms.q(4)),...
                            DecreasingBellShapedFunction(uvms.jlmin(5), uvms.jlmin(5) + uvms.rangeJoint, 0, 1, uvms.q(5)),...
                            DecreasingBellShapedFunction(uvms.jlmin(6), uvms.jlmin(6) + uvms.rangeJoint, 0, 1, uvms.q(6)),...
                            DecreasingBellShapedFunction(uvms.jlmin(7), uvms.jlmin(7) + uvms.rangeJoint, 0, 1, uvms.q(7))]) *  uvms.Aa.jointLimits;
                        
uvms.A.jointLimitsU = diag([IncreasingBellShapedFunction(uvms.jlmax(1) - uvms.rangeJoint, uvms.jlmax(1) , 0, 1, uvms.q(1)),...
                            IncreasingBellShapedFunction(uvms.jlmax(2) - uvms.rangeJoint, uvms.jlmax(2) , 0, 1, uvms.q(2)),...
                            IncreasingBellShapedFunction(uvms.jlmax(3) - uvms.rangeJoint, uvms.jlmax(3) , 0, 1, uvms.q(3)),...
                            IncreasingBellShapedFunction(uvms.jlmax(4) - uvms.rangeJoint, uvms.jlmax(4) , 0, 1, uvms.q(4)),...
                            IncreasingBellShapedFunction(uvms.jlmax(5) - uvms.rangeJoint, uvms.jlmax(5) , 0, 1, uvms.q(5)),...
                            IncreasingBellShapedFunction(uvms.jlmax(6) - uvms.rangeJoint, uvms.jlmax(6) , 0, 1, uvms.q(6)),...
                            IncreasingBellShapedFunction(uvms.jlmax(7) - uvms.rangeJoint, uvms.jlmax(7) , 0, 1, uvms.q(7))]) *  uvms.Aa.jointLimits;
                                                


