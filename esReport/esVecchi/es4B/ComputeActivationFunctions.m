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
            uvms.Aa.t = 0;
            uvms.Aa.targetDistance = 0;
            uvms.Aa.vehicleStop = 0;
            uvms.Aa.jointLimits = 0;

                
        case 2
            %
            uvms.Aa.horAlignement = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time);
            uvms.Aa.vehicleAtt = DecreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time) * eye(3);
            uvms.Aa.vehicleAlt = 1;
            uvms.Aa.vehiclePos =  eye(3);
            uvms.Aa.vehicleAltLanding = 0; 
            uvms.Aa.ha = 1;
            uvms.Aa.t = 0;
            uvms.Aa.targetDistance = 0;
            uvms.Aa.vehicleStop = 0;
            uvms.Aa.jointLimits = 0;
        case 3
            % activate landing disable others, maintain ha
            uvms.Aa.targetDistance = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time);
            uvms.Aa.horAlignement = 1;
            uvms.Aa.vehicleAtt = 0 * eye(3);
            uvms.Aa.vehicleAlt = DecreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time);
            uvms.Aa.vehiclePos = DecreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time) * eye(3);
            uvms.Aa.vehicleAltLanding = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time); 
            uvms.Aa.ha = 1;
            uvms.Aa.t = 0;
            uvms.Aa.vehicleStop = 0;
            %soon activation for safety reason
            %uvms.Aa.jointLimits = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time);
    case 4  
            uvms.Aa.targetDistance =1;
            uvms.Aa.horAlignement = DecreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time);
            uvms.Aa.vehicleAtt = 0 * eye(3);
            uvms.Aa.vehicleAlt = 0;
            uvms.Aa.vehiclePos = zeros(3,3);
            uvms.Aa.vehicleAltLanding = 1; 
            uvms.Aa.ha = 1;
            uvms.Aa.t = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time);
            uvms.Aa.vehicleStop = 1;
            %IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time); 
            %no one forbids to force it to one right away 
            uvms.Aa.jointLimits = 1;
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

% exampl activation function vehicle altitude(for sure about 1m, and i don't care
%over 1.5 m DecreasingBellShapedFunction(1, 1.5, 0, 1, uvms.sensorDistance);
threshold = 1;
range = 0.5;
uvms.A.vehicleAlt = DecreasingBellShapedFunction(threshold, (threshold + range), 0, 1, uvms.w_distance) * uvms.Aa.vehicleAlt;

%landing activation function 
uvms.A.vehicleAltLanding = 1 * uvms.Aa.vehicleAltLanding;

%
uvms.A.horAlignement = IncreasingBellShapedFunction(0.05, 0.1, 0, 1, uvms.phi) ;
%
uvms.A.targetDistance = IncreasingBellShapedFunction(1, 1.5, 0, 1, norm(uvms.targetDistance)) * uvms.Aa.targetDistance; 

uvms.A.vehicleStop = eye(6) * uvms.Aa.vehicleStop;
 
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
                                                
    


