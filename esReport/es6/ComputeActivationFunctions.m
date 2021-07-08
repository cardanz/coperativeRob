function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here
switch mission.phase
        case 1                       
            uvms.Aa.ha = 1;
            uvms.Aa.vehiclePos =  0 * eye(3);
            uvms.Aa.vehicleAtt = 0 * eye(3);
            uvms.Aa.vehicleStop = 1;
            uvms.Aa.preferedShape = 1;
            uvms.Aa.tool = 1;
            uvms.Aa.jointLimits = 1;
            uvms.Aa.vehicleControl = 1;
end

% arm tool position control
uvms.A.t = eye(6) * uvms.Aa.tool;

%activation function limited att
uvms.A.ha = IncreasingBellShapedFunction(0.1, 0.2, 0, 1, norm(uvms.v_rho)) * uvms.Aa.ha;

%activation function vehicle position 
uvms.A.vehiclePos = eye(3) * uvms.Aa.vehiclePos;
%activation function vehicle attitude
uvms.A.vehicleAtt = eye(3) * uvms.Aa.vehicleAtt;

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
                                  

%activation limit movement
uvms.A.vehicleStop = eye(6) * uvms.Aa.vehicleStop;

%shape
uvms.A.preferedShape = eye(4) * uvms.Aa.preferedShape;

%vehicle control 
uvms.A.vehicleControl = eye(6) * uvms.Aa.vehicleControl;


