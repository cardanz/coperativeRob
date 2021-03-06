function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

% arm tool position control
% always active
uvms.A.t = eye(6);
%activation function limited att
uvms.A.ha = IncreasingBellShapedFunction(0.1, 0.2, 0, 1, norm(uvms.v_rho));

%shape
uvms.A.preferedShape = eye(4);

%joint limits activation function
uvms.A.jointLimitsL = diag([DecreasingBellShapedFunction(uvms.jlmin(1), uvms.jlmin(1) + uvms.rangeJoint, 0, 1, uvms.q(1)),...
                            DecreasingBellShapedFunction(uvms.jlmin(2), uvms.jlmin(2) + uvms.rangeJoint, 0, 1, uvms.q(2)),...
                            DecreasingBellShapedFunction(uvms.jlmin(3), uvms.jlmin(3) + uvms.rangeJoint, 0, 1, uvms.q(3)),...
                            DecreasingBellShapedFunction(uvms.jlmin(4), uvms.jlmin(4) + uvms.rangeJoint, 0, 1, uvms.q(4)),...
                            DecreasingBellShapedFunction(uvms.jlmin(5), uvms.jlmin(5) + uvms.rangeJoint, 0, 1, uvms.q(5)),...
                            DecreasingBellShapedFunction(uvms.jlmin(6), uvms.jlmin(6) + uvms.rangeJoint, 0, 1, uvms.q(6)),...
                            DecreasingBellShapedFunction(uvms.jlmin(7), uvms.jlmin(7) + uvms.rangeJoint, 0, 1, uvms.q(7))]);
                        
uvms.A.jointLimitsU = diag([IncreasingBellShapedFunction(uvms.jlmax(1) - uvms.rangeJoint, uvms.jlmax(1) , 0, 1, uvms.q(1)),...
                            IncreasingBellShapedFunction(uvms.jlmax(2) - uvms.rangeJoint, uvms.jlmax(2) , 0, 1, uvms.q(2)),...
                            IncreasingBellShapedFunction(uvms.jlmax(3) - uvms.rangeJoint, uvms.jlmax(3) , 0, 1, uvms.q(3)),...
                            IncreasingBellShapedFunction(uvms.jlmax(4) - uvms.rangeJoint, uvms.jlmax(4) , 0, 1, uvms.q(4)),...
                            IncreasingBellShapedFunction(uvms.jlmax(5) - uvms.rangeJoint, uvms.jlmax(5) , 0, 1, uvms.q(5)),...
                            IncreasingBellShapedFunction(uvms.jlmax(6) - uvms.rangeJoint, uvms.jlmax(6) , 0, 1, uvms.q(6)),...
                            IncreasingBellShapedFunction(uvms.jlmax(7) - uvms.rangeJoint, uvms.jlmax(7) , 0, 1, uvms.q(7))]);


