function [uvms] = InitUVMS(robotname)

% uvms.vTb
% transformation matrix betwene the arm base wrt vehicle frame
% expresses how the base of the arm is attached to the vehicle
% do NOT change, since it must be coherent with the visualization tool
if (strcmp(robotname, 'DexROV'))    
    % do NOT change
    uvms.vTb = [rotation(pi, 0, pi) [0.167 0 -0.43]'; 0 0 0 1]; 
else
    if (strcmp(robotname, 'Robust'))
        % do NOT change
        uvms.vTb = [rotation(0, 0, pi) [0.85 0 -0.42]'; 0 0 0 1];
    end
end

uvms.q_dot = [0 0 0 0 0 0 0]';
uvms.p_dot = [0 0 0 0 0 0]';

% joint limits corresponding to the actual MARIS arm configuration
uvms.jlmin  = [-2.9;-1.6;-2.9;-2.95;-2.9;-1.65;-2.8];
uvms.jlmax  = [2.9;1.65;2.9;0.01;2.9;1.25;2.8];

% to be computed at each time step
uvms.wTv = eye(4,4);
uvms.wTt = eye(4,4);
uvms.vTw = eye(4,4);
uvms.vTe = eye(4,4);
uvms.vTt = eye(4,4);
uvms.vTg = eye(4,4);
uvms.Ste = eye(6,6);
uvms.bTe = eye(4,4);
uvms.bJe = eye(6,7);
uvms.djdq = zeros(6,7,7);
uvms.mu  = 0;
uvms.phi = zeros(3,1);
uvms.sensorDistance = 0;

uvms.Jjl = [];
uvms.Jmu = [];
uvms.Jha = [];
uvms.Jt_a = [];
uvms.Jt_v = [];
uvms.Jt = [];
%jacobian added
uvms.Jha = [];
uvms.JvehiclePos = [];
uvms.JvehicleAtt = [];
uvms.JvehicleAltitude = [];
uvms.JvehicleAllignement = [];
uvms.JtargetDistance = [];
uvms.JvehicleStop = [];
uvms.JjointLimits = [];

uvms.xdot.jl = [];
uvms.xdot.mu = [];
uvms.xdot.ha = [];
uvms.xdot.t = [];
%xdot added
uvms.xdot.ha = [];
uvms.xdot.vehiclePos = [];
uvms.xdot.vehicleAtt = [];
uvms.xdot.vehicleAlt = [];
uvms.xdot.vehicleAltLanding = [];
uvms.xdot.vehiclehorAlignement = [];
uvms.xdot.targetDistance =[];
uvms.xdot.vehicleStop = [];
uvms.xdot.jointLimitsL = [];
uvms.xdot.jointLimitsU = [];

uvms.A.jl = zeros(7,7);
uvms.A.mu = 0;
uvms.A.ha = zeros(1,1);
uvms.A.t = zeros(6,6);
%activation added
uvms.A.vehiclePos = zeros(3,3);
uvms.A.vehicleAtt = zeros(3,3);
uvms.A.vehicleAlt = zeros(1,1);
uvms.A.ha = 0;
uvms.A.vehicleAltLanding = 0;
uvms.A.horAlignement = 0;
uvms.A.targetDistance = 0;
uvms.A.vehicleStop = zeros(6,6);
uvms.A.jointLimitsL = zeros(7,7);
uvms.A.jointLimitsU = zeros(7,7);

%pay attention if initial value  (1) can be dangerous?!?
uvms.Aa.vehiclePos = eye(3);
uvms.Aa.vehicleAtt = eye(3);
uvms.Aa.ha = 1;
uvms.Aa.vehicleAlt = 0;
uvms.Aa.vehicleAltLanding = 0;
uvms.Aa.horAlignement = 0;
uvms.Aa.t = 0;
uvms.Aa.targetDistance = 0;
uvms.Aa.vehicleStop = 0;
uvms.Aa.jointLimits = 0;

%others
uvms.v_rho = zeros(3,1);
uvms.phi = 0;
uvms.targetDistance = zeros(2,1);
uvms.offsetJoint = 0.3;

%printValue
uvms.wAng = 0;
uvms.wLin = 0;



end

