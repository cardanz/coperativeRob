function MainDexrov
addpath('./simulation_scripts');
clc;
clear;
close all

% Simulation variables (integration and final time)
deltat = 0.005;
end_time = 30;
loop = 1;
maxloops = ceil(end_time/deltat);

% this struct can be used to evolve what the UVMS has to do
mission.phase = 1;
mission.phase_time = 0;

% Rotation matrix to convert coordinates between Unity and the <w> frame
% do not change
wuRw = rotation(0,-pi/2,pi/2);
vRvu = rotation(-pi/2,0,-pi/2);

% pipe parameters
u_pipe_center = [-10.66 31.47 -1.94]'; % in unity coordinates
pipe_center = wuRw'*u_pipe_center;     % in world frame coordinates
pipe_radius = 0.3;

% UDP Connection with Unity viewer v2
% UDP Connection with Unity viewer v2
uArm = udp('127.0.0.1',15000,'OutputDatagramPacketSize',28);
uVehicle = udp('127.0.0.1',15001,'OutputDatagramPacketSize',24);
fopen(uVehicle);
fopen(uArm);

% Preallocation
plt = InitDataPlot(maxloops);

% initialize uvms structure
uvms = InitUVMS('DexROV');
% uvms.q 
% Initial joint positions. You can change these values to initialize the simulation with a 
% different starting position for the arm
uvms.q = [-0.0031 1.2586 0.0128 -1.2460 0.0137 0.0853-pi/2 0.0137]';
% uvms.p
% initial position of the vehicle
% the vector contains the values in the following order
% [x y z r(rot_x) p(rot_y) y(rot_z)]
% RPY angles are applied in the following sequence
% R(rot_x, rot_y, rot_z) = Rz (rot_z) * Ry(rot_y) * Rx(rot_x)
%uvms.p = [-1.9379 10.4813-6.1 -29.7242-0.1 0 0 0]';
uvms.p = [ -1.8346, 10.540, -29.34, 0, 0,  0]';
% initial goal position definition
% slightly over the top of the pipe
distanceGoalWrtPipe = 0.3;
uvms.goalPosition = pipe_center + (pipe_radius + distanceGoalWrtPipe)*[0 0 1]';
uvms.wRg = rotation(pi,0,0);
uvms.wTg = [uvms.wRg uvms.goalPosition; 0 0 0 1];

%vehicle goal position
%define the goal position for the vehicle
distanceVehicleWrtPipe = 2;
uvms.vehicleGoalPosition = pipe_center + (pipe_radius + distanceVehicleWrtPipe)*[0 0 1]';
uvms.wRgvehicle = rotation(0, 0, 0);
%goal frame w.r.t world frameas
uvms.wTgvehicle = [uvms.wRgvehicle uvms.vehicleGoalPosition; 0 0 0 1];

% defines the tool control point
uvms.eTt = eye(4);
tic
for t = 0:deltat:end_time
    % update all the involved variables
    uvms = UpdateTransforms(uvms);
    uvms = ComputeJacobians(uvms);
    uvms = ComputeTaskReferences(uvms, mission);
    uvms = ComputeActivationFunctions(uvms, mission);
       
    % main kinematic algorithm initialization
    % rhop order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    rhop = zeros(13,1);
    Qp = eye(13); 
    %safety tasks
    [Qp, rhop] = iCAT_task(uvms.A.ha,   uvms.Jha,   Qp, rhop, uvms.xdot.ha, 0.0001,   0.01, 10); 
    
    [Qp, rhop] = iCAT_task(uvms.A.jointLimitsL,    uvms.JjointLimits,    Qp, rhop, uvms.xdot.jointLimitsL,  0.0001,   0.01, 10);
    [Qp, rhop] = iCAT_task(uvms.A.jointLimitsU,    uvms.JjointLimits,    Qp, rhop, uvms.xdot.jointLimitsU,  0.0001,   0.01, 10);  
    
    %toll control
    [Qp, rhop] = iCAT_task(uvms.A.t,    uvms.Jt,    Qp, rhop, uvms.xdot.t,  0.0001,   0.01, 10);
    %optmization
    [Qp, rhop] = iCAT_task(uvms.A.preferedShape,    uvms.JpreferedShape,    Qp, rhop, uvms.xdot.preferedShape,  0.0001,   0.01, 10);
    %[Qp, rhop] = iCAT_task(uvms.A.vehicleStop,    uvms.JvehicleStop,    Qp, rhop, uvms.xdot.vehicleStop,  0.0001,   0.01, 10);

    [Qp, rhop] = iCAT_task(eye(13),     eye(13),    Qp, rhop, zeros(13,1),  0.0001,   0.01, 10);    % this task should be the last one
    % rhop order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    rhop2 = zeros(13,1);
    Qp2 = eye(13); 
    %vehicle control
    [Qp2, rhop2] = iCAT_task(uvms.A.vehicleControl,   uvms.JvehicleControl,   Qp2, rhop2, uvms.xdot.vehicleControl, 0.0001,   0.01, 10);   
    [Qp2, rhop2] = iCAT_task(uvms.A.ha,   uvms.Jha,   Qp2, rhop2, uvms.xdot.ha, 0.0001,   0.01, 10);    
    
    [Qp2, rhop2] = iCAT_task(uvms.A.jointLimitsL,    uvms.JjointLimits,    Qp2, rhop2, uvms.xdot.jointLimitsL,  0.0001,   0.01, 10);
    [Qp2, rhop2] = iCAT_task(uvms.A.jointLimitsU,    uvms.JjointLimits,    Qp2, rhop2, uvms.xdot.jointLimitsU,  0.0001,   0.01, 10);  
    %toll control
    [Qp2, rhop2] = iCAT_task(uvms.A.t,    uvms.Jt,    Qp2, rhop2, uvms.xdot.t,  0.0001,   0.01, 10);
    %optmization
    [Qp2, rhop2] = iCAT_task(uvms.A.preferedShape,    uvms.JpreferedShape,    Qp2, rhop2, uvms.xdot.preferedShape,  0.0001,   0.01, 10);
    %[Qp2, rhop2] = iCAT_task(uvms.A.vehicleStop,    uvms.JvehicleStop,    Qp2, rhop2, uvms.xdot.vehicleStop,  0.0001,   0.01, 10);

    [Qp2, rhop2] = iCAT_task(eye(13),     eye(13),    Qp2, rhop2, zeros(13,1),  0.0001,   0.01, 10);    % this task should be the last one
    
    % get the two variables for integrations
    % vehicle velocity from the first, tool control from the second
    q_dot = rhop2(1:7);
    p_dot = rhop(8:13);
    
    % add noise
    % sinusoidal velocity disturbance * amplitude wrt world frame 
    dist = sin(0.5 * pi * t)*[1.5 1.5 0 0 0.0 0.0]';
    % sinusoidal velocity disturbance wrt vehicle frame
    noisePdot = [uvms.vTw(1:3,1:3)  zeros(3,3);zeros(3,3) uvms.vTw(1:3,1:3)]*dist;
    uvms.q_dot = q_dot;        
    uvms.p_dot = p_dot + noisePdot;   
    % Integration
	uvms.q = uvms.q + uvms.q_dot*deltat;
    % beware: p_dot should be projected on <v>
    uvms.p = integrate_vehicle(uvms.p, uvms.p_dot, deltat);
    
  % check if the mission phase should be changed
    mission.phase_time = mission.phase_time + deltat;
    [uvms, mission] = UpdateMissionPhase(uvms, mission);
    
    % send packets to Unity viewer
    SendUdpPackets(uvms,wuRw,vRvu,uArm,uVehicle);
        
    % collect data for plots
    plt = UpdateDataPlot(plt,uvms,t,loop);
    loop = loop + 1;
   
    % add debug prints hereww
    if (mod(t,0.1) == 0)
        mission.phase
        [ang, lin] = CartError(uvms.wTg , uvms.wTt);
        angular = ang
        linear = lin
        uvms.p
    end
    
    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    SlowdownToRealtime(deltat);
end

fclose(uVehicle);
fclose(uArm);

PrintPlot(plt);

end