function MainRobust
addpath('./simulation_scripts');
clc;
clear;
close all

% Simulation variables (integration and final time)
deltat = 0.005;
end_time = 25;
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

% rock position 
rock_center = [12.2025   37.3748  -39.8860]'; % in world frame coordinates

% UDP Connection with Unity viewer v2
uArm = udp('127.0.0.1',15000,'OutputDatagramPacketSize',28);
uVehicle = udp('127.0.0.1',15001,'OutputDatagramPacketSize',24);
fopen(uVehicle);
fopen(uArm);
uAltitude = dsp.UDPReceiver('LocalIPPort',15003,'MessageDataType','single');
uAltitude.setup();
% connessione tramide UDP ricordati di accettare la richiesta sulla
% connessione altrimenti non riesce a ricevere i dati

% Preallocation
plt = InitDataPlot(maxloops);

% initialize uvms structure
uvms = InitUVMS('Robust');
% uvms.q 
% Initial joint positions. You can change these values to initialize the simulation with a 
% different starting position for the arm
uvms.q = [-0.0031 0 0.0128 -1.2460 0.0137 0.0853-pi/2 0.0137]'; 
% uvms.p
% initial position of the vehicle
% the vector contains the values in the following order
% [x y z r(rot_x) p(rot_y) y(rot_z)]
% RPY angles are applied in the following sequence
% R(rot_x, rot_y, rot_z) = Rz (rot_z) * Ry(rot_y) * Rx(rot_x)
uvms.p = [10.5 35.5 -36   0 0 pi/2]'; 

% defines the goal position for the end-effector/tool position task
uvms.goalPosition = [12.2025   37.3748  -39.8860]';
uvms.wRg = rotation(0, pi, pi/2);
%goal frame w.r.t world frame
uvms.wTg = [uvms.wRg uvms.goalPosition; 0 0 0 1];

% defines the tool control point
uvms.eTt = eye(4);

%defines the goal position for the vehicle position/attitude task 
% plus 2 meter 
uvms.vgoalPosition = [10.5 37.5 -38   0 0 0]';
uvms.wRgv = rotation(0, 0, 0);
uvms.wTgv = [uvms.wRgv uvms.vgoalPosition; 0 0 0 1];

tic
for t = 0:deltat:end_time
    % update all the involved variables
    % se vuoi aggiungere qualcosa devi farlo durante l'init perch?? nel loop
    % passi e assegni la stessa var
    uvms = UpdateTransforms(uvms);
    uvms = ComputeJacobians(uvms);
    uvms = ComputeTaskReferences(uvms, mission);
    uvms = ComputeActivationFunctions(uvms, mission);
    
    % receive altitude information from unity
    uvms = ReceiveUdpPackets(uvms, uAltitude);
    
    %attenzione come ?? composta: 
    % main kinematic algorithm initialization
    % ydotbar order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    % the vector of the vehicle linear and angular velocities are assumed
    % projected on <v>
    
    ydotbar = zeros(13,1);
    Qp = eye(13); 
    % add all the other tasks here!
    % the sequence of iCAT_task calls defines the priority
    %parametri (funzione di attivazione tool frame, jac tool frame,
    %velocit?? di riferimento)
    % due azioni mi muovo rimanendo in piano e poi muovo il braccio una
    % volta che sono al punto scelto 
    % quindi  prima azione gestione del veicolo, con i primi tre 
    % seconda azione controllo del braccio mantenendo comunque il primo
    % relativo all'impostazione del veicolo 
    
    %task for the altitude
    % [Qp, ydotbar] = iCAT_task(uvms.A.valt,    uvms.Jvalt,    Qp, ydotbar, uvms.xdot.valt,  0.0001,   0.01, 10);
    %
    
    % more important --> 
    %[Qp, ydotbar] = iCAT_task(uvms.A.ha,    uvms.Jha,    Qp, ydotbar, uvms.xdot.ha,  0.0001,   0.01, 10);
    [Qp, ydotbar] = iCAT_task(uvms.A.vpos,    uvms.Jvpos,    Qp, ydotbar, uvms.xdot.vpos,  0.0001,   0.01, 10);
    [Qp, ydotbar] = iCAT_task(uvms.A.vatt,    uvms.Jvatt,    Qp, ydotbar, uvms.xdot.vatt,  0.0001,   0.01, 10);
    
    
    [Qp, ydotbar] = iCAT_task(eye(13),     eye(13),    Qp, ydotbar, zeros(13,1),  0.0001,   0.01, 10);    % this task should be the last one
    
    % get the two variables for integration
    uvms.q_dot = ydotbar(1:7);
    uvms.p_dot = ydotbar(8:13);
    
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
   
    % add debug prints here
    if (mod(t,0.1) == 0)
        %uvms.sensorDistance
        %activ = uvms.A.ha
        

    end

    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    SlowdownToRealtime(deltat);
end

fclose(uVehicle);
fclose(uArm);

PrintPlot(plt);

end