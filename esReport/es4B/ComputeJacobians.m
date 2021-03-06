function [uvms] = ComputeJacobians(uvms)
% compute the relevant Jacobians here
% joint limits
% manipulability
% tool-frame position control
% vehicle-frame position control
% horizontal attitude 
% minimum altitude
% preferred arm posture ( [-0.0031 1.2586 0.0128 -1.2460] )
%
% remember: the control vector is:
% [q_dot; p_dot] 
% [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
% with the vehicle velocities projected on <v>
%
% therefore all task jacobians should be of dimensions
% m x 13
% where m is the row dimension of the task, and of its reference rate

% computation for tool-frame Jacobian
% [omegax_t omegay_t omegaz_t xdot_t ydot_t zdot_t] = Jt ydot
% [angular velocities; linear velocities]
%
% Ste is the rigid body transformation from vehicle-frame to end-effector
% frame projected on <v>
uvms.Ste = [eye(3) zeros(3);  -skew(uvms.vTe(1:3,1:3)*uvms.eTt(1:3,4)) eye(3)];
% uvms.bJe contains the arm end-effector Jacobian (6x7) wrt arm base
% top three rows are angular velocities, bottom three linear velocities
uvms.Jt_a  = uvms.Ste * [uvms.vTb(1:3,1:3) zeros(3,3); zeros(3,3) uvms.vTb(1:3,1:3)] * uvms.bJe;
% vehicle contribution is simply a rigid body transformation from vehicle
% frame to tool frame. Notice that linear and angular velocities are
% swapped due to the different definitions of the task and control
% variables
uvms.Jt_v = [zeros(3) eye(3); eye(3) -skew(uvms.vTt(1:3,4))];
% juxtapose the two Jacobians to obtain the global one
uvms.Jt = [uvms.Jt_a uvms.Jt_v];

%jacobian horizontal attitude
w_kw = [0 0 1]';
v_kv = [0 0 1]';
v_kw = uvms.vTw(1:3,1:3) * w_kw;
uvms.v_rho = ReducedVersorLemma(v_kw, v_kv);
if norm(uvms.v_rho) ~= 0
    v_n = uvms.v_rho/norm(uvms.v_rho);
else
    v_n = [0, 0, 0]';
end 

uvms.Jha = [zeros(1,7), zeros(1,3), v_n'];

%jacobian for vehicle position
uvms.JvehiclePos = [zeros(3,7), uvms.wTv(1:3,1:3), zeros(3,3)];

%jacobian for vehicle attitude
uvms.JvehicleAtt = [zeros(3,7), zeros(3,3), uvms.wTv(1:3,1:3)];

%jacobian for altitude
v_sensorDistance = [0 0 uvms.sensorDistance]';
w_sensorDistance = uvms.wTv(1:3,1:3) * v_sensorDistance;
%scalar product (no projection for scalar value) only to recognize
uvms.w_distance = w_kw' * w_sensorDistance;
uvms.JvehicleAlt = [zeros(1,7) w_kw' * uvms.wTv(1:3, 1:3) zeros(1,3)];

%jacobian for alignment to target 
% rock position 
rock_center = [12.2025   37.3748  -39.8860]';
%distance vector beetween rock and vehicle on world
w_distRockVehicle = rock_center - uvms.wTv(1:3, 4);
w_distRockVehicleProjected = w_distRockVehicle - w_kw * (w_kw' * w_distRockVehicle);
if norm(w_distRockVehicleProjected) ~= 0
    %the vector have some component in xy, calculate the versor
    w_distRockVehiclePlanVersor = w_distRockVehicleProjected/norm(w_distRockVehicleProjected);
else
    w_distRockVehiclePlanVersor = [0, 0, 0]';
end

%for jacobian matrix <v>
v_distRockVehiclePlanVector = uvms.vTw(1:3, 1:3) * w_distRockVehicleProjected;
v_distRockVehiclePlanVersor = uvms.vTw(1:3, 1:3) * w_distRockVehiclePlanVersor;
%misallinement
rho = ReducedVersorLemma([1,0,0]', v_distRockVehiclePlanVersor); 
%avoid error
if norm(rho) ~= 0
    n = rho/norm(rho);
else
    n = [0, 0, 0]';
end    
uvms.theta = norm(rho);
uvms.JvehicleAllignement = n' *  [zeros(3, 7),  -(1 / (norm(v_distRockVehiclePlanVector) * norm(v_distRockVehiclePlanVector))) * skew(v_distRockVehiclePlanVector), -eye(3)];

%jacobian goal in workspace
uvms.targetDistance = rock_center(1:2) - uvms.p(1:2);
uvms.JtargetDistance = [zeros(1,7) -(1/norm(uvms.targetDistance)) * [uvms.targetDistance', 0] * uvms.wTv(1:3, 1:3) zeros(1,3)];
    
%jacobian stop vehicle 
uvms.JvehicleStop = [zeros(6,7), eye(6)];

%jacobian for join limits 
uvms.JjointLimits = [eye(7), zeros(7,6)];

end


