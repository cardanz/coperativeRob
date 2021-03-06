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
% lo jacobiano è composto da una parte relativa al braccio e l'altra
% relativa al tool, proiettato sul frame del veicolo 
uvms.Jt = [uvms.Jt_a uvms.Jt_v];

%vehicle frame jacobians  project on the world frame 
%position and attitude
uvms.Jvpos = [zeros(3,7), uvms.wTv(1:3,1:3) zeros(3,3)];
uvms.Jvatt = [zeros(3,7), zeros(3,3)  uvms.wTv(1:3,1:3)];

%vehicle horizontal attitude
w_kw = [0 0 1]'; %kworld rispetto world 
v_kv = [0 0 1]'; %kvehicle rispetto vehicle
v_kw = uvms.vTw(1:3,1:3) * w_kw;

%sfrutto reduced inversion lemma (questo è il disallineamento) 
uvms.v_rho = ReducedVersorLemma(v_kw, v_kv);
%ottengo l'angolo
v_n = uvms.v_rho / norm(uvms.v_rho);

uvms.Jha = [zeros(1,7) zeros(1,3) v_n'];

%vehicle altitude 
%detected distance projected o <v>
v_sen_alt = [0   0 uvms.sensorDistance]';
%detected distance projected o <w> 
w_sen_alt = uvms.wTv(1:3,1:3) * v_sen_alt;
%projection on k direction 
uvms.w_dist = w_kw' * w_sen_alt;

%vehicle altitude jacobian 
uvms.Jvalt = [zeros(1,7) w_kw' * uvms.wTv(1:3,1:3) zeros(1,3)];



end