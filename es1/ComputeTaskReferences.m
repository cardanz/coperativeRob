function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = 0.2 * [ang; lin];
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2);

%errore between goal and vehicle projected on <w>
[w_vang, w_vlin] = CartError(uvms.wTgv , uvms.wTv);
uvms.xdot.vpos(1:3,:) = Saturate(0.2 * w_vlin, 0.2);
uvms.xdot.vatt(1:3,:) = Saturate(0.2 * w_vang, 0.2);

%I wish to drive the angle to zero
uvms.xdot.ha = 0.2 * (0 - norm(uvms.v_rho));

%i wish to control the altitude at a given value
uvms.xdot.valt = 0.2 * (5 - uvms.w_dist);