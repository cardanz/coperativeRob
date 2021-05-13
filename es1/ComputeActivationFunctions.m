function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here
    switch mission.phase
        case 1  
            uvms.Aa.vpos = eye(3);
            uvms.Aa.vatt = eye(3);
            uvms.Aa.ha = eye(1);
            uvms.Aa.t = zeros(6);
        case 2
            uvms.Aa.t = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time) * eye(6);
            %uvms.Aa.vpos = DecreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time) * eye(3);
            %cambio l'azione e le priorità lasciando vincolo sulla
            %posizione 
            uvms.Aa.vpos = eye(3);
            uvms.Aa.vatt = DecreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time) * eye(3);
            uvms.Aa.ha = zeros(1);
    end
% arm tool position control
% always active
% moltiplico l'interna per quella che dipende dall'azione 
uvms.A.t = eye(6) * uvms.Aa.t;

%activation function equality 
uvms.A.vpos = eye(3) * uvms.Aa.vpos;
uvms.A.vatt = eye(3) * uvms.Aa.vatt;

%activaction functione angle
uvms.A.ha = IncreasingBellShapedFunction(0.1, 0.2, 0, 1, norm(uvms.v_rho)) * uvms.Aa.ha;