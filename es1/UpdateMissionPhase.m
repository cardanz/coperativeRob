function [uvms, mission] = UpdateMissionPhase(uvms, mission)
%controllo solo la fase in cui mi trovo 
    switch mission.phase
        case 1           
            [w_vang, w_vlin] = CartError(uvms.wTgv , uvms.wTv);
            if(norm(w_vlin) < 0.1)
                mission.phase = 2;
                mission.phase_time = 0;
            end
        case 2            
    end
end

