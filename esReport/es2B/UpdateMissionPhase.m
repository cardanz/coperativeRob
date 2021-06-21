function [uvms, mission] = UpdateMissionPhase(uvms, mission)
    switch mission.phase
        case 1  
            %change mission phase 
            [w_ang, w_lin] = CartError(uvms.wTgvehicle , uvms.wTv);
            if(norm(w_lin) < 0.1)
                mission.phase = 2;
                mission.phase_time = 0;
            end
         case 2
        
    end
end

