function [uvms, mission] = UpdateMissionPhase(uvms, mission)
    switch mission.phase
        case 1  
            %change mission phase 
            [w_vang, w_vlin] = CartError(uvms.wTgvehicle , uvms.wTv);
            if(norm(w_vlin) < 0.1) %10 centimetre
                mission.phase = 2;
                mission.phase_time = 0;
            end
         case 2
             if(uvms.phi < 0.1) 
                mission.phase = 3;
                mission.phase_time = 0;
             end
         case 3
             if(uvms.w_distance < 0.10)
                 mission.phase = 4;
                mission.phase_time = 0;
             end
        case 4
        
    end
end
