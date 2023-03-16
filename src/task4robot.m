function [pos_points1, pos_points2, pos_points3, pos_points4, pos_points5] = task4robot()
    
    open_value = 1024; 
    close_value = 2600; 
    
    thetaG_horizontal = deg2rad(0);
    thetaG_down = deg2rad(-75);
    
    %Default Pose 0.2740,0.0000,0.2048
    default_pos = [0.2740,0.0000,0.2048];
    
    
    % % Go to default position, |▔ pose
    %     pointsList = [pointsList; [default_pos(1), default_pos(2), default_pos(3),thetaG_down, open_value]];
    
    pointsList = [];
    
    
    height_when_holding_sponge = 0.025;  
    
    grab_sponge_z = 0.04; % height when grabbing pen

    sponge_loc = [0, 0.15];

    
    %start pos
    pointsList = [pointsList; [sponge_loc(1), sponge_loc(2), 0.15, thetaG_down, open_value]];
    pointsList = [pointsList; [sponge_loc(1), sponge_loc(2), 0.15, thetaG_down, open_value]];
    
    %pick up sponge
    pointsList = [pointsList; [sponge_loc(1), sponge_loc(2), grab_sponge_z, thetaG_down, open_value]];
    
    pointsList = [pointsList; [sponge_loc(1), sponge_loc(2), grab_sponge_z, thetaG_down, close_value]];
    pointsList = [pointsList; [sponge_loc(1), sponge_loc(2), grab_sponge_z, thetaG_down, close_value]];
    
    
    %lift up pen
    pointsList = [pointsList; [sponge_loc(1), sponge_loc(2), grab_sponge_z+ 0.03, thetaG_down, close_value]];
    
    %First wrap round
    %start
    coords = [ 0.1,0.025];
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_sponge, thetaG_down, close_value]];
    
    %horizontal right line
    coords(2) = coords(2) + 0.150;
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_sponge, thetaG_down, close_value]];  
    
    %downward vertical Line
    coords(1) = coords(1) + 0.025;
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_sponge, thetaG_down, close_value]];
    
    %horizontal left line   
    coords(2) = coords(2) - 0.150;
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_sponge, thetaG_down, close_value]];
    
    %vertial down line
    coords(1) = coords(1) + 0.025;
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_sponge, thetaG_down, close_value]];
    
    %horizontal right line
    coords(2) = coords(2) + 0.150;
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_sponge, thetaG_down, close_value]];
    
    %vertial down line
    coords(1) = coords(1) + 0.025;
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_sponge, thetaG_down, close_value]];
    
    %horizontal left line   
    coords(2) = coords(2) - 0.150;
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_sponge, thetaG_down, close_value]];

    %vertial down line
    coords(1) = coords(1) + 0.025;
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_sponge, thetaG_down, close_value]];

    %horizontal left line   
    coords(2) = coords(2) + 0.150;
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_sponge, thetaG_down, close_value]];
    
%     %%% reverse round
   
     %horizontal right line
    coords(1) = coords(1) -  0.10;
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_sponge, thetaG_down, close_value]];  
    
    %downward horizontal Line
    coords(2) = coords(2) - 0.025;%x
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_sponge, thetaG_down, close_value]];
    
    %horizontal left line   
    coords(1) = coords(1) + 0.10;
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_sponge, thetaG_down, close_value]];
    
    %horizontal Line
    coords(2) = coords(2) - 0.025;%x
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_sponge, thetaG_down, close_value]];
    
    %horizontal right line
    coords(1) = coords(1) -  0.10;
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_sponge, thetaG_down, close_value]];  
    
    %downward horizontal Line
    coords(2) = coords(2) - 0.025;%x
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_sponge, thetaG_down, close_value]];
    
    %horizontal left line   
    coords(1) = coords(1) + 0.10;
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_sponge, thetaG_down, close_value]];
    
    %horizontal Line
    coords(2) = coords(2) - 0.025;%x
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_sponge, thetaG_down, close_value]];
    
     %horizontal right line
    coords(1) = coords(1) -  0.10;
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_sponge, thetaG_down, close_value]];  
    
    %downward horizontal Line
    coords(2) = coords(2) - 0.025;%x
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_sponge, thetaG_down, close_value]];
    
    %horizontal left line   
    coords(1) = coords(1) + 0.10;
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_sponge, thetaG_down, close_value]];
    
    %horizontal Line
    coords(2) = coords(2) - 0.025;%x
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_sponge, thetaG_down, close_value]];
    
    %horizontal left line   
    coords(1) = coords(1) - 0.10;
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_sponge, thetaG_down, close_value]];
    
    number_of_intermediate_points = 25;
    [pos_points1, pos_points2, pos_points3, pos_points4, pos_points5] = cubicInterp_cartesian(pointsList, number_of_intermediate_points);
end