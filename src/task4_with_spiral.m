function   [theta1, theta2, theta3, theta4, gripperList] = task4_with_spiral()
    open_value = 1024; 
    close_value = 2390; 
    
    thetaG_horizontal = deg2rad(0);
    thetaG_down = deg2rad(-85);
    
    %Default Pose 0.2740,0.0000,0.2048
    default_pos = [0.2740,0.0000,0.2048];
    
    
    % % Go to default position, |▔ pose
    %     pointsList = [pointsList; [default_pos(1), default_pos(2), default_pos(3),thetaG_horizontal, open_value]];
    
    pointsList = [];
    
    % line_length = 0.025;
    height_when_holding_pen = 0.070;  %half height of pen, need to adjust for height of pen holder when grab
    
    grab_pen_z = 0.07 + 0.025; % height when grabbing pen
    pen_loc = [0.075, -0.200];
    
    %default pose
    pointsList = [pointsList; [0.2740, 0.0000, grab_pen_z + 0.1, thetaG_horizontal, open_value]];
    pointsList = [pointsList; [0.2740, 0.0000, grab_pen_z + 0.1, thetaG_horizontal, open_value]];
    
    pointsList = [pointsList; [pen_loc(1), pen_loc(2), grab_pen_z + 0.1, thetaG_horizontal, open_value]];
    pointsList = [pointsList; [pen_loc(1), pen_loc(2), grab_pen_z + 0.1, thetaG_horizontal, open_value]];
    
    
    %     %pick up pen 
    pointsList = [pointsList; [pen_loc(1), pen_loc(2), grab_pen_z, thetaG_horizontal, close_value]];
    pointsList = [pointsList; [pen_loc(1), pen_loc(2), grab_pen_z, thetaG_horizontal, close_value]];
    pointsList = [pointsList; [pen_loc(1), pen_loc(2), grab_pen_z, thetaG_horizontal, close_value]];
    pointsList = [pointsList; [pen_loc(1), pen_loc(2), grab_pen_z, thetaG_horizontal, close_value]];
    
    %lift up pen
    pointsList = [pointsList; [pen_loc(1), pen_loc(2), grab_pen_z+ 0.1, thetaG_horizontal, close_value]];
    
    %go to start
    pointsList = [pointsList; [0.12, 0.12, grab_pen_z+ 0.1, thetaG_horizontal, close_value]];
    cur_pos = [ 0.17,0.12];
    
    %% maybe change%%
    %arc
     [x_list, y_list] = circle_spiral(cur_pos,[0.170,0.1], deg2rad(-600), 15, 0.045);
    
    for i = 1:length(x_list)
        pointsList = [pointsList; [x_list(i),  y_list(i), height_when_holding_pen, thetaG_horizontal, close_value]];
    end   
    
    
    cur_pos = [0.170,0.1];
    
    %lift up pen
    pointsList = [pointsList; [cur_pos(1), cur_pos(2), grab_pen_z, thetaG_horizontal, close_value]];
    
    %go above penholder
    pointsList = [pointsList; [pen_loc(1), pen_loc(2), grab_pen_z + 0.1, thetaG_horizontal, close_value]];
    pointsList = [pointsList; [pen_loc(1), pen_loc(2), grab_pen_z + 0.1, thetaG_horizontal, close_value]];
    
    %put down pen
    pointsList = [pointsList; [pen_loc(1), pen_loc(2), grab_pen_z + 0.017, thetaG_horizontal, open_value]];
    pointsList = [pointsList; [pen_loc(1), pen_loc(2), grab_pen_z + 0.017, thetaG_horizontal, open_value]];
    
    
    height_when_holding_sponge = 0.025;  
        
    grab_sponge_z = 0.04; % height when grabbing sponge
    
    sponge_loc = [0, 0.15];
    
    
    %start pos
    pointsList = [pointsList; [sponge_loc(1), sponge_loc(2), 0.15, thetaG_down, open_value]];
    pointsList = [pointsList; [sponge_loc(1), sponge_loc(2), 0.15, thetaG_down, open_value]];
    
    %pick up sponge
    pointsList = [pointsList; [sponge_loc(1), sponge_loc(2), grab_sponge_z, thetaG_down, open_value]];
    
    pointsList = [pointsList; [sponge_loc(1), sponge_loc(2), grab_sponge_z, thetaG_down, close_value]];
    pointsList = [pointsList; [sponge_loc(1), sponge_loc(2), grab_sponge_z, thetaG_down, close_value]];
    
    
    %lift up sponge
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
    coords(2) = coords(2) + 0.125;
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_sponge, thetaG_down, close_value]];
    
    %     %%% reverse round
    
    
     %horizontal right line
    coords = [0.200, 0.175];
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
    
    
    number_of_intermediate_points = 30;
    [theta1, theta2, theta3, theta4, gripperList] = cubicInterp_cartesian(pointsList, number_of_intermediate_points);

end
