function [pos_points1, pos_points2, pos_points3, pos_points4, pos_points5] =  task3robot()

    open_value = 1024; 
    close_value = 2446; 
    
    thetaG_horizontal = deg2rad(0);
    thetaG_down = deg2rad(-85);
    
    %Default Pose 0.2740,0.0000,0.2048
    default_pos = [0.2740,0.0000,0.2048];
    
    
    % % Go to default position, |▔ pose
    %     pointsList = [pointsList; [default_pos(1), default_pos(2), default_pos(3),thetaG_horizontal, open_value]];
    
    pointsList = [];
    
    % line_length = 0.025;
    height_when_holding_pen = 0.07;  %half height of pen, need to adjust for height of pen holder when grab
    
    grab_pen_z = 0.07 + 0.03; % height when grabbing pen
    pen_loc = [0.075, -0.200];
    
    pointsList = [pointsList; [pen_loc(1), pen_loc(2), grab_pen_z + 0.12, thetaG_horizontal, open_value]];
%     %pick up pen 
    pointsList = [pointsList; [pen_loc(1), pen_loc(2), grab_pen_z, thetaG_horizontal, open_value]];
    pointsList = [pointsList; [pen_loc(1), pen_loc(2), grab_pen_z, thetaG_horizontal, close_value]];
    
    %lift up pen
    pointsList = [pointsList; [pen_loc(1), pen_loc(2), grab_pen_z+ 0.1, thetaG_horizontal, close_value]];
    
    
    %start
    coords = [0.200, 0.060];
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_pen, thetaG_horizontal, close_value]];
    
    %diagonal upwards to the right line
    coords = [ 0.125,0.140];
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_pen, thetaG_horizontal, close_value]];  
    
    %downward vertical Line
    coords = [ 0.2,0.140];
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_pen, thetaG_horizontal, close_value]];
    
    %horizontal left line   
    coords = [ 0.2,0.060];
    pointsList = [pointsList; [coords(1), coords(2), height_when_holding_pen, thetaG_horizontal, close_value]];
    
    cur_pos = [ 0.200,0.060];%- x coord as facing robot
    
    %arc
     [x_list, y_list] = circle(cur_pos,[0.2,0.1], deg2rad(-180), 14);%- x coord as facing robot
    
    for i = 1:length(x_list)
        pointsList = [pointsList; [x_list(i),  y_list(i), height_when_holding_pen, thetaG_horizontal, close_value]];
    end   
   

    number_of_intermediate_points = 10;
    [pos_points1, pos_points2, pos_points3, pos_points4, pos_points5] = cubicInterp_cartesian(pointsList, number_of_intermediate_points);

end
