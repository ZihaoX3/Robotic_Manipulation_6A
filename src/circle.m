function [x_list, y_list] = circle(start, mid, angle, steps)
  % -ve angle to draw clockwise
  % angle is angle to draw circle until
  % mid is centre of circle
  %steps is how many points to generate

% start = [0.060, 0.150];
% mid = [0.065, 0.150];
% angle = deg2rad(-270);
% steps = 100;

     
     
%     theta = linspace(0, 2*pi*1, 100);
%     radius = 0.05;
    
    
    x_list = [];
    y_list= [];
    
    radius = sqrt((start(1) - mid(1))^2 + (start(2) - mid(2)) ^2);
    dif = start - mid;
    adjust_angle = atan2(dif(2), dif(1));
    
    theta = linspace(adjust_angle, angle + adjust_angle, steps);
    points_circle = [];
    
    for i = 1:size(theta,2)
        x = radius*cos(theta(i)) + mid(1);
        y = radius*sin(theta(i)) + mid(2);
        points_circle = [points_circle; [x, y]];
        x_list = [x_list, x];
        y_list = [y_list, y];
        
    end

%     length(y_list)
%     length(x_list)
end