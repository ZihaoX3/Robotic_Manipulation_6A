function [theta1, theta2, theta3, theta4, gripperList] = task2b_robot()

% Cube Holder Coords
% 1. (0.075, -0.200)
% 2. (0.225, 0)
% 3. (0.150, 0.150)
% 4. (0.125, -0.125)
% 5. (0.100, 0)
% 6. (0, 0.100)

open_value = 1024; %deg2rad(90);
close_value = 2446; %deg2rad(215);
    
thetaG_horizontal = deg2rad(0);
thetaG_down = deg2rad(-85);

%Default Pose 0.2740,0.0000,0.2048
default_pos = [0.2740,0.0000,0.2048];

%cube1 to 4, cube2 to 5, cube3 to 6
cube1 = [0.075, -0.200];%cube holder 1 x and y
cube4 = [0.125, -0.125];%cube holder 4 x and y
cube2 = [0.225, 0];
cube5 = [0.100, 0];
cube6 = [0, 0.100];
cube3 = [0.150, 0.150];


cube_grab_top_z = 0.06; % height when grabbing cube from above
cube_grab_side_z = 0.045;%height when grabbing cube from side
safe_dist = 0.080; % safe distance above cube holder

pointsList = [];

% start_pos = cube1;
% end_pos = cube4;

start_list = [cube1; cube2; cube2; cube3];
end_list = [cube1; cube2;cube2; cube3];
% start_list = [cube2; cube1; cube3];
% end_list = [cube5; cube4; cube6];

% % Go to default position, |▔ pose
%     pointsList = [pointsList; [default_pos(1), default_pos(2), default_pos(3),thetaG_horizontal, open_value]];
i=1;
try_down = [];
pointsList = [];
%start pos
% pointsList = [pointsList; [0, 0.274  ,0.2048, thetaG_horizontal, open_value]];
% while i <= length(start_list)
   
    
      
    % Go to safe distance above cube 
    pointsList = [pointsList; [start_list(1,1), start_list(1,2), safe_dist, thetaG_down, open_value]];
    % Grab cube
    pointsList = [pointsList; [start_list(1,1), start_list(1,2), cube_grab_top_z, thetaG_down, open_value]];

    pointsList = [pointsList; [start_list(1,1), start_list(1,2), cube_grab_top_z, thetaG_down, close_value]];
    pointsList = [pointsList; [start_list(1,1), start_list(1,2), cube_grab_top_z, thetaG_down, close_value]];
    pointsList = [pointsList; [start_list(1,1), start_list(1,2), cube_grab_top_z, thetaG_down, close_value]];
    pointsList = [pointsList; [start_list(1,1), start_list(1,2), cube_grab_top_z, thetaG_down, close_value]];

    %pick cube up
    pointsList = [pointsList; [start_list(1,1)/1.75, start_list(1,2)/1.75, safe_dist, thetaG_down, close_value]];

    %rotate cube
    pointsList = [pointsList; [start_list(1,1)/1.75, start_list(1,2)/1.75, 0.1, thetaG_horizontal, close_value]];
    %put cube down
    pointsList = [pointsList; [start_list(1,1)*0.95, start_list(1,2)*0.95, cube_grab_side_z, thetaG_horizontal, open_value]];%might need to add offset as gripper cant pick up cube in middle when thetaG_down
    pointsList = [pointsList; [start_list(1,1)*0.95, start_list(1,2)*0.95, cube_grab_side_z, thetaG_horizontal, open_value]];
    pointsList = [pointsList; [start_list(1,1)*0.95, start_list(1,2)*0.95, cube_grab_side_z, thetaG_horizontal, open_value]];
    pointsList = [pointsList; [start_list(1,1)*0.95, start_list(1,2)*0.95, cube_grab_side_z, thetaG_horizontal, open_value]];
    %move to end point (stops end effector hitting floor/cube holder)
    pointsList = [pointsList; [end_list(i,1), end_list(i,2), safe_dist, thetaG_horizontal, open_value]];
    
%     i = i+1;
% end

%% switch to down %%
%   for j=1:size(pointsList,1)
%       jointLimitsOk = withinJointLimits(pointsList(j,:));
%       invalidIK = isIKInvalid(pointsList(j,:));
% 
%       if (~jointLimitsOk || invalidIK)
%       %if ( ~jointLimitsOk)
%           try_down(j) = true;
%       else
%           try_down = [try_down, false];
%       end
%   end
%   
% 
%   for k=1:size(pointsList,1)
%       
%       if try_down(k) == true && k <=6
%           
%           for i=1:k
% %               pointsList(k, 3) = cube_grab_top_z;
%               pointsList(i, 4) = thetaG_down;  
%           end
% 
%       elseif try_down(k) == true && k > 6 && k <=12
%           for i=6:k
% %               pointsList(k, 3) = cube_grab_top_z;
%               pointsList(i, 4) = thetaG_down;  
%           end
%       
% 
%       elseif try_down(k) == true && k > 12
%           for i=12:k
% %               pointsList(k, 3) = cube_grab_top_z;
%               pointsList(i, 4) = thetaG_down;  
%           end
%       end
%   end

number_of_intermediate_points = 30;
[theta1, theta2, theta3, theta4, gripperList] = cubicInterp(pointsList, number_of_intermediate_points);

end