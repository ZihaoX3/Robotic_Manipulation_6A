function [theta1, theta2, theta3, theta4, gripperList] = task2c_robot()

% Cube Holder Coords (flipped y and x as x flipped since facing robot, correct by -theta1)
% 1. (0.075, -0.200)
% 2. (0.225, 0)
% 3. (0.150, 0.150)
% 4. (0.125, -0.125)
% 5. (0.100, 0)
% 6. (0, 0.100)

open_value = 1024; %deg2rad(90);
close_value = 2446; %deg2rad(215);

thetaG_horizontal = deg2rad(0);
thetaG_down = deg2rad(-80);

%Default Pose 0.2740,0.0000,0.2048
default_pos = [0.2740,0.0000,0.2048];

cube1 = [0.075, -0.200];%cube holder 1 x and y
cube4 = [0.125, -0.125];%cube holder 4 x and y
cube2 = [0.225, 0];
cube5 = [0.100, 0];
cube6 = [0, 0.100];
cube3 = [0.150, 0.150];

start_list = [cube2; cube1; cube3];
end_list = [cube2; cube2; cube2];

% % Go to default position, |▔ pose
%     pointsList = [pointsList; [default_pos(1), default_pos(2), default_pos(3),thetaG_horizontal, open_value]];

try_down = [];
pointsList = [];

safe_dist = 0.0870; % safe distance above cube holder
cube_grab_top_z = 0.057; % height when grabbing cube from above
cube_grab_side_z = 0.050;
mult = 0.95;
current_stack = 0.0500;

%start pos
pointsList = [pointsList; [0.075, -0.200,0.15, thetaG_horizontal, open_value]];

   
%% cube 1 %%
% Go to safe distance above cube 
pointsList = [pointsList; [start_list(1,1), start_list(1,2), safe_dist, thetaG_down, open_value]];

for k = 1:2
    % Grab cube
    pointsList = [pointsList; [start_list(1,1), start_list(1,2), cube_grab_top_z+0.002, thetaG_down, close_value]];
end

%pick cube up
pointsList = [pointsList; [start_list(1,1)/1.15, start_list(1,2)/1.15, safe_dist, thetaG_down, close_value]];

 %only rotate cube 1
%rotate cube
pointsList = [pointsList; [start_list(1,1)/1.15, start_list(1,2)/1.15, 0.2, thetaG_horizontal, close_value]];


%put cube down
pointsList = [pointsList; [start_list(1,1)*mult, start_list(1,2)*mult, current_stack - 0.005, thetaG_horizontal, close_value]];
pointsList = [pointsList; [start_list(1,1)*mult, start_list(1,2)*mult, current_stack - 0.005, thetaG_horizontal, open_value]];%might need to add offset as gripper cant pick up cube in middle when thetaG_down

%move to end point (stops end effector hitting floor/cube holder)
pointsList = [pointsList; [end_list(1,1), end_list(1,2), safe_dist, thetaG_horizontal, open_value]];
pointsList = [pointsList; [end_list(1,1), end_list(1,2),0.15, thetaG_horizontal, open_value]];

current_stack = current_stack + 0.035;

%% cube 2%%
% Go to safe distance above cube 
pointsList = [pointsList; [start_list(2,1), start_list(2,2), safe_dist, thetaG_down, open_value]];

for k = 1:2
    % Grab cube
    pointsList = [pointsList; [start_list(2,1), start_list(2,2), cube_grab_top_z, thetaG_down, close_value]];
end

%pick cube up
pointsList = [pointsList; [start_list(2,1)/1.15, start_list(2,2)/1.15, 0.10, thetaG_down, close_value]];

%move to end point
pointsList = [pointsList; [end_list(2,1), end_list(2,2),0.10, thetaG_down, close_value]];
%put cube down
pointsList = [pointsList; [end_list(2,1)*0.99, end_list(2,2)*0.99, current_stack, thetaG_down, open_value]];
%move back to safe distance
pointsList = [pointsList; [end_list(2,1), end_list(2,2), 0.10, thetaG_down, open_value]];




current_stack = current_stack + 0.025;

%% cube 3%%

%rotate cube
pointsList = [pointsList; [start_list(3,1)/1.15, start_list(3,1)/1.15, 0.2, thetaG_horizontal, open_value]];

% Go to safe distance above cube 
pointsList = [pointsList; [start_list(3,1), start_list(3,2), safe_dist, thetaG_horizontal, open_value]];

for k = 1:2
    % Grab cube
    pointsList = [pointsList; [start_list(3,1), start_list(3,2), cube_grab_side_z, thetaG_horizontal, close_value]];
end

%pick cube up
pointsList = [pointsList; [start_list(3,1)/1.15, start_list(3,2)/1.15, safe_dist, thetaG_horizontal, close_value]];

%move to end point
pointsList = [pointsList; [end_list(3,1), end_list(3,2),0.2, thetaG_horizontal, close_value]];
%put cube down
pointsList = [pointsList; [end_list(3,1), end_list(3,2), current_stack, thetaG_horizontal, open_value]];
%move back to safe distance
pointsList = [pointsList; [end_list(3,1), end_list(3,2), 0.2, thetaG_horizontal, open_value]];
    

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
[theta1, theta2, theta3, theta4, gripperList] = cubicInterp_cartesian(pointsList, number_of_intermediate_points);

end