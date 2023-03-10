clc
%clear all
close all

% theta1 = deg2rad(0);
% theta2 = constant + deg2rad(0);
% theta3 = - constant + deg2rad(0);
% theta4 = deg2rad(0);
gamma = atan2(0.024,0.128);% offset angle between link 2 and 3

alpha1 = pi/2;
alpha2 = 0;
alpha3 = 0;
alpha4 = 0;

a1 = 0;
a2 = 0.13;
a3 = 0.124;
a4 = 0.126;

d1 = 0.077;
d2 = 0;
d3 = 0;
d4 = 0;

plot3([0 0], [0 0], [0 0])
xlabel('X')
ylabel('Y')
zlabel('Z')

title("Task 2a Simulation")

grid on
hold on
xlim([-0.3 0.3])
ylim([-0.3 0.3])
zlim([0 0.4])
view(3);


% Cube Holder Coords
% 1. (0.075, -0.200, 0.040)
% 2. (0.225, 0, 0.040)
% 3. (0.150, 0.150, 0.040)
% 4. (0.125, -0.125, 0.050)
% 5. (0.100, 0, 0.050)
% 6. (0, 0.100, 0.050)

open_value = deg2rad(90);
close_value = deg2rad(215);

thetaG_horizontal = deg2rad(0);
thetaG_down = deg2rad(-90);

%Default Pose 0.2740,0.0000,0.2048
default_pos = [0.2740,0.0000,0.2048];

%cube1 to 4, cube2 to 5, cube3 to 6
cube1 = [0.075, -0.200];%cube holder 1 x and y
cube4 = [0.125, -0.125];%cube holder 4 x and y
cube2 = [0.225, 0];
cube5 = [0.100, 0];
cube6 = [0, 0.100];
cube3 = [0.150, 0.150];


cube_grab_top_z = 0.035; % height when grabbing cube from above
cube_grab_side_z = 0.03;%height when grabbing cube from side
safe_dist = 0.070; % safe distance above cube holder

pointsList = [];

start_list = [cube1; cube2; cube3];
end_list = [cube4; cube5; cube6];

thetaG = thetaG_down;
try_side = false;
    
i=1;
while i <= size(start_list,1)
      
      new_pointsList = task2acoordlist(start_list(i, :), end_list(i, :), safe_dist, thetaG_down, cube_grab_top_z, open_value, close_value);
    
        
%       for j=1:size(new_pointsList,1)
%           jointLimitsOk = withinJointLimits(new_pointsList(j,:));
%           invalidIK = isIKInvalid(new_pointsList(j,:));
% 
%           if ~jointLimitsOk || invalidIK
%               try_side = true;
%           end
%       end

%       if try_side
%         new_pointsList = task2acoordlist(start_list(i, 1:2), end_list(i, 1:2), safe_dist, thetaG_horizontal, cube_grab_side_z, open_value, close_value, cur_pointsList);
%       end

      pointsList = [pointsList; new_pointsList];
    
%     % Go to safe distance above cube 
%     point = [start_list(i,1), start_list(i,2), safe_dist, thetaG, open_value];
% 
%     jointLimitsOk = withinJointLimits(point);
%     invalidIK = isIKInvalid(point);
%     if ~jointLimitsOk || invalidIK
%         point(4) = thetaG_horizontal;
%         thetaG = thetaG_horizontal;
%     end
%     
%     pointsList = [pointsList; point];
% 
%     % Grab cube
%     point = [start_list(i,1), start_list(i,2), cube_grab_top_z, thetaG, close_value];
% 
%     jointLimitsOk = withinJointLimits(point);
%     invalidIK = isIKInvalid(point);
%     if ~jointLimitsOk || invalidIK
%         point(4) = thetaG_horizontal;
%         thetaG = thetaG_horizontal;
%         point(3) = cube_grab_side_z;
%     end
%     pointsList = [pointsList; point];
% 
%     %pick cube up
%     point = [start_list(i,1), start_list(i,2), safe_dist, thetaG, close_value];
% 
%     jointLimitsOk = withinJointLimits(point);
%     invalidIK = isIKInvalid(point);
%     if ~jointLimitsOk || invalidIK
%         point(4) = thetaG_horizontal;
%         thetaG = thetaG_horizontal;
%     end
%     pointsList = [pointsList; point];
% 
%     
%     %move to end point
%     point = [end_list(i,1), end_list(i,2), safe_dist, thetaG, close_value];
%     jointLimitsOk = withinJointLimits(point);
%     invalidIK = isIKInvalid(point);
%     if ~jointLimitsOk || invalidIK
%         point(4) = thetaG_horizontal;
%         thetaG = thetaG_horizontal;
%     end
%     pointsList = [pointsList; point];
% 
% 
%     %put cube down
%     point = [end_list(i,1), end_list(i,2), cube_grab_top_z, thetaG, open_value];
%     jointLimitsOk = withinJointLimits(point);
%     invalidIK = isIKInvalid(point);
%     if ~jointLimitsOk || invalidIK
%         point(4) = thetaG_horizontal;
%         thetaG = thetaG_horizontal;
%         point(3) = cube_grab_side_z;
%     end
%     pointsList = [pointsList; point];
% 
%     %move back to safe distance
%     point = [end_list(i,1), end_list(i,2), safe_dist, thetaG, open_value];
%     pointsList = [pointsList; point];

    i = i+1;
end

pointsList

% TODO: actually use open/close_value
        % if joint limits exceeded switch gripper orientation
number_of_intermediate_points = 40;
[pos_points1, pos_points2, pos_points3, pos_points4] = cubicInterp(pointsList, number_of_intermediate_points);

% %default point
% point.x = 0.2740;
% point.y = 0;
% point.z = 0.2048; %default |▔ pose
% point.thetaG = deg2rad(0);
% point.gripPose = open_value;
% 
% pointsList = [pointsList, point]; % append point to pointList
% 
% %go to above cube
% point.x = start_pos(1);
% point.y = start_pos(2);
% point.z = safe_dist;

for i=1:size(pos_points1,2) 
    
    theta1 = pos_points1(i);
    theta2 = pos_points2(i);
    theta3 = pos_points3(i);
    theta4 = pos_points4(i);

    dh_theta2 = theta2 - gamma + pi/2;
    dh_theta3 = theta3 + gamma - pi/2;

    %fk for links
    T0_1 = Transform(alpha1, a1, d1, theta1);
    T1_2 = Transform(alpha2, a2, d2, dh_theta2);
    T2_3 = Transform(alpha3, a3, d3, dh_theta3);
    T3_4 = Transform(alpha4, a4, d4, theta4);
    
    T0_4 = T0_1 * T1_2 * T2_3 * T3_4;
    T0_3 = T0_1 * T1_2 * T2_3;
    T0_2 = T0_1 * T1_2;
    
    link1end = T0_1(1:3,4);
    link2end = T0_2(1:3,4);
    link3end = T0_3(1:3,4);
    end_effector_pos = T0_4(1:3,4);
    end_effector_orientation = T0_4(1:3,1:3);
    
    %remove robot from previous iteration
    if i ~=1
        delete(link1)
        delete(link2)
        delete(link3)
        delete(link4)
    end
    
    %plot robot links
    link1 = drawLink([0,0,0], link1end, 2, 'black');
    link2 = drawLink(link1end, link2end, 2, 'black');
    link3 = drawLink(link2end, link3end, 2, 'black');
    link4 = drawLink(link3end, end_effector_pos, 2, 'black');
        
    if rem(i,10) == 0
        scatter3(end_effector_pos(1), end_effector_pos(2), end_effector_pos(3), 'MarkerEdgeColor', 'blue', 'MarkerFaceColor',[0 .75 .75], 'Marker', 'o');
    end
   
    pause(0.01)   
end

function T = Transform(alpha, a, d, theta)
    
T = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
    sin(theta) cos(theta)*cos(alpha) -cos(theta)*cos(alpha) a*sin(theta);
    0 sin(alpha) cos(alpha) d;
    0 0 0 1];
end

function link = drawLink (start, endpoint, width, colour)
    link = line([start(1) endpoint(1)],[start(2) endpoint(2)],[start(3) endpoint(3)],'LineWidth',width,'Color',colour);
end







