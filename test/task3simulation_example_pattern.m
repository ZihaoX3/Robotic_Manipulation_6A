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

title("Task 3 Simulation")

grid on
hold on

%main view
xlim([-0.3 0.3])
ylim([-0.3 0.3])
zlim([0 0.4])
view(3);

%view drawing board
% xlim([0.02 0.12])
% ylim([0.14 0.24])
% zlim([0 0.4])
% view(0,90) 


% Cube Holder Coords (flipped y and x as x flipped since facing robot, correct by -theta1)
% 1. (0.075, -0.200)
% 2. (0.225, 0)
% 3. (0.150, 0.150)
% 4. (0.125, -0.125)
% 5. (0.100, 0)
% 6. (0, 0.100)

open_value = deg2rad(90);
close_value = deg2rad(215);

thetaG_horizontal = deg2rad(0);
thetaG_down = deg2rad(-85);

%Default Pose 0.2740,0.0000,0.2048
default_pos = [0.2740,0.0000,0.2048];



% % Go to default position, |▔ pose
%     pointsList = [pointsList; [default_pos(1), default_pos(2), default_pos(3),thetaG_horizontal, open_value]];

try_down = [];
pointsList = [];

line_length = 0.025;
height_when_holding_pen = 0.07;  %half height of pen, need to adjust for height of pen holder when grab

grab_pen_z = 0.07 + 0.03; % height when grabbing pen
pen_loc = [0.1,-0.1];

%pick up pen 
pointsList = [pointsList; [pen_loc(1), pen_loc(2), grab_pen_z, thetaG_horizontal, open_value]];
pointsList = [pointsList; [pen_loc(1), pen_loc(2), grab_pen_z, thetaG_horizontal, close_value]];

%lift up pen
pointsList = [pointsList; [pen_loc(1), pen_loc(2), grab_pen_z+ 0.03, thetaG_horizontal, close_value]];


%start
cur_pos = [0.060, 0.150];
pointsList = [pointsList; [cur_pos(1), cur_pos(2), height_when_holding_pen, thetaG_horizontal, close_value]];
      
%horizontal Line
cur_pos(2) = cur_pos(2) + line_length;
pointsList = [pointsList; [cur_pos(1), cur_pos(2), height_when_holding_pen, thetaG_horizontal, close_value]];

%vertical line
cur_pos(1) = cur_pos(1) + line_length;                                        
pointsList = [pointsList; [cur_pos(1), cur_pos(2), height_when_holding_pen, thetaG_horizontal, close_value]];

%diagonal upwards to the right line
cur_pos(1) = cur_pos(1) - line_length;
cur_pos(2) = cur_pos(2) + line_length;
pointsList = [pointsList; [cur_pos(1), cur_pos(2), height_when_holding_pen, thetaG_horizontal, close_value]];

%arc
 [x_list, y_list] = circle(cur_pos,[cur_pos(1),cur_pos(2)+0.015], deg2rad(-270), 14);


for i = 1:length(x_list)
    pointsList = [pointsList; [x_list(i),  y_list(i), height_when_holding_pen, thetaG_horizontal, close_value]];
end   
   
   

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

number_of_intermediate_points = 10;
[pos_points1, pos_points2, pos_points3, pos_points4, pos_points5] = cubicInterp_cartesian(pointsList, number_of_intermediate_points);

%plot fk
counter = 0;
counter2 = 0;
for i=1:length(pos_points1)
    
    theta1 = pos_points1(i);
    theta2 = pos_points2(i);
    theta3 = pos_points3(i);
    theta4 = pos_points4(i);
    grip_value = pos_points5(i);

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

     
    if end_effector_pos(3) <= 0
        disp("end effector hit floor")
    end
    
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
        
    if rem(i,1) == 0
        scatter3(end_effector_pos(1), end_effector_pos(2), end_effector_pos(3), 'MarkerEdgeColor', 'blue', 'MarkerFaceColor',[0 .75 .75], 'Marker', 'o');
    end

    %is gripper closed?
    if grip_value == close_value && counter2 == 0 %yes pink
        counter = 0;
        counter2 = counter2+1;
        scatter3(-0.2, -0.2, 0, 'MarkerEdgeColor', 'magenta', 'MarkerFaceColor','magenta', 'Marker', 'o');
    elseif counter == 0 && grip_value == open_value %no blue
        scatter3(-0.2, -0.2, 0, 'MarkerEdgeColor', 'blue', 'MarkerFaceColor','blue', 'Marker', 'o');
        counter = counter +1;
        counter2 =0;
    end
   
    pause(0.0001)   
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







