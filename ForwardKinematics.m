clc
clear;
close all

constant = - atan2(0.024,0.128) + pi/2;% offset angle between link 2 and 3

% theta_g = deg2rad(-90); % gripper orientation

theta1 = deg2rad(8);
theta2 = constant + deg2rad(-14);
theta3 = - constant + deg2rad(3);
theta4 = deg2rad(-78.8679);    %theta_g - theta2 - theta3;%
disp("theta4 "+ rad2deg(theta4))

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

T0_1 = Transform(alpha1, a1, d1, theta1);
T1_2 = Transform(alpha2, a2, d2, theta2);
T2_3 = Transform(alpha3, a3, d3, theta3);
T3_4 = Transform(alpha4, a4, d4, theta4);

T0_4 = T0_1 * T1_2 * T2_3 * T3_4;
T0_3 = T0_1 * T1_2 * T2_3;
T0_2 = T0_1 * T1_2;
% simplify(T0_4)

link1end = T0_1(1:3,4);
link2end = T0_2(1:3,4);
link3end = T0_3(1:3,4);
end_effector_pos = T0_4(1:3,4);
end_effector_orientation = T0_4(1:3,1:3);

%plot origin frame
line([0 0.03], [0 0], [0 0],'LineWidth',1,'Color','red');
line([0 0], [0 0.03], [0 0],'LineWidth',1,'Color','blue');
line([0 0], [0 0], [0 0.03],'LineWidth',5,'Color','green');

line([link2end(1) link2end(1)+0.03 ],[link2end(2) link2end(2)],[link2end(3) link2end(3)],'LineWidth',1,'Color','magenta');
line([link2end(1) link2end(1) ],[link2end(2) link2end(2)+0.03],[link2end(3) link2end(3)],'LineWidth',1,'Color','blue');
line([link2end(1) link2end(1) ],[link2end(2) link2end(2)],[link2end(3) link2end(3)+0.03],'LineWidth',1,'Color','green');

%plot robot links
line([0 link1end(1)],[0 link1end(2)],[0 link1end(3)],'LineWidth',3,'Color','blue');%link 1

line([link1end(1) link2end(1)], [link1end(2) link2end(2)], [link1end(3) link2end(3)], 'LineWidth',3,'Color','red');%link 2

line([link2end(1) link3end(1)], [link2end(2) link3end(2)], [link2end(3) link3end(3)], 'LineWidth',3,'Color','cyan');%link3

line([link3end(1) end_effector_pos(1)], [link3end(2) end_effector_pos(2)], [link3end(3) end_effector_pos(3)], 'LineWidth',3,'Color','magenta');%link 4


xlabel('x');
ylabel('y');
zlabel('z');
axis('equal');
grid on;

xlim([-0.1 0.3]); 
ylim([-0.1 0.3]);
zlim([-0.1 0.3]);

view(3);
%view ([0 -90 0]); %X Z Plane

function T = Transform(alpha, a, d, theta)
    
T = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
    sin(theta) cos(theta)*cos(alpha) -cos(theta)*cos(alpha) a*sin(theta);
    0 sin(alpha) cos(alpha) d;
    0 0 0 1];
end





