clc
%clear all
close all

gamma = atan2(0.024,0.128);% offset angle between link 2 and 3






% theta1 = deg2rad(0);
% theta2 = constant + deg2rad(0);
% theta3 = - constant + deg2rad(0);
% theta4 = deg2rad(0);
constant = 0;

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

title("Inverse Kinematics")

grid on
hold on
xlim([-0.2 0.2])
ylim([-0.2 0.2])
zlim([0 0.4])


%IK
[theta1, theta2, theta3, theta4] = InverseKinematics(0.2740,0.0000,0.2048, deg2rad(0));
rad2deg(theta1)
rad2deg(theta2)
rad2deg(theta3)
rad2deg(theta4)


dh_theta2 = theta2 - gamma + pi/2;
dh_theta3 = theta3 + gamma - pi/2;
% theta2 = constant + theta2;
% theta3 = - constant + theta3;


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


%plot robot links
link1 = drawLink([0,0,0], link1end, 2, 'black');
link2 = drawLink(link1end, link2end, 2, 'black');
link3 = drawLink(link2end, link3end, 2, 'black');
link4 = drawLink(link3end, end_effector_pos, 2, 'black');
    
 
view(3);
    %view ([0 -90 0]); %X Z Plane
    %view ([0 0 -90 ]); 

function T = Transform(alpha, a, d, theta)
    
T = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
    sin(theta) cos(theta)*cos(alpha) -cos(theta)*cos(alpha) a*sin(theta);
    0 sin(alpha) cos(alpha) d;
    0 0 0 1];
end


function link = drawLink (start, endpoint, width, colour)
    link = line([start(1) endpoint(1)],[start(2) endpoint(2)],[start(3) endpoint(3)],'LineWidth',width,'Color',colour);
end







