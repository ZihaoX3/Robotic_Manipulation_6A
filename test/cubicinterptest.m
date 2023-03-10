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

title("Interp Test")

grid on
hold on
xlim([-0.3 0.3])
ylim([-0.3 0.3])
zlim([0 0.4])
view(3);

%generate points
point1 = [0.075, -0.200,0.070];%[0.1,0.1,0.1];
point2 = [0.075, -0.200,0.070];

thetaG_horizontal = deg2rad(0);
thetaG_down = deg2rad(-89.6);

[theta] = InverseKinematics2(point1(1),point1(2),point1(3),thetaG_down);
[thetaf] = InverseKinematics2(point2(1),point2(2),point2(3),thetaG_horizontal);

theta1 = theta(1);
theta2 = theta(2);
theta3 = theta(3);
theta4 = theta(4);

theta1f = thetaf(1);
theta2f = thetaf(2);
theta3f = thetaf(3);
theta4f = thetaf(4);

m = 50;

[q1, qd1, qdd1] = cubicTrajectoryPlanning(m, theta1, theta1f, 0, 0);
[q2, qd2, qdd2] = cubicTrajectoryPlanning(m, theta2, theta2f, 0, 0);
[q3, qd3, qdd3] = cubicTrajectoryPlanning(m, theta3, theta3f, 0, 0);
[q4, qd4, qdd4] = cubicTrajectoryPlanning(m, theta4, theta4f, 0, 0);

for i=1:length(q1)

    theta1 = q1(i);
    theta2 = q2(i);
    theta3 = q3(i);
    theta4 = q4(i);

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

    scatter3(end_effector_pos(1), end_effector_pos(2), end_effector_pos(3), 'MarkerEdgeColor', 'blue', 'MarkerFaceColor',[0 .75 .75], 'Marker', 'o');
   
    pause(0.02)   
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







