clc
%clear all
close all

gamma = atan2(0.024,0.128);% offset angle between link 2 and 3

%theta_g = deg2rad(90); % gripper orientation

theta1 = deg2rad(00);
theta2 = deg2rad(00);
theta3 = deg2rad(00);
theta4 = deg2rad(0);%theta_g - theta2 - theta3;
%disp("theta4 "+ rad2deg(theta4))

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
xlim([-0.21, 0.21])
ylim([-0.21, 0.21])
zlim([0 0.457])
title("Forward Kinematics")
grid on
hold on

dh_theta2 = theta2 - gamma + pi/2;
dh_theta3 = theta3 + gamma - pi/2;


b=1;
while (b <= 5)
    
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
    

     %remove robot and coord frames from previous iteration
        if b ~=1
            delete(link1)
            delete(link2)
            delete(link3)
            delete(link4)
            delete(frame1)
            delete(frame2)
            delete(frame3)
            delete(frame4)
        end

    %plot robot links
    link1 = drawLink([0,0,0], link1end, 2, 'black');
    link2 = drawLink(link1end, link2end, 2, 'black');
    link3 = drawLink(link2end, link3end, 2, 'black');
    link4 = drawLink(link3end, end_effector_pos, 2, 'black');

    %plot coord frames
    frame1= plotCoordFrame(T0_1);
    frame2 = plotCoordFrame(T0_2);
    frame3 = plotCoordFrame(T0_3);
    frame4 = plotCoordFrame(T0_4);
    
    xlim([-0.1 0.3]); 
    ylim([-0.1 0.3]);
    zlim([0 0.5]);
    
    view(3);
    %view ([0 -90 0]); %X Z Plane
    %view ([0 0 -90 ]); 
    
    %increment angles
    theta1 = theta1 + pi/10;
    theta2 = theta2 + pi/8;
    theta3 = theta3 + pi/16;
    theta4 = theta4 + pi/16;    
    
    %%plot end effector position
    scatter3(end_effector_pos(1), end_effector_pos(2), end_effector_pos(3), 'MarkerEdgeColor', [0 .75 .75], 'MarkerFaceColor',[0 .75 .75], 'Marker', 'o');
    b = b+1;
    pause(0.5)   
end


% close all

function T = Transform(alpha, a, d, theta)
    
T = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
    sin(theta) cos(theta)*cos(alpha) -cos(theta)*cos(alpha) a*sin(theta);
    0 sin(alpha) cos(alpha) d;
    0 0 0 1];
end

function frame = plotCoordFrame(transform)

    x_dir = transform(1:3,1)*0.015;
    y_dir = transform(1:3,2)*0.015;

    z_dir = cross(x_dir,y_dir) *70;
    start = transform(:,4);

    frame(1) = plot3([start(1) start(1)+x_dir(1)],[start(2) start(2)+x_dir(2)],[start(3) start(3)+x_dir(3)],'LineWidth',2,'Color','red');
    frame(2) = plot3([start(1) start(1)+y_dir(1)],[start(2) start(2)+y_dir(2)],[start(3) start(3)+y_dir(3)],'LineWidth',2,'Color','green');
    frame(3) = plot3([start(1) start(1)+z_dir(1)],[start(2) start(2)+z_dir(2)],[start(3) start(3)+z_dir(3)],'LineWidth',2,'Color','blue');
end

function link = drawLink (start, endpoint, width, colour)
    link = line([start(1) endpoint(1)],[start(2) endpoint(2)],[start(3) endpoint(3)],'LineWidth',width,'Color',colour);
end







